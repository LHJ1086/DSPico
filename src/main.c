// ---------------------------------------------------------------------------
// DSPico — RP2350 USB-to-USB parametric-EQ bridge.
//
// Phase 0 + Phase 1 firmware (brief §8). Core split (brief §4):
//   * core1: Pico-PIO-USB HOST  -> downstream USB DAC (tight bit timing)
//   * core0: native USB DEVICE (UAC2 speaker) + app/DSP
//
// The full bridge is wired: the device side enumerates as a UAC2 output, and
// PC audio is EQ'd and streamed on to the downstream DAC (audio_source below).
// A firmware-generated test tone feeds the DAC whenever no PC is streaming, so
// the DAC stays fed for bring-up with nothing attached upstream.
// ---------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/structs/powman.h"

#include "pio_usb.h"
#include "tusb.h"

#include "board_config.h"
#include "debug_log.h"
#include "status_led.h"
#include "uac2_device.h"
#include "uac_host.h"
#include "test_tone.h"
#include "signal_path.h"
#include "config_usb.h"

// Enable FPU flush-to-zero + default-NaN on the CALLING core (FPSCR is
// per-core, so each core that touches float must call this once). Denormal
// operands cost many extra cycles on the M33 FPU, and the SVF integrator state
// (dsp_peq.c) decays through the denormal range during silence — without FZ a
// quiet passage can spike the DSP time and threaten audio deadlines.
static void fpu_enable_flush_to_zero(void) {
  uint32_t fpscr;
  __asm volatile ("vmrs %0, fpscr" : "=r" (fpscr));
  fpscr |= (1u << 24) | (1u << 25);   // FZ | DN
  __asm volatile ("vmsr fpscr, %0" : : "r" (fpscr));
}

// The audio the host streams to the DAC: EQ'd PC audio from the play ring when
// the PC is streaming, otherwise the firmware test tone (keeps the DAC fed and
// lets the Phase 1b gate run with no PC attached). While the PC *is* streaming
// but the ring is priming or momentarily underrun, fill with silence — a gap
// must not blast the test tone into the music.
static size_t audio_source(uint8_t *dst, size_t max_frames) {
  size_t frames = signal_path_pull_play(dst, max_frames);
  if (frames == 0) {
    if (uac2_is_streaming()) {
      memset(dst, 0, max_frames * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE);
      frames = max_frames;
    } else {
      frames = test_tone_fill(dst, max_frames);   // returns frames
    }
  }
  return frames;
}

// ---------------------------------------------------------------------------
// core1 — PIO-USB host
// ---------------------------------------------------------------------------
static void core1_main(void) {
  fpu_enable_flush_to_zero();   // core1 runs float too (test tone sinf)

  // The PIO-USB host is configured AND serviced on the core that owns its
  // timing. Configure the data pins for this board's variant (board_config.h).
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = DSPICO_PIO_USB_DP_PIN;      // D+ (variant-specific, board_config.h)
#if DSPICO_PIO_USB_PINOUT_DPDM_SWAP
  pio_cfg.pinout = PIO_USB_PINOUT_DMDP;        // -CM: D- is the lower pin
#else
  pio_cfg.pinout = PIO_USB_PINOUT_DPDM;        // -C:  D- is the higher pin
#endif

  tuh_configure(DSPICO_RHPORT_HOST, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
  tuh_init(DSPICO_RHPORT_HOST);

  // Allow core0 to briefly freeze this core during flash writes (config commit).
  multicore_lockout_victim_init();

  for (;;) {
    tuh_task();
    uac_host_task();   // watchdog for DAC control transfers that NAK forever
  }
}

// ---------------------------------------------------------------------------
// core0 — device stack + app
// ---------------------------------------------------------------------------
int main(void) {
  fpu_enable_flush_to_zero();   // core0 runs the PEQ (see helper above)

  // Pico-PIO-USB requires a 120 MHz-derived system clock for correct Full-Speed
  // timing (brief §4). Do this BEFORE bringing up either USB stack.
  set_sys_clock_khz(DSPICO_SYS_CLK_KHZ, true);

  stdio_init_all();                 // debug over UART (see CMakeLists)
  status_led_init();

  // Boot banner with the RESET CAUSE — the decisive datapoint for "is the
  // board silently resetting?": repeated BROWNOUT lines mean the supply is
  // sagging (e.g. a bus-powered DAC pulling the shared rail down); repeated
  // banners of any kind while in use mean the board is crash-looping.
  const uint32_t rst = powman_hw->chip_reset;
  dlog0("\nDSPico boot — reset cause:%s%s%s%s%s (0x%08lx), sysclk=%lu kHz\n",
        (rst & POWMAN_CHIP_RESET_HAD_POR_BITS)     ? " power-on"  : "",
        (rst & POWMAN_CHIP_RESET_HAD_BOR_BITS)     ? " BROWNOUT"  : "",
        (rst & POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS) ? " run/reset-pin" : "",
        (rst & (POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_RSM_BITS |
                POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE_BITS |
                POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_BITS)) ? " watchdog" : "",
        (rst & POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS) ? " debugger" : "",
        (unsigned long) rst, (unsigned long) DSPICO_SYS_CLK_KHZ);

  // On-device signal path (PEQ engine + play ring). Flat by default; the UAC2
  // volume callback and the WebUSB config handler configure it live.
  signal_path_init();
  config_usb_init();          // restore any flash-persisted EQ preset

  // Example: hard-code an EQ preset until the WebUSB configurator lands. This is
  // the brief's acceptance-test filter (a -6 dB dip @ 1 kHz with -6 dB pre-gain).
  // Uncomment to hear/measure the on-device EQ working end-to-end.
  //
  // signal_path_set_pre_gain_db(-6.0f);
  // peq_band_t demo = { .enabled = true, .type = PEQ_PEAKING,
  //                     .fc = 1000.0f, .gain_db = -6.0f, .q = 1.0f };
  // signal_path_set_band(0, &demo);

  // Idle/gate fallback tone, and the DAC audio source.
  test_tone_config(1000.0f, -6.0f);
  uac_host_set_source(audio_source);

  // Launch the PIO-USB host on core1.
  multicore_reset_core1();
  sleep_ms(10);
  multicore_launch_core1(core1_main);

  // Native USB device stack on core0.
  tud_init(DSPICO_RHPORT_DEVICE);

  led_state_t shown = LED_IDLE;
  status_led_set(shown);

  for (;;) {
    tud_task();
    uac2_feedback_task();// async feedback from the play-ring fill (uac2_device.c)
    uac2_device_task();  // periodic device stream/RX status heartbeat
    config_usb_task();   // deferred flash commit (too slow for a USB callback)
    dlog_task();         // forward core1 host diagnostics to UART + WebUSB log

    // Glanceable status, host side first (that's where bring-up problems
    // live): green = streaming, red = DAC attached but unusable/failed,
    // cyan = DAC negotiating, blue = PC connected (no DAC), amber = nothing.
    led_state_t want;
    switch (uac_host_state()) {
      case UAC_HOST_STREAMING:    want = LED_STREAM;    break;
      case UAC_HOST_INCOMPATIBLE: want = LED_ERROR;     break;
      case UAC_HOST_SETUP:        want = LED_DAC_SETUP; break;
      default:                    want = tud_mounted() ? LED_IDLE : LED_SEARCH;
    }
    if (want != shown) {
      shown = want;
      status_led_set(shown);
    }
  }
}
