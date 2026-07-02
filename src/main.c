// ---------------------------------------------------------------------------
// DSPico — RP2350 USB-to-USB parametric-EQ bridge.
//
// Phase 0 + Phase 1 firmware (brief §8). Core split (brief §4):
//   * core1: Pico-PIO-USB HOST  -> downstream USB DAC (tight bit timing)
//   * core0: native USB DEVICE (UAC2 speaker) + app/DSP
//
// Phase 1b streams a firmware-generated test tone to the DAC; the device side
// enumerates as a UAC2 output and discards received audio. Stop here for the
// hardware go/no-go gate before wiring the two together (Phase 2).
// ---------------------------------------------------------------------------
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"

#include "pio_usb.h"
#include "tusb.h"

#include "board_config.h"
#include "status_led.h"
#include "uac2_device.h"
#include "uac_host.h"
#include "test_tone.h"

// Adapt the tone generator to the host driver's fill-callback signature.
static size_t test_tone_source(uint8_t *dst, size_t max_frames) {
  test_tone_fill(dst, max_frames);
  return max_frames;
}

// ---------------------------------------------------------------------------
// core1 — PIO-USB host
// ---------------------------------------------------------------------------
static void core1_main(void) {
  // The PIO-USB host is configured AND serviced on the core that owns its
  // timing. Configure the data pins for this board's reversed layout (brief §3b).
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = DSPICO_PIO_USB_DP_PIN;      // GPIO13 = D+
#if DSPICO_PIO_USB_PINOUT_DPDM_SWAP
  pio_cfg.pinout = PIO_USB_PINOUT_DMDP;        // D- is the lower pin (GPIO12)
#else
  pio_cfg.pinout = PIO_USB_PINOUT_DPDM;
#endif

  tuh_configure(DSPICO_RHPORT_HOST, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
  tuh_init(DSPICO_RHPORT_HOST);

  for (;;) {
    tuh_task();
  }
}

// ---------------------------------------------------------------------------
// core0 — device stack + app
// ---------------------------------------------------------------------------
int main(void) {
  // Pico-PIO-USB requires a 120 MHz-derived system clock for correct Full-Speed
  // timing (brief §4). Do this BEFORE bringing up either USB stack.
  set_sys_clock_khz(DSPICO_SYS_CLK_KHZ, true);

  stdio_init_all();                 // debug over UART (see CMakeLists)
  status_led_init();

  printf("\nDSPico — Phase 0/1 firmware, sysclk=%lu kHz\n", (unsigned long) DSPICO_SYS_CLK_KHZ);

  // Phase 1b source: a 1 kHz, -6 dBFS tone streamed to the DAC.
  test_tone_config(1000.0f, -6.0f);
  uac_host_set_source(test_tone_source);

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

    // Coarse status: streaming to DAC > enumerated device > searching.
    led_state_t want;
    if (uac_host_is_streaming())      want = LED_STREAM;
    else if (tud_mounted())           want = LED_IDLE;
    else                              want = LED_SEARCH;
    if (want != shown) {
      shown = want;
      status_led_set(shown);
    }
  }
}
