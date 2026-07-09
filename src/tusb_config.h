// ---------------------------------------------------------------------------
// DSPico — TinyUSB configuration
//
// This board runs TWO TinyUSB stacks at once (brief §4):
//   * DEVICE stack on rhport 0 = native USB controller (Type-C1 -> PC)
//     exposing a UAC2 speaker (playback only) with a Feature Unit.
//   * HOST stack on rhport 1 = Pico-PIO-USB software controller (Type-C2 -> DAC)
//     which we drive with a small *custom* audio-host class driver, because
//     TinyUSB ships no UAC host class (brief §7 — this is THE risk).
//
// Targeted against the TinyUSB bundled with Pico SDK 2.1.1. If you bump the SDK,
// re-check the CFG_TUD_AUDIO_* macro names against your tree during Phase 1a.
// ---------------------------------------------------------------------------
#ifndef DSPICO_TUSB_CONFIG_H
#define DSPICO_TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board_config.h"
#include "usb_descriptors.h"   // for DSPICO_UAC2_DESC_TOTAL_LEN (pure macros)

// ---------------------------------------------------------------------------
// Common
// ---------------------------------------------------------------------------
// CFG_TUSB_MCU / CFG_TUSB_OS are injected by the Pico SDK build for the RP2350
// family; do not hard-code them here.
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_PICO
#endif

// Bring-up diagnostic: route TinyUSB's DEVICE-side internal trace (SET_INTERFACE,
// endpoint open, control requests) into the WebUSB log so we can see whether the
// PC ever activates the audio stream and, if it fails, where. The sink
// (dspico_tusb_printf) drops HOST-stack (core1) trace entirely, protecting the
// timing-critical PIO-USB path. DSPICO_USB_TRACE is defined in board_config.h
// (included above); set it to 0 there to return to a quiet production build.
// The verbose TinyUSB internal trace served its purpose (it showed the PC does
// reach SET_INTERFACE(alt 1)) but floods the log. Keep DSPICO_USB_TRACE for the
// host-side log-silencing gates in uac_host.c, but drop the verbose device
// trace — our own targeted markers in uac2_device.c are enough now and far
// quieter. Flip USE_TINYUSB_VERBOSE_TRACE to 1 to bring the full trace back.
#define USE_TINYUSB_VERBOSE_TRACE 0
#if USE_TINYUSB_VERBOSE_TRACE
#undef  CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG      2
#define CFG_TUD_LOG_LEVEL   2      // device stack: verbose (what we want to see)
#define CFG_TUH_LOG_LEVEL   0      // host stack: silent (protect core1 timing)
extern int dspico_tusb_printf(const char *fmt, ...);
#define CFG_TUSB_DEBUG_PRINTF dspico_tusb_printf
#elif defined(DSPICO_HOST_ENUM_TRACE) && DSPICO_HOST_ENUM_TRACE
// Optional deep host-enumeration trace (level 1): TinyUSB's own milestones and
// errors ("Get Configuration Descriptor", "STALLED", "Enumeration attempt N")
// routed to the WebUSB log via the non-blocking cross-core ring. Invaluable for
// a DAC that fails BEFORE our driver sees it (a pre-config STALL), but it also
// prints per iso transfer, which floods the log during streaming and drowns
// our own markers — so it is OFF by default and only flipped on for a
// bring-up session (-DDSPICO_HOST_ENUM_TRACE=1). Our own dlog markers in
// uac_host.c (device attach, VID:PID, every format candidate with its
// accept/reject reason, setup steps) stay on always and cover the common cases
// without any flooding.
#undef  CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG      1
#define CFG_TUD_LOG_LEVEL   0
#define CFG_TUH_LOG_LEVEL   1
extern int dspico_tusb_printf(const char *fmt, ...);
#define CFG_TUSB_DEBUG_PRINTF dspico_tusb_printf
#else
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif
#endif

// Both native and PIO ports are Full Speed only (brief §2).
#define BOARD_TUD_RHPORT     DSPICO_RHPORT_DEVICE
#define BOARD_TUH_RHPORT     DSPICO_RHPORT_HOST
#define BOARD_TUD_MAX_SPEED  OPT_MODE_FULL_SPEED
#define BOARD_TUH_MAX_SPEED  OPT_MODE_FULL_SPEED

// DMA-capable, 4-byte aligned section for USB buffers.
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

// ---------------------------------------------------------------------------
// DEVICE stack (rhport 0, native USB -> PC)
// ---------------------------------------------------------------------------
#define CFG_TUD_ENABLED       1
#define CFG_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

// Control endpoint 0 packet size.
#define CFG_TUD_ENDPOINT0_SIZE 64

// Class driver counts. Phase 1 = audio only. The WebUSB vendor interface
// (brief §4/§6b) is added in Phase 3; flip CFG_TUD_VENDOR then.
#define CFG_TUD_AUDIO   1
#define CFG_TUD_CDC     0
#define CFG_TUD_MSC     0
#define CFG_TUD_HID     0
#define CFG_TUD_MIDI    0
#define CFG_TUD_VENDOR  1        // WebUSB EQ config channel (brief §6b)

// Vendor bulk FIFOs — the config protocol is control-only, so keep them small.
#define CFG_TUD_VENDOR_RX_BUFSIZE 64
#define CFG_TUD_VENDOR_TX_BUFSIZE 64

// --- UAC2 speaker (OUT only) parameters ------------------------------------
// One Audio Streaming interface (the OUT/speaker path). No capture (brief §1).
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT     1

// Total length of the audio function's interface descriptor block. Defined next
// to the descriptor itself so the two can never drift apart.
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN     DSPICO_UAC2_DESC_TOTAL_LEN

// Control request scratch buffer (get/set on Clock Source + Feature Unit).
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ  64

// We are an audio SINK: only the OUT endpoint exists.
#define CFG_TUD_AUDIO_ENABLE_EP_OUT       1
#define CFG_TUD_AUDIO_ENABLE_EP_IN        0

// PC-facing RX format is 24-bit (see board_config.h) — same width as the
// internal path, so RX pushes samples straight through with no conversion.
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX          DSPICO_NUM_CHANNELS
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX  DSPICO_DEV_BYTES_PER_SAMPLE

// Max OUT endpoint payload. For an asynchronous sink the host may send one
// extra sample per frame, so budget (samples/ms + 1).
#define CFG_TUD_AUDIO_EP_SZ_OUT \
  ((DSPICO_SAMPLES_PER_MS + 1) * DSPICO_NUM_CHANNELS * DSPICO_DEV_BYTES_PER_SAMPLE)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX     CFG_TUD_AUDIO_EP_SZ_OUT

// Software FIFO behind the OUT endpoint (a couple of frames of slack).
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ  (CFG_TUD_AUDIO_EP_SZ_OUT * 4)

// --- Explicit feedback endpoint (async) ------------------------------------
// The DAC is the clock master; we report our fill level so the PC slaves its
// send rate to us (brief §5). The WIRE FORMAT of the feedback value is picked
// at runtime by tud_audio_feedback_format_correction_cb (uac2_device.c):
// 10.14/3-byte for macOS/iOS/Linux, 16.16/4-byte for Windows (fingerprinted
// by its MS OS 2.0 request) — usbaudio2.sys can't read the 10.14 form and
// the Windows audio engine hangs on it. The macro below is only the weak
// default; the callback supersedes it.
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP                 1
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION  1

// ---------------------------------------------------------------------------
// HOST stack (rhport 1, Pico-PIO-USB -> DAC)
// ---------------------------------------------------------------------------
#define CFG_TUH_ENABLED       1
#define CFG_TUH_MAX_SPEED     OPT_MODE_FULL_SPEED

// Select the Pico-PIO-USB software host controller for rhport 1.
#define CFG_TUH_RPI_PIO_USB   1

// Built-in host class drivers we do NOT use (the DAC is handled by our own
// application host driver, see uac_host.c).
#define CFG_TUH_CDC   0
#define CFG_TUH_HID   0
#define CFG_TUH_MSC   0
#define CFG_TUH_VENDOR 0

// Allow a DAC that sits behind a hub, plus a small device table.
#define CFG_TUH_HUB          1
#define CFG_TUH_DEVICE_MAX   (CFG_TUH_HUB ? 4 : 1)
// TinyUSB fetches the device's ENTIRE configuration descriptor into this
// buffer during enumeration; if the descriptor is larger, enumeration FAILS
// before the device ever mounts (address assigned, then dropped — the exact
// "device N removed with no attach line" symptom seen with a Cirrus Logic
// DAC). Multi-rate / multi-format UAC2 codecs (Cirrus, ESS, AKM) carry many
// alternate settings and easily exceed 512 bytes, so budget generously —
// RP2350 has the RAM. This also must hold the whole audio-function block or
// uac_host.c's format parser can miss a valid alt setting.
#define CFG_TUH_ENUMERATION_BUFSIZE 4096

// Enable the raw endpoint transfer API — our custom driver uses
// usbh_edpt_open()/usbh_edpt_xfer() to open and pump the DAC's isochronous
// OUT endpoint (brief §7).
#define CFG_TUH_API_EDPT_XFER 1

#ifdef __cplusplus
}
#endif

#endif // DSPICO_TUSB_CONFIG_H
