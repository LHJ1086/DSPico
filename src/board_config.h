// ---------------------------------------------------------------------------
// DSPico — board configuration for the Waveshare RP2350-USB-C / RP2350-USB-CM
//
// All hardware-specific facts (pins, clock, USB roles) live here so the rest of
// the firmware never hard-codes a magic pin number. Values are taken from the
// board schematics.
//
// The two variants are identical EXCEPT the PIO-USB port: their D+/D- GPIO
// assignments are swapped (verified on the RP2350-USB-C schematic: GPIO12 ->
// 27R -> DB_P/D+, GPIO13 -> 27R -> DB_N/D-, and the populated 1.5k D+ pull-up
// R13 sits on the GPIO12 net). Default build = RP2350-USB-C; configure with
// -DDSPICO_BOARD_USB_CM=ON for the -CM variant.
// ---------------------------------------------------------------------------
#ifndef DSPICO_BOARD_CONFIG_H
#define DSPICO_BOARD_CONFIG_H

// Bring-up USB trace (see tusb_config.h). When on, the host-side (core1) log
// bursts — the DAC descriptor dump and per-stream heartbeat — are silenced so
// they don't byte-interleave with and garble the device-side (core0) trace we
// are trying to read. Set to 0 for a normal build.
#ifndef DSPICO_USB_TRACE
#define DSPICO_USB_TRACE 0
#endif

// --- System clock ----------------------------------------------------------
// Pico-PIO-USB requires a 120 MHz-derived clock for correct Full-Speed bit
// timing. Do NOT change this to an arbitrary frequency (brief §4). 240 MHz is
// also valid; 120 keeps power/heat down and is the reference-tested value.
#define DSPICO_SYS_CLK_KHZ (120 * 1000)

// --- USB role / port assignment (brief §2) ---------------------------------
// Native USB 1.1 controller (Type-C1)  -> TinyUSB DEVICE  -> rhport 0 -> PC.
// PIO-USB software controller (Type-C2) -> TinyUSB HOST    -> rhport 1 -> DAC.
#define DSPICO_RHPORT_DEVICE 0
#define DSPICO_RHPORT_HOST   1

// --- PIO-USB data pins (Type-C2) — VARIANT-SPECIFIC --------------------------
// The D+/D- GPIO assignment is SWAPPED between the two board variants:
//
//   RP2350-USB-C  (default):  D+ = GPIO12,  D- = GPIO13   (DPDM, no swap)
//   RP2350-USB-CM (-DDSPICO_BOARD_USB_CM=ON): D+ = GPIO13, D- = GPIO12 (DMDP)
//
// Pico-PIO-USB's config takes the D+ pin; the swap flag selects whether D- is
// DP+1 (DPDM) or DP-1 (DMDP). Getting this wrong drives the bus with inverted
// polarity and the downstream DAC will never enumerate — flash the right
// variant before suspecting hardware. Note: the ~1.5 kOhm D+ pull-up may need
// removing for host mode — see docs/hardware-fix.md (trust the net, not the
// label: the populated part is R13 on -C, R10 on -CM, always D+ -> 3V3).
#ifndef DSPICO_BOARD_USB_CM
#define DSPICO_BOARD_USB_CM 0
#endif

#if DSPICO_BOARD_USB_CM
#define DSPICO_PIO_USB_DP_PIN 13            // -CM: GPIO13 = D+
#define DSPICO_PIO_USB_PINOUT_DPDM_SWAP 1   //      D- = DP-1 = GPIO12 (DMDP)
#else
#define DSPICO_PIO_USB_DP_PIN 12            // -C:  GPIO12 = D+
#define DSPICO_PIO_USB_PINOUT_DPDM_SWAP 0   //      D- = DP+1 = GPIO13 (DPDM)
#endif

// --- Status LED (brief §2) -------------------------------------------------
// WS2812B (NeoPixel) on GPIO16. Single pixel used as a coarse state indicator.
#define DSPICO_WS2812_PIN 16

// --- Audio format ----------------------------------------------------------
// Internal DSP path + DAC (host) side: 24-bit. The PEQ runs in float and the
// play ring / DAC packets are 24-bit packed, so we keep full headroom through
// the EQ and can feed a 24-bit DAC alt when one is chosen.
#define DSPICO_SAMPLE_RATE_HZ   48000u
#define DSPICO_NUM_CHANNELS     2u
#define DSPICO_BYTES_PER_SAMPLE 3u        // 24-bit packed (internal + DAC side)
#define DSPICO_RESOLUTION_BITS  24u

// PC-facing (device) wire format: 24-bit, matching the internal path and the
// DAC side for a TRUE bit-perfect 24-bit chain end-to-end (PC -> EQ -> DAC)
// with no width conversion anywhere when the EQ is flat. 16-bit was used
// earlier because Windows' shared-mode engine is fussy about 24-bit-only
// devices, but Apple (the primary target) handles 24/48 UAC2 natively, and a
// 16-bit wire was capping resolution and audibly muddying hi-res material.
// RX now pushes the received 24-bit samples straight into the signal path.
#define DSPICO_DEV_BYTES_PER_SAMPLE 3u
#define DSPICO_DEV_RESOLUTION_BITS  24u

// One 1 ms USB frame's worth of audio (nominal). At 48 kHz that is 48
// samples/frame/channel. We size ring buffers generously around this.
#define DSPICO_SAMPLES_PER_MS   (DSPICO_SAMPLE_RATE_HZ / 1000u)                 // 48
#define DSPICO_FRAME_BYTES      (DSPICO_SAMPLES_PER_MS * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE) // 288

#endif // DSPICO_BOARD_CONFIG_H
