// ---------------------------------------------------------------------------
// DSPico — board configuration for the Waveshare RP2350-USB-CM
//
// All hardware-specific facts (pins, clock, USB roles) live here so the rest of
// the firmware never hard-codes a magic pin number. Values are taken from the
// board schematic as summarised in the build brief (§2, §4, §8).
// ---------------------------------------------------------------------------
#ifndef DSPICO_BOARD_CONFIG_H
#define DSPICO_BOARD_CONFIG_H

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

// --- PIO-USB data pins (Type-C2) -------------------------------------------
// The schematic wires D- = GPIO12 and D+ = GPIO13. Pico-PIO-USB's config takes
// the D+ pin and assumes D- = D+ + 1 by default (ascending). This board has the
// pins REVERSED (D+ is the higher pin), so we must set D+ = GPIO13 and enable
// the DP/DM swap so the library drives the correct polarity (brief §3b).
#define DSPICO_PIO_USB_DP_PIN 13   // GPIO13 = D+
// D- is DP-1 = GPIO12 once the swap flag below is applied.
#define DSPICO_PIO_USB_PINOUT_DPDM_SWAP 1

// --- Status LED (brief §2) -------------------------------------------------
// WS2812B (NeoPixel) on GPIO16. Single pixel used as a coarse state indicator.
#define DSPICO_WS2812_PIN 16

// --- Audio format (LOCKED, brief §1) ---------------------------------------
#define DSPICO_SAMPLE_RATE_HZ   48000u
#define DSPICO_NUM_CHANNELS     2u
#define DSPICO_BYTES_PER_SAMPLE 3u        // 24-bit packed on the wire
#define DSPICO_RESOLUTION_BITS  24u

// One 1 ms USB frame's worth of audio (nominal). At 48 kHz that is 48
// samples/frame/channel. We size ring buffers generously around this.
#define DSPICO_SAMPLES_PER_MS   (DSPICO_SAMPLE_RATE_HZ / 1000u)                 // 48
#define DSPICO_FRAME_BYTES      (DSPICO_SAMPLES_PER_MS * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE) // 288

#endif // DSPICO_BOARD_CONFIG_H
