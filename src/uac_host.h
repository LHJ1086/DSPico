// ---------------------------------------------------------------------------
// DSPico — USB Audio host driver over Pico-PIO-USB (brief §7, Phase 1b)
//
// TinyUSB ships NO UAC host class driver, so this registers a small custom
// application host driver (via usbh_app_driver_get_cb) that:
//   1. claims a class-compliant audio device on the PIO host port,
//   2. sets its clock to 48 kHz and selects the operational alt setting,
//   3. opens the isochronous OUT endpoint, and
//   4. continuously streams whatever the fill callback provides (a test tone
//      in Phase 1b; the play ring in Phase 2+).
//
// Whether iso-OUT streams cleanly over the bit-banged PIO port is THE open
// question the whole project hinges on — expect to iterate here on hardware
// with a logic analyzer.
// ---------------------------------------------------------------------------
#ifndef DSPICO_UAC_HOST_H
#define DSPICO_UAC_HOST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Source callback: fill `dst` with up to `max_frames` stereo frames of 24-bit
// packed audio and return the number of frames actually written. Called from
// the host stack context (core1) each time a packet is needed. Returning fewer
// frames than requested is fine (the packet shrinks); returning 0 sends silence.
typedef size_t (*uac_host_fill_cb_t)(uint8_t *dst, size_t max_frames);

// Register the audio-source callback used for every outgoing packet.
void uac_host_set_source(uac_host_fill_cb_t cb);

// True once a DAC is mounted and its iso OUT endpoint is streaming.
bool uac_host_is_streaming(void);

#endif // DSPICO_UAC_HOST_H
