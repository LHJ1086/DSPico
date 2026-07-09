// ---------------------------------------------------------------------------
// DSPico — UAC2 device side (Phase 1a, brief §8)
//
// Owns the volume/mute state and the audio-class control-request handling for
// the native (PC-facing) USB device. Received audio is EQ'd and forwarded to
// the play ring toward the downstream DAC (see tud_audio_rx_done_post_read_cb).
// ---------------------------------------------------------------------------
#ifndef DSPICO_UAC2_DEVICE_H
#define DSPICO_UAC2_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

// True while the host has selected the operational alt setting (streaming).
bool uac2_is_streaming(void);

// Current host volume as a linear float gain in [0,1+] (mute applied).
// Phase 3 multiplies the DSP output by this.
float uac2_host_gain(void);

// Call from the core0 main loop: recomputes the async-feedback value from the
// play ring's fill level so the PC's send rate tracks the DAC's drain rate.
void uac2_feedback_task(void);

// Call from the core0 main loop: periodic device-side status heartbeat
// (mount/stream state + bytes received from the PC) for field diagnosis.
void uac2_device_task(void);

// Called by the vendor control handler when the host requests the MS OS 2.0
// descriptor set — only Windows does, and Windows needs the 16.16/4-byte
// feedback format instead of the full-speed-spec 10.14/3-byte one.
void uac2_note_windows_host(void);

#endif // DSPICO_UAC2_DEVICE_H
