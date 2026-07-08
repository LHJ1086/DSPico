// ---------------------------------------------------------------------------
// DSPico — UAC2 device side (Phase 1a, brief §8)
//
// Owns the volume/mute state and the audio-class control-request handling for
// the native (PC-facing) USB device. In Phase 1a received audio is drained and
// discarded; Phase 2 will forward it into the capture ring instead.
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

#endif // DSPICO_UAC2_DEVICE_H
