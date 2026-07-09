// ---------------------------------------------------------------------------
// DSPico — audio signal path (brief §5).
//
// Ties the on-device PEQ engine to the cross-core play ring:
//
//   [device RX, core0] push_capture()
//        -> PRE-GAIN -> PEQ biquads -> HOST VOLUME   (all on core0)
//        -> play ring
//   [host TX,   core1] pull_play() -> iso OUT -> DAC
//
// This is the single place that owns the EQ state, so the (future) WebUSB
// handler and the UAC2 volume callback both talk to it here.
// ---------------------------------------------------------------------------
#ifndef DSPICO_SIGNAL_PATH_H
#define DSPICO_SIGNAL_PATH_H

#include <stddef.h>
#include <stdint.h>

#include "board_config.h"
#include "dsp_peq.h"

// Consumer-side priming: pull_play() returns nothing until this much audio is
// buffered after a stream (re)start or an underrun, so playback begins with
// enough cushion to ride out scheduling jitter instead of stuttering.
#define SIGNAL_PATH_PRIME_MS     8u
#define SIGNAL_PATH_PRIME_BYTES  (SIGNAL_PATH_PRIME_MS * DSPICO_FRAME_BYTES)

// Initialise the PEQ (flat) and the play ring at the locked sample rate.
void signal_path_init(void);

// --- Producer side (device RX, core0) --------------------------------------
// Feed received 24-bit interleaved stereo audio. It is EQ'd IN PLACE (the
// caller's buffer is clobbered — treat it as transient) and pushed to the
// play ring. `bytes` need not be frame-aligned; a remainder is carried.
void signal_path_push_capture(uint8_t *s24, uint32_t bytes);

// Called when the PC (re)starts/stops streaming, to reset filter history.
void signal_path_on_stream_start(void);

// --- Consumer side (host TX, core1) ----------------------------------------
// Pull up to `max_frames` EQ'd stereo frames into `dst` (24-bit interleaved).
// Returns frames actually produced (0 if the ring is empty / PC not streaming).
size_t signal_path_pull_play(uint8_t *dst, size_t max_frames);

// --- Configuration (core0) — used by UAC2 volume + future WebUSB handler ----
void signal_path_set_host_gain(float linear);
void signal_path_set_pre_gain_db(float db);
void signal_path_set_band(uint8_t idx, const peq_band_t *band);

// Direct access for the (future) config/query surface. Same-core use only.
peq_t *signal_path_peq(void);

// Bytes currently queued toward the DAC (safe from either core) — diagnostics.
uint32_t signal_path_play_fill(void);

#endif // DSPICO_SIGNAL_PATH_H
