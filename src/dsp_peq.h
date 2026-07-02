// ---------------------------------------------------------------------------
// DSPico — on-device parametric EQ engine (brief §5, §6b).
//
// This is the actual EQ that runs ON THE BRIDGE (not the PC): a stereo chain of
// RBJ-cookbook biquads with a global pre-gain (headroom) stage in front and a
// host-volume gain at the end. All math is float32 (hardware FPU). The device is
// the source of truth for coefficients — the (future) WebUSB app only sends
// high-level band parameters, which this module turns into coefficients.
//
// Signal chain (per the brief):
//   samples -> PRE-GAIN -> biquad[0..N) -> HOST VOLUME -> samples
//
// Config setters and the process function are all expected to run on the same
// core (core0), so no locking is needed between them.
// ---------------------------------------------------------------------------
#ifndef DSPICO_DSP_PEQ_H
#define DSPICO_DSP_PEQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PEQ_MAX_BANDS 10

typedef enum {
  PEQ_PEAKING = 0,   // parametric peak/dip (uses Fc, gain, Q)
  PEQ_LOWSHELF,      // low shelf       (uses Fc, gain, Q)
  PEQ_HIGHSHELF,     // high shelf      (uses Fc, gain, Q)
  PEQ_LOWPASS,       // 2nd-order LPF   (uses Fc, Q; gain ignored)
  PEQ_HIGHPASS,      // 2nd-order HPF   (uses Fc, Q; gain ignored)
} peq_type_t;

typedef struct {
  bool       enabled;
  peq_type_t type;
  float      fc;       // centre/corner frequency, Hz
  float      gain_db;  // band gain, dB (peaking/shelf)
  float      q;        // quality factor
} peq_band_t;

typedef struct { float b0, b1, b2, a1, a2; } biquad_coeffs_t;  // a0-normalised
typedef struct { float x1, x2, y1, y2; }     biquad_state_t;   // Direct Form I

typedef struct {
  float           fs;                 // sample rate, Hz
  float           pre_gain;           // linear, applied before the biquads
  float           host_gain;          // linear, applied after (UAC2 volume/mute)
  uint8_t         n_bands;            // active band slots in use (<= PEQ_MAX_BANDS)
  peq_band_t      band[PEQ_MAX_BANDS];
  biquad_coeffs_t coeffs[PEQ_MAX_BANDS];
  biquad_state_t  state[2][PEQ_MAX_BANDS];  // [channel L/R][band]
} peq_t;

// Initialise flat (all bands disabled, unity gains) at sample rate `fs`.
void peq_init(peq_t *p, float fs);

// Configure one band and recompute just its coefficients. `idx` < PEQ_MAX_BANDS.
void peq_set_band(peq_t *p, uint8_t idx, const peq_band_t *band);

// Global pre-gain / host-volume. Rule of thumb: pre_gain_db <= -(max band boost).
void peq_set_pre_gain_db(peq_t *p, float db);
void peq_set_host_gain(peq_t *p, float linear);

// Recompute all band coefficients (e.g. after a bulk config load).
void peq_recompute(peq_t *p);

// Reset filter history (call on stream start to avoid a transient).
void peq_reset_state(peq_t *p);

// Process `frames` stereo samples in float, in place. L and R are separate.
void peq_process_stereo_f32(peq_t *p, float *l, float *r, size_t frames);

// Process `frames` of interleaved 24-bit packed little-endian (L[3]R[3]) audio
// in place — the wire format on both USB ports.
void peq_process_interleaved_s24(peq_t *p, uint8_t *buf, size_t frames);

// Suggested pre-gain (dB) = -(largest positive band gain), 0 if none boost.
float peq_suggested_pre_gain_db(const peq_t *p);

#endif // DSPICO_DSP_PEQ_H
