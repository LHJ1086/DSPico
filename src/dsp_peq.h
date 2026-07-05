// ---------------------------------------------------------------------------
// DSPico — on-device parametric EQ engine (brief §5, §6b).
//
// This is the actual EQ that runs ON THE BRIDGE (not the PC): a stereo chain of
// TPT state-variable filters (Cytomic SVF — see the svf_coeffs_t note below)
// with a global pre-gain (headroom) stage in front and a
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

#define PEQ_MAX_BANDS 16

// --- Allowed parameter ranges (enforced by peq_set_band) --------------------
// Frequency 1 Hz .. 20 kHz, band gain +/-12 dB (peaking/shelf), Q 0.1 .. 10.
// Out-of-range values passed to peq_set_band are clamped into these bounds.
#define PEQ_FC_MIN_HZ    1.0f
#define PEQ_FC_MAX_HZ    20000.0f
#define PEQ_GAIN_MIN_DB  (-12.0f)
#define PEQ_GAIN_MAX_DB  12.0f
#define PEQ_Q_MIN        0.1f
#define PEQ_Q_MAX        10.0f

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

// Each band is a TPT state-variable filter (Andrew Simper / Cytomic). Chosen
// over a Direct-Form biquad because it stays accurate in float32 all the way
// down to ~1 Hz, where a DF biquad's coefficients collapse (cos(w0) rounds to
// 1.0). Per-sample cost is the same handful of multiply-adds.
typedef struct { float a1, a2, a3, m0, m1, m2; } svf_coeffs_t;  // g/k folded in
typedef struct { float ic1eq, ic2eq; }           svf_state_t;   // 2 integrators

typedef struct {
  float           fs;                 // sample rate, Hz
  float           pre_gain;           // linear, applied before the bands
  float           host_gain;          // TARGET host gain (UAC2 volume/mute)
  float           host_gain_cur;      // applied gain, slewed toward host_gain
  float           gain_step;          // per-frame slew step (~5 ms full scale)
  bool            band_active[PEQ_MAX_BANDS];  // enabled AND not identity
  peq_band_t      band[PEQ_MAX_BANDS];
  svf_coeffs_t    coeffs[PEQ_MAX_BANDS];
  svf_state_t     state[2][PEQ_MAX_BANDS];  // [channel L/R][band]
} peq_t;

// Initialise flat (all bands disabled, unity gains) at sample rate `fs`.
void peq_init(peq_t *p, float fs);

// Configure one band and recompute just its coefficients. `idx` < PEQ_MAX_BANDS.
// Fc, gain, and Q are clamped to the PEQ_* ranges above before use.
void peq_set_band(peq_t *p, uint8_t idx, const peq_band_t *band);

// Clamp a band's Fc/gain/Q into the allowed ranges (in place). Exposed so a
// config/UI layer can round-trip the same values the device will actually use.
void peq_clamp_band(peq_band_t *band);

// Global pre-gain / host-volume. Rule of thumb: pre_gain_db <= -(max band boost).
// peq_set_host_gain sets a TARGET: the applied gain slews toward it per frame
// (full scale in ~5 ms) so OS volume steps and mutes never click. Use the
// _now variant to snap both (e.g. on a config reset, to avoid a ramp through
// the wrong loudness).
void peq_set_pre_gain_db(peq_t *p, float db);
void peq_set_host_gain(peq_t *p, float linear);
void peq_set_host_gain_now(peq_t *p, float linear);

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
// Conservative heuristic: overlapping boosts near the same frequency can sum
// above any single band, so extreme curves may still need more headroom.
float peq_suggested_pre_gain_db(const peq_t *p);

#endif // DSPICO_DSP_PEQ_H
