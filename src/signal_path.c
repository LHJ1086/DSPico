// ---------------------------------------------------------------------------
// DSPico — audio signal path implementation.
// ---------------------------------------------------------------------------
#include <string.h>

#include "board_config.h"
#include "audio_ring.h"
#include "signal_path.h"

#define FRAME_BYTES (DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE)   // 6

static peq_t        s_peq;
static audio_ring_t s_play;

// Carry buffer for a partial trailing frame across push calls.
static uint8_t  s_carry[FRAME_BYTES];
static uint32_t s_carry_len;

void signal_path_init(void) {
  peq_init(&s_peq, (float) DSPICO_SAMPLE_RATE_HZ);
  audio_ring_init(&s_play);
  s_carry_len = 0;
}

void signal_path_on_stream_start(void) {
  peq_reset_state(&s_peq);
  s_carry_len = 0;
}

// Process a whole number of frames (in a scratch buffer) and push to the ring.
static void process_and_push(const uint8_t *frames, uint32_t n_frames) {
  // Work in modest chunks so the scratch buffer stays small.
  uint8_t scratch[64 * FRAME_BYTES];
  const uint32_t chunk_frames = sizeof(scratch) / FRAME_BYTES;

  while (n_frames) {
    uint32_t f = n_frames > chunk_frames ? chunk_frames : n_frames;
    memcpy(scratch, frames, f * FRAME_BYTES);
    peq_process_interleaved_s24(&s_peq, scratch, f);
    audio_ring_write(&s_play, scratch, f * FRAME_BYTES);   // drops if ring full
    frames    += f * FRAME_BYTES;
    n_frames  -= f;
  }
}

void signal_path_push_capture(const uint8_t *s24, uint32_t bytes) {
  // 1) Complete any carried partial frame first.
  if (s_carry_len) {
    uint32_t need = FRAME_BYTES - s_carry_len;
    uint32_t take = bytes < need ? bytes : need;
    memcpy(s_carry + s_carry_len, s24, take);
    s_carry_len += take;
    s24   += take;
    bytes -= take;
    if (s_carry_len == FRAME_BYTES) {
      process_and_push(s_carry, 1);
      s_carry_len = 0;
    }
  }

  // 2) Bulk of whole frames.
  uint32_t whole = bytes / FRAME_BYTES;
  if (whole) {
    process_and_push(s24, whole);
    s24   += whole * FRAME_BYTES;
    bytes -= whole * FRAME_BYTES;
  }

  // 3) Stash any trailing partial frame.
  if (bytes) {
    memcpy(s_carry, s24, bytes);
    s_carry_len = bytes;
  }
}

size_t signal_path_pull_play(uint8_t *dst, size_t max_frames) {
  uint32_t want = (uint32_t) max_frames * FRAME_BYTES;
  uint32_t got  = audio_ring_read(&s_play, dst, want);
  return got / FRAME_BYTES;
}

// --- Config -----------------------------------------------------------------
void signal_path_set_host_gain(float linear) { peq_set_host_gain(&s_peq, linear); }
void signal_path_set_pre_gain_db(float db)   { peq_set_pre_gain_db(&s_peq, db); }
void signal_path_set_band(uint8_t idx, const peq_band_t *band) {
  peq_set_band(&s_peq, idx, band);
}
peq_t *signal_path_peq(void) { return &s_peq; }
