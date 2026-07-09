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

// Bumped on every stream (re)start (core0). The consumer (core1) discards
// whatever is still queued from before the bump, so a new stream never opens
// with the previous stream's stale tail. Accessed with acquire/release
// atomics (like the ring's indices), not a bare volatile.
static uint32_t s_stream_epoch;

void signal_path_init(void) {
  peq_init(&s_peq, (float) DSPICO_SAMPLE_RATE_HZ);
  audio_ring_init(&s_play);
  s_carry_len = 0;
}

void signal_path_on_stream_start(void) {
  peq_reset_state(&s_peq);
  s_carry_len = 0;
  __atomic_add_fetch(&s_stream_epoch, 1, __ATOMIC_RELEASE);
}

// EQ a whole number of frames IN PLACE in the caller's buffer, then push them
// to the ring. In-place processing saves a per-packet scratch copy on the hot
// device-RX path (the caller's buffer is transient by contract, see the
// header). Only whole frames may enter the ring: a byte-truncated write would
// shift the producer/consumer frame alignment permanently, so when the ring
// is (nearly) full whole frames are dropped instead (drop-newest) — and only
// the frames that will actually be queued are EQ'd.
static void process_and_push(uint8_t *frames, uint32_t n_frames) {
  const uint32_t space_frames = audio_ring_free(&s_play) / FRAME_BYTES;
  if (n_frames > space_frames) n_frames = space_frames;
  if (n_frames == 0) return;

  peq_process_interleaved_s24(&s_peq, frames, n_frames);
  audio_ring_write(&s_play, frames, n_frames * FRAME_BYTES);
}

void signal_path_push_capture(uint8_t *s24, uint32_t bytes) {
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
  static uint32_t seen_epoch;   // core1-only state
  static bool     primed;

  // A new stream invalidates whatever is still queued: discard it so the new
  // audio doesn't open with the previous stream's tail. (The producer bumps
  // the epoch before pushing the new stream's data; at worst the first
  // millisecond or two of the new stream is discarded along with the tail,
  // which the priming below would have held back anyway.)
  const uint32_t epoch = __atomic_load_n(&s_stream_epoch, __ATOMIC_ACQUIRE);
  if (epoch != seen_epoch) {
    seen_epoch = epoch;
    primed = false;
    uint8_t scrap[8 * FRAME_BYTES];
    while (audio_ring_read(&s_play, scrap, sizeof(scrap))) {}
  }

  const uint32_t used = audio_ring_used(&s_play);
  if (!primed) {
    if (used < SIGNAL_PATH_PRIME_BYTES) return 0;   // build a cushion first
    primed = true;
  } else if (used == 0) {
    primed = false;                                 // underrun: re-prime
    return 0;
  }

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
uint32_t signal_path_play_fill(void) { return audio_ring_used(&s_play); }
