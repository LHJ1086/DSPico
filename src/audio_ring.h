// ---------------------------------------------------------------------------
// DSPico — single-producer / single-consumer byte ring (lock-free).
//
// Connects the two cores: the device RX callback (core0) is the sole producer,
// the host fill callback (core1) is the sole consumer. Acquire/release atomics
// on the head/tail indices make it safe across the RP2350's two M33 cores with
// no spinlock. Capacity must be a power of two.
// ---------------------------------------------------------------------------
#ifndef DSPICO_AUDIO_RING_H
#define DSPICO_AUDIO_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef AUDIO_RING_CAP
#define AUDIO_RING_CAP 8192u          // ~28 ms of 48k/24/stereo — power of two
#endif
_Static_assert((AUDIO_RING_CAP & (AUDIO_RING_CAP - 1)) == 0,
               "AUDIO_RING_CAP must be a power of two");

typedef struct {
  uint8_t  buf[AUDIO_RING_CAP];
  volatile uint32_t head;   // producer writes here (next free slot)
  volatile uint32_t tail;   // consumer reads here
} audio_ring_t;

static inline void audio_ring_init(audio_ring_t *r) {
  r->head = 0;
  r->tail = 0;
}

static inline uint32_t audio_ring_used(const audio_ring_t *r) {
  const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);
  const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);
  return head - tail;
}

static inline uint32_t audio_ring_free(const audio_ring_t *r) {
  return AUDIO_RING_CAP - audio_ring_used(r) - 1;   // keep one slot empty
}

// Producer: push up to `len` bytes; returns bytes actually written.
static inline uint32_t audio_ring_write(audio_ring_t *r, const uint8_t *src, uint32_t len) {
  uint32_t space = audio_ring_free(r);
  if (len > space) len = space;
  uint32_t head = r->head;
  for (uint32_t i = 0; i < len; i++) {
    r->buf[(head + i) & (AUDIO_RING_CAP - 1)] = src[i];
  }
  __atomic_store_n(&r->head, head + len, __ATOMIC_RELEASE);
  return len;
}

// Consumer: pop up to `len` bytes; returns bytes actually read.
static inline uint32_t audio_ring_read(audio_ring_t *r, uint8_t *dst, uint32_t len) {
  uint32_t avail = audio_ring_used(r);
  if (len > avail) len = avail;
  uint32_t tail = r->tail;
  for (uint32_t i = 0; i < len; i++) {
    dst[i] = r->buf[(tail + i) & (AUDIO_RING_CAP - 1)];
  }
  __atomic_store_n(&r->tail, tail + len, __ATOMIC_RELEASE);
  return len;
}

#endif // DSPICO_AUDIO_RING_H
