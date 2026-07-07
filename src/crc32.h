// ---------------------------------------------------------------------------
// DSPico — CRC-32 (ISO-HDLC / zlib polynomial, reflected) for integrity checks
// on the flash-persisted config blob. Pure C, no dependencies, so the native
// test suite exercises the exact code the firmware runs.
//
// Bitwise implementation (no table): the blob is ~270 bytes and is hashed only
// on boot and on commit, so speed is irrelevant and we save 1 KB of table.
// ---------------------------------------------------------------------------
#ifndef DSPICO_CRC32_H
#define DSPICO_CRC32_H

#include <stddef.h>
#include <stdint.h>

static inline uint32_t dspico_crc32(const void *data, size_t len) {
  const uint8_t *p = (const uint8_t *) data;
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

#endif // DSPICO_CRC32_H
