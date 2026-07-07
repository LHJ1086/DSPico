// ---------------------------------------------------------------------------
// DSPico — UAC2 device callbacks (Phase 1a).
//
// Implements the TinyUSB audio-device hooks: alt-setting changes, draining the
// isochronous OUT stream, the async feedback parameters, and the Clock Source /
// Feature Unit control requests that make Windows/macOS/Linux/Android show the
// bridge as a selectable output with a working volume slider + mute.
// ---------------------------------------------------------------------------
#include <math.h>
#include <string.h>

#include "tusb.h"
#include "board_config.h"
#include "debug_log.h"
#include "usb_descriptors.h"
#include "uac2_device.h"
#include "signal_path.h"

// --- Device-stack lifecycle logging (core0) ---------------------------------
// Mount/unmount/suspend cycles visible in the diagnostic log make PC-side
// instability (re-enumeration loops, selective suspend) diagnosable from the
// configurator alone.
void tud_mount_cb(void)   { dlog0("DSPico device: mounted by PC\n"); }
void tud_umount_cb(void)  { dlog0("DSPico device: unmounted\n"); }
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  dlog0("DSPico device: suspended by PC\n");
}
void tud_resume_cb(void)  { dlog0("DSPico device: resumed\n"); }

// --- Volume / mute state ---------------------------------------------------
// UAC2 volume is signed 16-bit in 1/256 dB steps. We advertise -60..0 dB.
#define VOL_MIN_DB_256   (-60 * 256)
#define VOL_MAX_DB_256   (0 * 256)
#define VOL_RES_DB_256   (256)          // 1 dB steps

// Index 0 = master, 1 = front-left, 2 = front-right.
static int16_t s_volume_db256[DSPICO_NUM_CHANNELS + 1] = { -6 * 256, 0, 0 };
static int8_t  s_mute[DSPICO_NUM_CHANNELS + 1] = { 0, 0, 0 };

// Written on core0 (USB callbacks), read on core1 (the DAC fill path) — use
// acquire/release atomics like the ring does, not a bare volatile.
static bool s_streaming = false;

static inline void set_streaming(bool on) {
  __atomic_store_n(&s_streaming, on, __ATOMIC_RELEASE);
}

bool uac2_is_streaming(void) {
  return __atomic_load_n(&s_streaming, __ATOMIC_ACQUIRE);
}

float uac2_host_gain(void) {
  if (s_mute[0]) return 0.0f;
  // Convert master volume (1/256 dB) to a linear gain.
  //
  // Deliberately MASTER-ONLY: the descriptor advertises volume/mute controls
  // on the master channel only (UAC2_FU_CTRL_MASTER in usb_descriptors.c), so
  // compliant hosts drive index 0. Per-channel SETs are still accepted and
  // stored (and read back by GET, keeping hosts consistent) but do not affect
  // the audio — the signal path applies one gain to both channels.
  const float db = (float)s_volume_db256[0] / 256.0f;
  return powf(10.0f, db / 20.0f);
}

// ---------------------------------------------------------------------------
// Streaming start/stop
// ---------------------------------------------------------------------------
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  const uint8_t itf = TU_U16_LOW(p_request->wIndex);
  const uint8_t alt = TU_U16_LOW(p_request->wValue);

  if (itf == ITF_NUM_AUDIO_STREAMING) {
    const bool on = (alt != 0);   // alt 1 = operational, alt 0 = zero bandwidth
    if (on) {
      signal_path_on_stream_start();               // clear filter history
      signal_path_set_host_gain(uac2_host_gain()); // apply current volume/mute
    }
    set_streaming(on);
  }
  return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  const uint8_t itf = TU_U16_LOW(p_request->wIndex);
  if (itf == ITF_NUM_AUDIO_STREAMING) {
    set_streaming(false);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Received audio: run it through the on-device signal path (pre-gain -> PEQ ->
// host volume) and push it to the play ring for the DAC (brief §5).
// ---------------------------------------------------------------------------
bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received,
                                    uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  (void) rhport; (void) func_id; (void) ep_out; (void) cur_alt_setting;

  static uint8_t scratch[CFG_TUD_AUDIO_EP_SZ_OUT];
  uint16_t remaining = n_bytes_received;
  while (remaining) {
    const uint16_t chunk = remaining > sizeof(scratch) ? (uint16_t)sizeof(scratch) : remaining;
    const uint16_t got = tud_audio_read(scratch, chunk);
    if (got == 0) break;
    remaining -= got;
    signal_path_push_capture(scratch, got);   // EQ + enqueue toward the DAC
  }
  return true;
}

// ---------------------------------------------------------------------------
// Async feedback: let TinyUSB derive the value from our FIFO fill level so the
// PC's send rate tracks our drain rate (brief §5). Simple and correct for a
// single fixed sample rate; Phase 4 replaces the source with the DAC FIFO.
// ---------------------------------------------------------------------------
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf,
                                  audio_feedback_params_t *feedback_param) {
  (void) func_id; (void) alt_itf;
  feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
  feedback_param->sample_freq = DSPICO_SAMPLE_RATE_HZ;
}

// ---------------------------------------------------------------------------
// Control requests — GET
// ---------------------------------------------------------------------------
static bool clock_get_request(uint8_t rhport, tusb_control_request_t const *req) {
  const uint8_t ctrl_sel = TU_U16_HIGH(req->wValue);

  if (ctrl_sel == AUDIO_CS_CTRL_SAM_FREQ) {
    if (req->bRequest == AUDIO_CS_REQ_CUR) {
      audio_control_cur_4_t cur = { .bCur = (int32_t) tu_htole32(DSPICO_SAMPLE_RATE_HZ) };
      return tud_control_xfer(rhport, req, &cur, sizeof(cur));
    }
    if (req->bRequest == AUDIO_CS_REQ_RANGE) {
      // Single supported frequency: 48000..48000, res 0.
      audio_control_range_4_n_t(1) rng = {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { .bMin = (int32_t) DSPICO_SAMPLE_RATE_HZ,
                         .bMax = (int32_t) DSPICO_SAMPLE_RATE_HZ,
                         .bRes = 0 },
      };
      return tud_control_xfer(rhport, req, &rng, sizeof(rng));
    }
  } else if (ctrl_sel == AUDIO_CS_CTRL_CLK_VALID && req->bRequest == AUDIO_CS_REQ_CUR) {
    audio_control_cur_1_t valid = { .bCur = 1 };   // clock always valid
    return tud_control_xfer(rhport, req, &valid, sizeof(valid));
  }
  return false;
}

static bool feature_unit_get_request(uint8_t rhport, tusb_control_request_t const *req) {
  const uint8_t ctrl_sel = TU_U16_HIGH(req->wValue);
  const uint8_t channel  = TU_U16_LOW(req->wValue);

  if (channel > DSPICO_NUM_CHANNELS) return false;

  if (ctrl_sel == AUDIO_FU_CTRL_MUTE && req->bRequest == AUDIO_CS_REQ_CUR) {
    audio_control_cur_1_t cur = { .bCur = s_mute[channel] };
    return tud_control_xfer(rhport, req, &cur, sizeof(cur));
  }

  if (ctrl_sel == AUDIO_FU_CTRL_VOLUME) {
    if (req->bRequest == AUDIO_CS_REQ_CUR) {
      audio_control_cur_2_t cur = { .bCur = tu_htole16(s_volume_db256[channel]) };
      return tud_control_xfer(rhport, req, &cur, sizeof(cur));
    }
    if (req->bRequest == AUDIO_CS_REQ_RANGE) {
      audio_control_range_2_n_t(1) rng = {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { .bMin = tu_htole16(VOL_MIN_DB_256),
                         .bMax = tu_htole16(VOL_MAX_DB_256),
                         .bRes = tu_htole16(VOL_RES_DB_256) },
      };
      return tud_control_xfer(rhport, req, &rng, sizeof(rng));
    }
  }
  return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  const uint8_t entity_id = TU_U16_HIGH(p_request->wIndex);

  if (entity_id == UAC2_ENTITY_CLOCK)        return clock_get_request(rhport, p_request);
  if (entity_id == UAC2_ENTITY_FEATURE_UNIT) return feature_unit_get_request(rhport, p_request);
  return false;
}

// ---------------------------------------------------------------------------
// Control requests — SET
// ---------------------------------------------------------------------------
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request,
                                 uint8_t *buf) {
  (void) rhport;
  const uint8_t entity_id = TU_U16_HIGH(p_request->wIndex);
  const uint8_t ctrl_sel  = TU_U16_HIGH(p_request->wValue);
  const uint8_t channel   = TU_U16_LOW(p_request->wValue);

  if (entity_id != UAC2_ENTITY_FEATURE_UNIT) return false;
  if (channel > DSPICO_NUM_CHANNELS) return false;
  if (p_request->bRequest != AUDIO_CS_REQ_CUR) return false;

  if (ctrl_sel == AUDIO_FU_CTRL_MUTE) {
    s_mute[channel] = ((audio_control_cur_1_t const *) buf)->bCur;
    signal_path_set_host_gain(uac2_host_gain());
    return true;
  }
  if (ctrl_sel == AUDIO_FU_CTRL_VOLUME) {
    s_volume_db256[channel] = (int16_t) tu_le16toh(((audio_control_cur_2_t const *) buf)->bCur);
    signal_path_set_host_gain(uac2_host_gain());
    return true;
  }
  return false;
}
