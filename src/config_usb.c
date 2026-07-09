// ---------------------------------------------------------------------------
// DSPico — WebUSB config protocol handler + flash persistence.
// ---------------------------------------------------------------------------
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "tusb.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "board_config.h"
#include "crc32.h"
#include "debug_log.h"
#include "dsp_peq.h"
#include "signal_path.h"
#include "usb_descriptors.h"
#include "uac2_device.h"
#include "config_usb.h"

// --- Protocol (must match web/dspico.js PROTO) -----------------------------
#define REQ_INFO         0x01
#define REQ_GET_STATE    0x02
#define REQ_SET_PREGAIN  0x03
#define REQ_SET_BAND     0x04
#define REQ_COMMIT       0x05
#define REQ_RESET        0x06
#define REQ_GET_LOG      0x07   // IN: drain buffered host-side diagnostics
#define VENDOR_REQUEST_WEBUSB    0x21
#define VENDOR_REQUEST_MICROSOFT 0x22
#define WEBUSB_REQUEST_GET_URL   2

#define CFG_MAGIC   0x51505344u   // 'DSPQ'
#define CFG_VERSION 2             // v2: added crc32 over the payload

// One band as it travels on the wire / sits in flash (16 bytes).
typedef struct TU_ATTR_PACKED {
  uint8_t type;
  uint8_t enabled;
  uint8_t _pad[2];
  float   fc;
  float   gain_db;
  float   q;
} wire_band_t;
TU_VERIFY_STATIC(sizeof(wire_band_t) == 16, "wire_band_t must be 16 bytes");

// --- Buffers for control transfers -----------------------------------------
static uint8_t s_ctrl_out[64];                       // inbound SET_* payloads
static uint8_t s_state_buf[8 + PEQ_MAX_BANDS * 16];  // outbound GET_STATE

// Set by REQ_COMMIT in the control callback, consumed by config_usb_task() in
// the core0 main loop (same core, so a plain flag is enough; the callback runs
// inside tud_task(), never from an ISR).
static bool s_commit_pending = false;

// ===========================================================================
// Flash persistence (last sector). Writing flash must pause core1 (which runs
// the PIO host from XIP) and disable IRQs on core0.
// ===========================================================================
#define CFG_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct TU_ATTR_PACKED {
  uint32_t    magic;
  uint16_t    version;
  uint8_t     n_bands;
  uint8_t     _pad;
  uint32_t    crc32;        // over everything AFTER this field (the payload);
                            // rejects torn writes (power loss mid-commit)
  float       pre_gain_db;
  wire_band_t band[PEQ_MAX_BANDS];
} cfg_blob_t;
TU_VERIFY_STATIC(sizeof(cfg_blob_t) <= FLASH_PAGE_SIZE * 2, "cfg blob too large");

#define CFG_PAYLOAD_OFFSET offsetof(cfg_blob_t, pre_gain_db)
#define CFG_PAYLOAD_LEN    (sizeof(cfg_blob_t) - CFG_PAYLOAD_OFFSET)

static float linear_to_db(float lin) {
  if (lin <= 1e-6f) return -120.0f;
  return 20.0f * log10f(lin);
}

static void config_persist(void) {
  peq_t *p = signal_path_peq();

  cfg_blob_t blob;
  memset(&blob, 0, sizeof(blob));
  blob.magic       = CFG_MAGIC;
  blob.version     = CFG_VERSION;
  blob.pre_gain_db = linear_to_db(p->pre_gain);
  blob.n_bands     = PEQ_MAX_BANDS;
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    blob.band[i].type    = (uint8_t) p->band[i].type;
    blob.band[i].enabled = p->band[i].enabled ? 1 : 0;
    blob.band[i].fc      = p->band[i].fc;
    blob.band[i].gain_db = p->band[i].gain_db;
    blob.band[i].q       = p->band[i].q;
  }
  blob.crc32 = dspico_crc32((const uint8_t *) &blob + CFG_PAYLOAD_OFFSET,
                            CFG_PAYLOAD_LEN);

  // Program a whole number of 256-byte pages.
  static uint8_t page[((sizeof(cfg_blob_t) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE];
  memset(page, 0, sizeof(page));
  memcpy(page, &blob, sizeof(blob));

  multicore_lockout_start_blocking();          // freeze core1 (PIO host)
  const uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(CFG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(CFG_FLASH_OFFSET, page, sizeof(page));
  restore_interrupts(ints);
  multicore_lockout_end_blocking();
}

void config_usb_init(void) {
  const cfg_blob_t *f = (const cfg_blob_t *) (XIP_BASE + CFG_FLASH_OFFSET);
  if (f->magic != CFG_MAGIC || f->version != CFG_VERSION) return;   // nothing saved
  if (dspico_crc32((const uint8_t *) f + CFG_PAYLOAD_OFFSET, CFG_PAYLOAD_LEN)
      != f->crc32) {
    return;   // torn/corrupt blob — boot flat rather than load garbage
  }

  signal_path_set_pre_gain_db(f->pre_gain_db);
  const uint8_t n = f->n_bands > PEQ_MAX_BANDS ? PEQ_MAX_BANDS : f->n_bands;
  for (uint8_t i = 0; i < n; i++) {
    peq_band_t b = {
        .enabled = f->band[i].enabled != 0,
        .type    = (peq_type_t) f->band[i].type,
        .fc      = f->band[i].fc,
        .gain_db = f->band[i].gain_db,
        .q       = f->band[i].q,
    };
    signal_path_set_band(i, &b);
  }
}

// ===========================================================================
// Vendor control protocol
// ===========================================================================
static uint16_t build_info(uint8_t *b) {
  uint32_t magic = CFG_MAGIC;
  memcpy(b + 0, &magic, 4);
  uint16_t ver = CFG_VERSION;      memcpy(b + 4, &ver, 2);
  b[6] = PEQ_MAX_BANDS;
  b[7] = 0;
  uint32_t sr = DSPICO_SAMPLE_RATE_HZ; memcpy(b + 8, &sr, 4);
  // Uptime (whole seconds) — a device that always reports a tiny uptime is
  // reset-looping, which the configurator flags. Older clients that request
  // only 12 bytes simply don't see this field.
  uint32_t up = to_ms_since_boot(get_absolute_time()) / 1000u;
  memcpy(b + 12, &up, 4);
  return 16;
}

static uint16_t build_state(uint8_t *b) {
  peq_t *p = signal_path_peq();
  const float pg = linear_to_db(p->pre_gain);
  memcpy(b + 0, &pg, 4);

  uint8_t n = 0;
  uint8_t *rec = b + 8;
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    if (!p->band[i].enabled) continue;      // report only active bands
    wire_band_t w = {
        .type = (uint8_t) p->band[i].type, .enabled = 1,
        .fc = p->band[i].fc, .gain_db = p->band[i].gain_db, .q = p->band[i].q,
    };
    memcpy(rec, &w, sizeof(w));
    rec += sizeof(w);
    n++;
  }
  b[4] = n; b[5] = b[6] = b[7] = 0;
  return (uint16_t) (8 + n * sizeof(wire_band_t));
}

// TinyUSB calls this for vendor-type control requests (device + interface).
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  switch (request->bRequest) {

    // --- WebUSB: return the landing-page URL descriptor ---------------------
    case VENDOR_REQUEST_WEBUSB:
      if (stage != CONTROL_STAGE_SETUP) return true;
      if (request->wIndex == WEBUSB_REQUEST_GET_URL) {
        uint16_t len; const uint8_t *url = dspico_desc_webusb_url(&len);
        return tud_control_xfer(rhport, request, (void *) url, len);
      }
      return false;

    // --- MS OS 2.0 descriptor set (wIndex == 7) -----------------------------
    case VENDOR_REQUEST_MICROSOFT:
      if (stage != CONTROL_STAGE_SETUP) return true;
      if (request->wIndex == 7) {
        uac2_note_windows_host();   // only Windows asks — switch feedback format
        uint16_t len; const uint8_t *desc = dspico_desc_ms_os_20(&len);
        return tud_control_xfer(rhport, request, (void *) desc, len);
      }
      return false;

    // --- App: device info ---------------------------------------------------
    case REQ_INFO:
      if (stage != CONTROL_STAGE_SETUP) return true;
      return tud_control_xfer(rhport, request, s_ctrl_out /*reuse*/, build_info(s_ctrl_out));

    // --- App: full state (pre-gain + active bands) --------------------------
    case REQ_GET_STATE:
      if (stage != CONTROL_STAGE_SETUP) return true;
      return tud_control_xfer(rhport, request, s_state_buf, build_state(s_state_buf));

    // --- App: drain the device's diagnostic log (host-side bring-up) --------
    case REQ_GET_LOG: {
      if (stage != CONTROL_STAGE_SETUP) return true;
      static uint8_t logbuf[256];
      const uint16_t want = request->wLength > sizeof(logbuf)
                          ? (uint16_t) sizeof(logbuf) : request->wLength;
      return tud_control_xfer(rhport, request, logbuf, dlog_usb_read(logbuf, want));
    }

    // --- App: set pre-gain (f32 dB) -----------------------------------------
    case REQ_SET_PREGAIN:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wLength < 4) return false;   // short transfer -> STALL
        return tud_control_xfer(rhport, request, s_ctrl_out,
                                request->wLength > sizeof(s_ctrl_out) ? sizeof(s_ctrl_out) : request->wLength);
      }
      if (stage == CONTROL_STAGE_ACK) {
        float db; memcpy(&db, s_ctrl_out, 4);
        signal_path_set_pre_gain_db(db);
      }
      return true;

    // --- App: set one band (wIndex high byte = slot) ------------------------
    case REQ_SET_BAND:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wLength < sizeof(wire_band_t)) return false;   // STALL
        return tud_control_xfer(rhport, request, s_ctrl_out,
                                request->wLength > sizeof(s_ctrl_out) ? sizeof(s_ctrl_out) : request->wLength);
      }
      if (stage == CONTROL_STAGE_ACK) {
        const uint8_t idx = (uint8_t) (request->wIndex >> 8);
        wire_band_t w; memcpy(&w, s_ctrl_out, sizeof(w));
        peq_band_t b = {
            .enabled = w.enabled != 0, .type = (peq_type_t) w.type,
            .fc = w.fc, .gain_db = w.gain_db, .q = w.q,
        };
        signal_path_set_band(idx, &b);
      }
      return true;

    // --- App: commit current EQ to flash ------------------------------------
    // The write itself (sector erase + program, tens of ms with core0 IRQs off
    // and core1 frozen) is far too slow for a control callback — blocking here
    // would stall tud_task() and can make the host time the transfer out. ACK
    // now, persist from the core0 main loop via config_usb_task().
    case REQ_COMMIT:
      if (stage != CONTROL_STAGE_SETUP) return true;
      s_commit_pending = true;
      return tud_control_xfer(rhport, request, NULL, 0);

    // --- App: reset to flat -------------------------------------------------
    case REQ_RESET:
      if (stage != CONTROL_STAGE_SETUP) return true;
      peq_init(signal_path_peq(), (float) DSPICO_SAMPLE_RATE_HZ);
      // peq_init wipes the gains too — SNAP (not ramp) the OS volume/mute back
      // in, otherwise a reset at low volume would slew from unity through a
      // brief loud burst.
      peq_set_host_gain_now(signal_path_peq(), uac2_host_gain());
      return tud_control_xfer(rhport, request, NULL, 0);
  }

  return false;   // unknown request -> STALL
}

// Runs deferred work that must not block the USB control callback. Call from
// the core0 main loop, outside tud_task().
void config_usb_task(void) {
  // Hold the commit until core1's lockout victim is up — freezing core1 is
  // mandatory during the flash write (it executes from XIP), and requesting a
  // lockout before the victim exists would block forever. Retries next loop.
  if (s_commit_pending && multicore_lockout_victim_is_initialized(1)) {
    s_commit_pending = false;
    config_persist();
  }
}
