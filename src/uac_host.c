// ---------------------------------------------------------------------------
// DSPico — USB Audio host driver over Pico-PIO-USB (Phase 1b).
//
// Custom TinyUSB application host driver. See uac_host.h for the contract and
// the big caveat: isochronous OUT over the bit-banged PIO port is unproven, so
// treat this as the scaffold you iterate on with a logic analyzer.
// ---------------------------------------------------------------------------
#include <string.h>

#include "tusb.h"
#include "host/usbh_pvt.h"   // usbh_class_driver_t, usbh_driver_set_config_complete
#include "board_config.h"
#include "uac_host.h"

// Max audio payload we will ever put on the wire in one frame.
#define UAC_HOST_PKT_MAX  ((DSPICO_SAMPLES_PER_MS + 1) * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE)

// ---------------------------------------------------------------------------
// Device state (single DAC supported for the spike)
// ---------------------------------------------------------------------------
typedef struct {
  bool     in_use;
  uint8_t  dev_addr;
  uint8_t  rhport;

  uint8_t  ac_itf;          // Audio Control interface number
  uint16_t audio_bcd;       // 0x0100 = UAC1, 0x0200 = UAC2
  uint8_t  clock_id;        // UAC2 Clock Source entity (0 if none)

  uint8_t  as_itf;          // Audio Streaming interface number
  uint8_t  as_alt;          // operational alt setting (has the iso EP)
  bool     have_as;

  tusb_desc_endpoint_t ep_out;   // the isochronous OUT endpoint descriptor
  bool     have_ep;

  bool     streaming;
} dac_dev_t;

static dac_dev_t s_dac;
static uac_host_fill_cb_t s_fill_cb = NULL;

// One outstanding iso packet at a time; DMA-aligned.
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t s_pkt[UAC_HOST_PKT_MAX];

void uac_host_set_source(uac_host_fill_cb_t cb) { s_fill_cb = cb; }
bool uac_host_is_streaming(void) { return s_dac.in_use && s_dac.streaming; }

// ---------------------------------------------------------------------------
// Descriptor parsing helpers
// ---------------------------------------------------------------------------
static uint8_t const *desc_next(uint8_t const *p) { return p + p[0]; }

// Single-pass parse of the whole audio-function descriptor block. With an IAD
// (typical UAC2) TinyUSB hands us the entire function at once; without one
// (typical UAC1) it hands us one interface at a time — walking `len` bytes and
// tracking the current interface context handles both.
//
// Records: the AC interface number, UAC version, a UAC2 clock-source id, the AS
// interface number, and the first alt setting carrying an isochronous OUT data
// endpoint (plus a copy of that endpoint descriptor).
static void parse_audio_function(dac_dev_t *d, uint8_t const *p, uint16_t len) {
  uint8_t const *end = p + len;
  uint8_t cur_sub = 0xFF;   // current interface subclass
  uint8_t cur_alt = 0;      // current alternate setting

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    const uint8_t dtype = p[1];

    if (dtype == TUSB_DESC_INTERFACE) {
      tusb_desc_interface_t const *itf = (tusb_desc_interface_t const *) p;
      if (itf->bInterfaceClass == TUSB_CLASS_AUDIO) {
        cur_sub = itf->bInterfaceSubClass;
        cur_alt = itf->bAlternateSetting;
        if (cur_sub == AUDIO_SUBCLASS_CONTROL) {
          d->ac_itf = itf->bInterfaceNumber;
        } else if (cur_sub == AUDIO_SUBCLASS_STREAMING && !d->have_as) {
          d->as_itf = itf->bInterfaceNumber;
        }
      } else {
        cur_sub = 0xFF;   // some non-audio interface inside the block
      }
    } else if (dtype == TUSB_DESC_CS_INTERFACE && cur_sub == AUDIO_SUBCLASS_CONTROL) {
      const uint8_t subtype = p[2];
      if (subtype == AUDIO_CS_AC_INTERFACE_HEADER) {
        d->audio_bcd = tu_unaligned_read16(p + 3);   // bcdADC
      } else if (subtype == AUDIO_CS_AC_INTERFACE_CLOCK_SOURCE) {
        d->clock_id = p[3];                            // bClockID
      }
    } else if (dtype == TUSB_DESC_ENDPOINT && cur_sub == AUDIO_SUBCLASS_STREAMING) {
      tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *) p;
      const bool is_iso  = (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS);
      const bool is_out  = (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_OUT);
      const bool is_data = (ep->bmAttributes.usage == 0x00);   // not feedback
      if (is_iso && is_out && is_data && !d->have_ep) {
        d->as_alt = cur_alt;
        memcpy(&d->ep_out, ep, sizeof(tusb_desc_endpoint_t));
        d->have_ep = true;
        d->have_as = true;
      }
    }
    p = desc_next(p);
  }
}

// ---------------------------------------------------------------------------
// TinyUSB host class-driver callbacks
// ---------------------------------------------------------------------------
static void dac_init(void) {
  memset(&s_dac, 0, sizeof(s_dac));
}

static bool dac_open(uint8_t rhport, uint8_t dev_addr,
                     tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  if (itf_desc->bInterfaceClass != TUSB_CLASS_AUDIO) return false;

  // Bind to the first audio device we see (single-DAC spike).
  if (s_dac.in_use && s_dac.dev_addr != dev_addr) return false;
  s_dac.in_use = true;
  s_dac.dev_addr = dev_addr;
  s_dac.rhport = rhport;

  parse_audio_function(&s_dac, (uint8_t const *) itf_desc, max_len);
  return true;   // claim the audio interface(s) in this block
}

// --- Setup state machine (runs after enumeration) --------------------------
static void submit_first_packet(void);
static void on_set_alt_complete(tuh_xfer_t *xfer);
static void on_set_freq_complete(tuh_xfer_t *xfer);

// SET_INTERFACE(as_itf, as_alt) — activates the iso endpoint on the DAC.
static void send_set_interface(void) {
  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_INTERFACE,
                             .type = TUSB_REQ_TYPE_STANDARD,
                             .direction = TUSB_DIR_OUT },
      .bRequest = TUSB_REQ_SET_INTERFACE,
      .wValue = tu_htole16(s_dac.as_alt),
      .wIndex = tu_htole16(s_dac.as_itf),
      .wLength = 0,
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = NULL,
      .complete_cb = on_set_alt_complete,
      .user_data = 0,
  };
  tuh_control_xfer(&xfer);
}

// UAC2: SET_CUR sample frequency on the Clock Source entity.
static uint8_t s_freq_buf[4];
static void send_set_sample_rate_uac2(void) {
  s_freq_buf[0] = (uint8_t)(DSPICO_SAMPLE_RATE_HZ & 0xFF);
  s_freq_buf[1] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 8) & 0xFF);
  s_freq_buf[2] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 16) & 0xFF);
  s_freq_buf[3] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 24) & 0xFF);

  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_INTERFACE,
                             .type = TUSB_REQ_TYPE_CLASS,
                             .direction = TUSB_DIR_OUT },
      .bRequest = AUDIO_CS_REQ_CUR,
      .wValue = tu_htole16((uint16_t)(AUDIO_CS_CTRL_SAM_FREQ << 8)),
      .wIndex = tu_htole16((uint16_t)((s_dac.clock_id << 8) | s_dac.ac_itf)),
      .wLength = tu_htole16(4),
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = s_freq_buf,
      .complete_cb = on_set_freq_complete,
      .user_data = 0,
  };
  tuh_control_xfer(&xfer);
}

// UAC1: SET_CUR sampling frequency (SAMPLING_FREQ_CONTROL) on the iso endpoint.
static void send_set_sample_rate_uac1(void) {
  s_freq_buf[0] = (uint8_t)(DSPICO_SAMPLE_RATE_HZ & 0xFF);
  s_freq_buf[1] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 8) & 0xFF);
  s_freq_buf[2] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 16) & 0xFF);

  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_ENDPOINT,
                             .type = TUSB_REQ_TYPE_CLASS,
                             .direction = TUSB_DIR_OUT },
      .bRequest = AUDIO_CS_REQ_CUR,
      // UAC1 endpoint control selector: SAMPLING_FREQ_CONTROL = 0x01.
      .wValue = tu_htole16((uint16_t)(0x01 << 8)),
      .wIndex = tu_htole16(s_dac.ep_out.bEndpointAddress),
      .wLength = tu_htole16(3),
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = s_freq_buf,
      .complete_cb = on_set_freq_complete,
      .user_data = 0,
  };
  tuh_control_xfer(&xfer);
}

static bool dac_set_config(uint8_t dev_addr, uint8_t itf_num) {
  // We only drive the streaming interface; report the control interface as
  // immediately configured.
  if (!s_dac.have_as || itf_num != s_dac.as_itf) {
    usbh_driver_set_config_complete(dev_addr, itf_num);
    return true;
  }

  // Kick the setup chain: set alt -> set sample rate -> open EP + stream.
  send_set_interface();
  return true;
}

static void on_set_alt_complete(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    usbh_driver_set_config_complete(s_dac.dev_addr, s_dac.as_itf);
    return;
  }
  if (s_dac.audio_bcd >= 0x0200 && s_dac.clock_id != 0) {
    send_set_sample_rate_uac2();
  } else {
    send_set_sample_rate_uac1();
  }
}

static void on_set_freq_complete(tuh_xfer_t *xfer) {
  // A DAC that only supports 48 kHz may STALL the set-rate request; that's fine,
  // continue anyway.
  (void) xfer;

  if (tuh_edpt_open(s_dac.dev_addr, &s_dac.ep_out)) {
    s_dac.streaming = true;
    submit_first_packet();
  }
  usbh_driver_set_config_complete(s_dac.dev_addr, s_dac.as_itf);
}

// --- Isochronous streaming loop --------------------------------------------
static uint16_t build_packet(void) {
  size_t frames = DSPICO_SAMPLES_PER_MS;
  const size_t max_frames = s_dac.ep_out.wMaxPacketSize /
                            (DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE);
  if (frames > max_frames) frames = max_frames;

  size_t got = 0;
  if (s_fill_cb) got = s_fill_cb(s_pkt, frames);
  if (got == 0) {
    // No source / underrun: send a silent packet to keep the stream alive.
    memset(s_pkt, 0, frames * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE);
    got = frames;
  }
  return (uint16_t)(got * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE);
}

static void on_iso_complete(tuh_xfer_t *xfer);

static void submit_packet(void) {
  if (!s_dac.streaming) return;
  const uint16_t len = build_packet();

  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = s_dac.ep_out.bEndpointAddress,
      .buflen = len,
      .buffer = s_pkt,
      .complete_cb = on_iso_complete,
      .user_data = 0,
  };
  tuh_edpt_xfer(&xfer);
}

static void submit_first_packet(void) { submit_packet(); }

// Iso has no retransmit; even on a (rare) error we keep the cadence going by
// immediately queuing the next frame.
static void on_iso_complete(tuh_xfer_t *xfer) {
  if (xfer->daddr != s_dac.dev_addr) return;
  submit_packet();
}

// Endpoints opened via tuh_edpt_open() report completion through the per-xfer
// complete_cb above, so the class-driver xfer_cb has nothing to do here.
static bool dac_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result,
                        uint32_t xferred_bytes) {
  (void) dev_addr; (void) ep_addr; (void) result; (void) xferred_bytes;
  return true;
}

static void dac_close(uint8_t dev_addr) {
  if (dev_addr != s_dac.dev_addr) return;
  memset(&s_dac, 0, sizeof(s_dac));
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------
static const usbh_class_driver_t s_dac_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name       = "UAC_OUT",   // this field only exists in debug builds
#endif
    .init       = dac_init,
    .open       = dac_open,
    .set_config = dac_set_config,
    .xfer_cb    = dac_xfer_cb,
    .close      = dac_close,
};

usbh_class_driver_t const *usbh_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &s_dac_driver;
}
