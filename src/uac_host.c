// ---------------------------------------------------------------------------
// DSPico — USB Audio host driver over Pico-PIO-USB (Phase 1b).
//
// Custom TinyUSB application host driver. See uac_host.h for the contract and
// the big caveat: isochronous OUT over the bit-banged PIO port is unproven, so
// treat this as the scaffold you iterate on with a logic analyzer.
// ---------------------------------------------------------------------------
#include <stdio.h>
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

  // Feature Unit discovery, to unmute/set volume on the DAC's playback path
  // (real hosts always do this; DACs may power up muted or at minimum volume).
  uint8_t  fu_ids[8];       // every Feature Unit seen in the AC block
  uint8_t  fu_count;
  uint8_t  ot_source_id;    // bSourceID of the speaker/headphone Output Terminal
  uint8_t  fu_id;           // the FU we drive (resolved in dac_set_config)

  uint8_t  as_itf;          // Audio Streaming interface number
  uint8_t  as_alt;          // operational alt setting (has the iso EP)
  bool     have_as;

  uint8_t  subslot;         // wire bytes/sample: 3 = native 24-bit, 2 = 16-bit fallback
  uint8_t  bits;            // valid bits/sample of the chosen alt (for diagnostics)

  tusb_desc_endpoint_t ep_out;   // the isochronous OUT endpoint descriptor
  bool     have_ep;

  bool     incompatible;    // audio device seen, but nothing we can stream to
  bool     streaming;
} dac_dev_t;

static dac_dev_t s_dac;
static uac_host_fill_cb_t s_fill_cb = NULL;

// One outstanding iso packet at a time; DMA-aligned.
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t s_pkt[UAC_HOST_PKT_MAX];

void uac_host_set_source(uac_host_fill_cb_t cb) { s_fill_cb = cb; }
bool uac_host_is_streaming(void) { return s_dac.in_use && s_dac.streaming; }

uac_host_state_t uac_host_state(void) {
  if (!s_dac.in_use)      return UAC_HOST_NO_DAC;
  if (s_dac.streaming)    return UAC_HOST_STREAMING;
  if (s_dac.incompatible || !s_dac.have_as) return UAC_HOST_INCOMPATIBLE;
  return UAC_HOST_SETUP;
}

// ---------------------------------------------------------------------------
// Descriptor parsing helpers
// ---------------------------------------------------------------------------
static uint8_t const *desc_next(uint8_t const *p) { return p + p[0]; }

// Format facts collected for one AS alternate setting while walking its
// descriptors; judged when we reach the alt's isochronous OUT data endpoint.
typedef struct {
  uint8_t itf, alt;
  uint8_t channels;    // 0 until seen
  uint8_t subslot;     // wire bytes per sample, 0 until seen
  uint8_t bits;        // valid bits per sample
  bool    pcm;         // format is Type I PCM
  bool    rate_ok;     // 48 kHz supported (assumed true unless a UAC1 list says no)
  bool    in_as_itf;   // currently inside an AS interface's descriptors
} alt_cand_t;

// Accept an alt setting if it can take our stream: stereo Type-I PCM at 48 kHz
// in a 3-byte (native 24-bit) or 2-byte (16-bit fallback) subslot. Prefer a
// 24-bit alt over a 16-bit one; within the same width keep the first found.
static void consider_candidate(dac_dev_t *d, const alt_cand_t *c,
                               tusb_desc_endpoint_t const *ep) {
  // Log every candidate so an "incompatible" verdict is diagnosable from the
  // UART without a USB analyzer.
  printf("DSPico host: alt itf %u.%u: ch=%u subslot=%u bits=%u pcm=%d rate48=%d maxpkt=%u\n",
         c->itf, c->alt, c->channels, c->subslot, c->bits,
         (int) c->pcm, (int) c->rate_ok, ep->wMaxPacketSize);
  if (!c->pcm || !c->rate_ok) return;
  if (c->channels != DSPICO_NUM_CHANNELS) return;
  if (c->subslot != 3 && c->subslot != 2) return;
  if (ep->wMaxPacketSize < DSPICO_NUM_CHANNELS * c->subslot) return;
  if (d->have_ep && d->subslot >= c->subslot) return;   // existing pick is >= tier

  d->as_itf  = c->itf;
  d->as_alt  = c->alt;
  d->subslot = c->subslot;
  d->bits    = c->bits;
  memcpy(&d->ep_out, ep, sizeof(*ep));
  d->have_ep = d->have_as = true;
}

// UAC1 Type I format descriptor carries the supported sample rates; check that
// 48 kHz is among them (discrete list) or inside them (continuous range).
// Reads stay within the descriptor's own bLength.
static bool uac1_rate_list_has_48k(uint8_t const *p, uint8_t dlen) {
  const uint8_t n = p[7];   // bSamFreqType: 0 = continuous min..max, else count
  if (n == 0) {
    if (dlen < 14) return false;
    const uint32_t lo = (uint32_t) (p[8]  | (p[9]  << 8) | (p[10] << 16));
    const uint32_t hi = (uint32_t) (p[11] | (p[12] << 8) | (p[13] << 16));
    return lo <= DSPICO_SAMPLE_RATE_HZ && DSPICO_SAMPLE_RATE_HZ <= hi;
  }
  for (uint8_t i = 0; i < n && (uint16_t) (8 + 3 * i + 3) <= dlen; i++) {
    const uint8_t *f = p + 8 + 3 * i;
    if ((uint32_t) (f[0] | (f[1] << 8) | (f[2] << 16)) == DSPICO_SAMPLE_RATE_HZ)
      return true;
  }
  return false;
}

// Single-pass parse of the whole audio-function descriptor block. With an IAD
// (typical UAC2) TinyUSB hands us the entire function at once; without one
// (typical UAC1) it hands us one interface at a time — walking `len` bytes and
// tracking the current interface context handles both.
//
// Records: the AC interface number, UAC version, a UAC2 clock-source id, and —
// via consider_candidate() — the best alt setting whose Type-I PCM format we
// can actually feed (stereo 48 kHz, 24-bit preferred, 16-bit fallback).
static void parse_audio_function(dac_dev_t *d, uint8_t const *p, uint16_t len) {
  uint8_t const *end = p + len;
  uint8_t cur_sub = 0xFF;                 // current interface subclass
  alt_cand_t cand = { .in_as_itf = false };

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    const uint8_t dtype = p[1];
    const uint8_t dlen  = p[0];

    if (dtype == TUSB_DESC_INTERFACE) {
      tusb_desc_interface_t const *itf = (tusb_desc_interface_t const *) p;
      cand.in_as_itf = false;
      if (itf->bInterfaceClass == TUSB_CLASS_AUDIO) {
        cur_sub = itf->bInterfaceSubClass;
        if (cur_sub == AUDIO_SUBCLASS_CONTROL) {
          d->ac_itf = itf->bInterfaceNumber;
        } else if (cur_sub == AUDIO_SUBCLASS_STREAMING) {
          cand = (alt_cand_t) {
            .itf = itf->bInterfaceNumber, .alt = itf->bAlternateSetting,
            .rate_ok = true, .in_as_itf = true,
          };
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
      } else if (subtype == AUDIO_CS_AC_INTERFACE_FEATURE_UNIT && dlen >= 5) {
        if (d->fu_count < sizeof(d->fu_ids)) d->fu_ids[d->fu_count++] = p[3];
      } else if (subtype == AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL && dlen >= 8) {
        // Remember the speaker/headphone terminal's source: if that source is
        // a Feature Unit, it's the one controlling playback volume/mute.
        const uint16_t ttype = tu_unaligned_read16(p + 4);
        if ((ttype & 0xFF00) == 0x0300) d->ot_source_id = p[7];   // 0x03xx = output
      }
    } else if (dtype == TUSB_DESC_CS_INTERFACE && cand.in_as_itf) {
      const uint8_t subtype = p[2];
      const bool uac2 = d->audio_bcd >= 0x0200;
      if (subtype == AUDIO_CS_AS_INTERFACE_AS_GENERAL) {
        if (uac2 && dlen >= 16) {
          cand.pcm      = (p[6] & 0x01) != 0;   // bmFormats bit 0 = Type I PCM
          cand.channels = p[10];                // bNrChannels
        } else if (!uac2 && dlen >= 7) {
          cand.pcm = (uint16_t) (p[5] | (p[6] << 8)) == 0x0001;   // wFormatTag PCM
        }
      } else if (subtype == AUDIO_CS_AS_INTERFACE_FORMAT_TYPE && p[3] == 1 /*Type I*/) {
        if (uac2 && dlen >= 6) {
          cand.subslot = p[4];                  // bSubslotSize
          cand.bits    = p[5];                  // bBitResolution
        } else if (!uac2 && dlen >= 8) {
          cand.channels = p[4];                 // bNrChannels
          cand.subslot  = p[5];                 // bSubframeSize
          cand.bits     = p[6];                 // bBitResolution
          cand.rate_ok  = uac1_rate_list_has_48k(p, dlen);
        }
      }
    } else if (dtype == TUSB_DESC_ENDPOINT && cand.in_as_itf) {
      tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *) p;
      const bool is_iso  = (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS);
      const bool is_out  = (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_OUT);
      const bool is_data = (ep->bmAttributes.usage == 0x00);   // not feedback
      if (is_iso && is_out && is_data) {
        consider_candidate(d, &cand, ep);
      }
    }
    p = desc_next(p);
  }
}

// ---------------------------------------------------------------------------
// TinyUSB host class-driver callbacks
// ---------------------------------------------------------------------------
static bool dac_init(void) {
  memset(&s_dac, 0, sizeof(s_dac));
  return true;
}

static bool dac_open(uint8_t rhport, uint8_t dev_addr,
                     tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  if (itf_desc->bInterfaceClass != TUSB_CLASS_AUDIO) return false;

  // Bind to the first audio device we see (single-DAC spike).
  if (s_dac.in_use && s_dac.dev_addr != dev_addr) return false;
  if (!s_dac.in_use) {
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    printf("DSPico host: audio device attached (addr %u, VID:PID %04X:%04X)\n",
           dev_addr, vid, pid);
  }
  s_dac.in_use = true;
  s_dac.dev_addr = dev_addr;
  s_dac.rhport = rhport;

  parse_audio_function(&s_dac, (uint8_t const *) itf_desc, max_len);
  return true;   // claim the audio interface(s) in this block
}

// --- Setup state machine (runs after enumeration) ---------------------------
// Step order matters and differs by UAC version (matching what Linux/Windows
// do, which is what DACs are tested against):
//   UAC2: SET_CUR(clock = 48 kHz) FIRST (alt still 0), then SET_INTERFACE(alt)
//         — many DACs reject or ignore a rate change on an already-active alt.
//   UAC1: SET_INTERFACE(alt) first, then the endpoint SAMPLING_FREQ control
//         — the endpoint only exists once the alt is active.
static void submit_first_packet(void);
static void on_set_alt_complete(tuh_xfer_t *xfer);
static void on_set_freq_complete(tuh_xfer_t *xfer);

static bool setup_is_uac2(void) {
  return s_dac.audio_bcd >= 0x0200 && s_dac.clock_id != 0;
}

// --- DAC Feature-Unit init (unmute + 0 dB volume) ----------------------------
// Real hosts set the playback Feature Unit's mute/volume right after
// enumeration, and DACs are tested against that — some power up muted or at
// minimum volume and stay silent forever if nobody does it. The wire format is
// the same for UAC1 (SET_CUR) and UAC2 (CUR): bRequest 0x01, wValue =
// (selector << 8) | channel, wIndex = (fu_id << 8) | ac_itf. Controls a DAC
// doesn't implement simply STALL; every step tolerates that and moves on.
static void finish_setup(void);

static const struct { uint8_t sel, ch, len; } s_fu_steps[] = {
  { AUDIO_FU_CTRL_MUTE,   0, 1 }, { AUDIO_FU_CTRL_MUTE,   1, 1 },
  { AUDIO_FU_CTRL_MUTE,   2, 1 },
  { AUDIO_FU_CTRL_VOLUME, 0, 2 }, { AUDIO_FU_CTRL_VOLUME, 1, 2 },
  { AUDIO_FU_CTRL_VOLUME, 2, 2 },
};
static uint8_t s_fu_step;
static uint8_t s_fu_buf[2];   // mute: {0}; volume: 0x0000 = 0.0 dB (1/256 dB units)

static void fu_send_next(void);

static void on_fu_step_complete(tuh_xfer_t *xfer) {
  const uint8_t i = s_fu_step - 1;
  printf("DSPico host: FU %u %s ch%u -> %s\n", s_dac.fu_id,
         s_fu_steps[i].sel == AUDIO_FU_CTRL_MUTE ? "unmute" : "vol 0dB",
         s_fu_steps[i].ch,
         xfer->result == XFER_RESULT_SUCCESS ? "OK" : "stalled (not implemented)");
  fu_send_next();
}

static void fu_send_next(void) {
  if (s_dac.fu_id == 0 || s_fu_step >= TU_ARRAY_SIZE(s_fu_steps)) {
    finish_setup();
    return;
  }
  const uint8_t i = s_fu_step++;
  s_fu_buf[0] = 0; s_fu_buf[1] = 0;   // unmuted / 0.0 dB

  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_INTERFACE,
                             .type = TUSB_REQ_TYPE_CLASS,
                             .direction = TUSB_DIR_OUT },
      .bRequest = AUDIO_CS_REQ_CUR,   // == UAC1 SET_CUR (0x01)
      .wValue = tu_htole16((uint16_t) ((s_fu_steps[i].sel << 8) | s_fu_steps[i].ch)),
      .wIndex = tu_htole16((uint16_t) ((s_dac.fu_id << 8) | s_dac.ac_itf)),
      .wLength = tu_htole16(s_fu_steps[i].len),
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = s_fu_buf,
      .complete_cb = on_fu_step_complete,
      .user_data = 0,
  };
  tuh_control_xfer(&xfer);
}

static void start_fu_init(void) {
  s_fu_step = 0;
  if (s_dac.fu_id) {
    printf("DSPico host: initialising DAC Feature Unit %u (unmute + 0 dB)\n",
           s_dac.fu_id);
  } else {
    printf("DSPico host: no Feature Unit found — skipping volume init\n");
  }
  fu_send_next();
}

// Final step for both protocols: open the iso OUT endpoint and start pumping.
static void finish_setup(void) {
  if (tuh_edpt_open(s_dac.dev_addr, &s_dac.ep_out)) {
    s_dac.streaming = true;
    printf("DSPico host: streaming started (EP 0x%02X, %u-byte subslot)\n",
           s_dac.ep_out.bEndpointAddress, s_dac.subslot);
    submit_first_packet();
  } else {
    s_dac.incompatible = true;
    printf("DSPico host: ERROR — tuh_edpt_open(0x%02X) failed\n",
           s_dac.ep_out.bEndpointAddress);
  }
  usbh_driver_set_config_complete(s_dac.dev_addr, s_dac.as_itf);
}

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
    if (!s_dac.have_as && itf_num == s_dac.ac_itf) {
      s_dac.incompatible = true;
      printf("DSPico host: DAC has no stereo 48 kHz Type-I PCM alt (16/24-bit) — not streaming\n");
    }
    usbh_driver_set_config_complete(dev_addr, itf_num);
    return true;
  }

  printf("DSPico host: DAC (UAC%c) itf %u alt %u — %u-bit in %u-byte subslot, EP 0x%02X maxpkt %u%s\n",
         setup_is_uac2() ? '2' : '1',
         s_dac.as_itf, s_dac.as_alt, s_dac.bits, s_dac.subslot,
         s_dac.ep_out.bEndpointAddress, s_dac.ep_out.wMaxPacketSize,
         s_dac.subslot == 2 ? " (16-bit fallback, truncating)" : "");

  // Resolve the playback Feature Unit for the volume-init step: prefer the
  // unit that feeds the speaker/headphone Output Terminal, else the first.
  s_dac.fu_id = 0;
  for (uint8_t i = 0; i < s_dac.fu_count; i++) {
    if (s_dac.fu_ids[i] == s_dac.ot_source_id) { s_dac.fu_id = s_dac.ot_source_id; break; }
  }
  if (s_dac.fu_id == 0 && s_dac.fu_count > 0) s_dac.fu_id = s_dac.fu_ids[0];

  // Kick the setup chain (see the ordering note above).
  if (setup_is_uac2()) {
    send_set_sample_rate_uac2();   // rate first, then alt
  } else {
    send_set_interface();          // alt first, then endpoint rate
  }
  return true;
}

static void on_set_alt_complete(tuh_xfer_t *xfer) {
  printf("DSPico host: SET_INTERFACE(itf %u, alt %u) -> %s\n",
         s_dac.as_itf, s_dac.as_alt,
         xfer->result == XFER_RESULT_SUCCESS ? "OK" : "FAILED");
  if (xfer->result != XFER_RESULT_SUCCESS) {
    s_dac.incompatible = true;
    usbh_driver_set_config_complete(s_dac.dev_addr, s_dac.as_itf);
    return;
  }
  if (setup_is_uac2()) {
    start_fu_init();               // UAC2: rate already set — unmute, then stream
  } else {
    send_set_sample_rate_uac1();   // UAC1: rate lives on the (now active) EP
  }
}

static void on_set_freq_complete(tuh_xfer_t *xfer) {
  // A DAC that only supports 48 kHz may STALL the set-rate request; that's
  // fine — its fixed rate is the one we want anyway. Log it either way.
  printf("DSPico host: SET sample rate 48000 -> %s\n",
         xfer->result == XFER_RESULT_SUCCESS ? "OK" : "stalled/failed (continuing)");
  if (setup_is_uac2()) {
    send_set_interface();          // UAC2: now activate the alt
  } else {
    start_fu_init();               // UAC1: alt+rate done — unmute, then stream
  }
}

// --- Isochronous streaming loop --------------------------------------------
static uint16_t build_packet(void) {
  const uint8_t  wire_bps = s_dac.subslot;                       // 3 or 2
  const uint16_t wire_bpf = DSPICO_NUM_CHANNELS * wire_bps;      // bytes/frame on the wire

  size_t frames = DSPICO_SAMPLES_PER_MS;
  const size_t max_frames = s_dac.ep_out.wMaxPacketSize / wire_bpf;
  if (frames > max_frames) frames = max_frames;

  // The source always produces the native 24-bit/3-byte format (6 B/frame).
  size_t got = 0;
  if (s_fill_cb) got = s_fill_cb(s_pkt, frames);
  if (got == 0) {
    // No source / underrun: send a silent packet to keep the stream alive.
    memset(s_pkt, 0, frames * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE);
    got = frames;
  }

  if (wire_bps == 2) {
    // 16-bit fallback alt: truncate each 24-bit LE sample to its top 16 bits,
    // in place (the destination never catches up with the source).
    const size_t samples = got * DSPICO_NUM_CHANNELS;
    for (size_t s = 0; s < samples; s++) {
      s_pkt[s * 2 + 0] = s_pkt[s * 3 + 1];
      s_pkt[s * 2 + 1] = s_pkt[s * 3 + 2];
    }
  }
  return (uint16_t)(got * wire_bpf);
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
// immediately queuing the next frame. A periodic heartbeat on the UART proves
// packets are actually flowing (~1000/s expected at 48 kHz).
static void on_iso_complete(tuh_xfer_t *xfer) {
  if (xfer->daddr != s_dac.dev_addr) return;
  static uint32_t pkts;
  if ((++pkts & 0x1FFF) == 0) {   // every 8192 packets ≈ 8 s
    printf("DSPico host: streaming heartbeat — %lu iso packets sent\n",
           (unsigned long) pkts);
  }
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
  printf("DSPico host: DAC disconnected\n");
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
