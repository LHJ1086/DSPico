// ---------------------------------------------------------------------------
// DSPico — USB descriptors (device side) and descriptor callbacks.
//
// Built from TinyUSB's audio descriptor building blocks so the byte layout and
// lengths are computed by the library rather than hand-counted. See
// usb_descriptors.h for the interface/entity/endpoint map.
// ---------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "pico/unique_id.h"
#include "board_config.h"
#include "debug_log.h"
#include "usb_descriptors.h"

// A development VID/PID pair from the pid.codes test range. Replace with your
// own allocation before shipping. The iProduct string is what the OS shows in
// its output-device list (brief §6a), so make it recognisable.
#define USB_VID   0x1209
#define USB_PID   0xD590
#define USB_BCD   0x0200   // USB 2.0 device, Full Speed

// ---------------------------------------------------------------------------
// Device descriptor
// ---------------------------------------------------------------------------
// Class 0xEF/0x02/0x01 = "Miscellaneous / common class / Interface Association
// Descriptor", required because the audio function uses an IAD.
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    // Windows caches the MS OS 2.0 / WinUSB binding AND the audio device's
    // format/endpoint properties per VID/PID/bcdDevice. Bump this whenever the
    // descriptor layout changes, or a stale (possibly failed) cached state
    // sticks forever. Bumped to 0x0104 with the volume-default/clip-guard
    // release so hosts that cached the earlier (possibly failed) state
    // re-read the device fresh.
    .bcdDevice          = 0x0104,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

// ---------------------------------------------------------------------------
// Configuration descriptor
// ---------------------------------------------------------------------------
// wTotalLength of the class-specific AudioControl interface block. This counts
// ONLY the entity descriptors that FOLLOW the CS-AC header (clock source, input
// terminal, feature unit, output terminal). The TUD_AUDIO_DESC_CS_AC macro adds
// the 9-byte header itself (see its comment: "Do not include
// TUD_AUDIO_DESC_CS_AC_LEN, we already do this here").
//
// Including the header here was a real bug: it made wTotalLength 9 bytes too
// large (73 vs 64), so TinyUSB's audiod skipped past the AS alt-0 interface
// when parsing, failed to find the streaming interface, and silently never
// opened the OUT stream — the PC enumerated the device and drove its volume but
// no audio ever flowed. Matches the proven uac2_speaker_fb example, which also
// omits the header from this sum.
#define UAC2_CS_AC_TOTAL_LEN                       \
  (TUD_AUDIO_DESC_CLK_SRC_LEN                       \
   + TUD_AUDIO_DESC_INPUT_TERM_LEN                  \
   + TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN    \
   + TUD_AUDIO_DESC_OUTPUT_TERM_LEN)

// Feature Unit control bitmap: Master channel (ch0) gets host-readable/writable
// Mute + Volume (brief §1.5 / §6a). L/R channels expose nothing extra.
#define UAC2_FU_CTRL_MASTER                                             \
  ((AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS)                  \
   | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS))

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + DSPICO_UAC2_DESC_TOTAL_LEN + TUD_VENDOR_DESC_LEN)

// String descriptor indices. Declared here (ahead of the configuration and
// string tables that reference them) so STRID_VENDOR is visible where the
// vendor interface descriptor names it.
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_VENDOR,
};

static uint8_t const desc_configuration[] = {
    // Configuration header: 1 config, ITF_NUM_TOTAL interfaces, self-checked
    // total length, bus-powered, 500 mA (worst case incl. a bus-powered DAC).
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 500),

    // --- Interface Association: the two audio interfaces belong together -----
    TUD_AUDIO_DESC_IAD(/*_firstitf*/ ITF_NUM_AUDIO_CONTROL, /*_nitfs*/ 2, /*_stridx*/ 0x00),

    // --- Standard AC interface (no endpoints; alt 0) ------------------------
    TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ ITF_NUM_AUDIO_CONTROL, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),

    // --- Class-specific AC interface header ---------------------------------
    TUD_AUDIO_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_DESKTOP_SPEAKER,
                         /*_totallen*/ UAC2_CS_AC_TOTAL_LEN, /*_ctrl*/ AUDIO_CTRL_NONE),

    // --- Clock Source: internal 48 kHz, frequency host-programmable ----------
    // Advertised as a PROGRAMMABLE clock with a READ/WRITE frequency control,
    // matching TinyUSB's proven uac2_speaker_fb example. Windows' usbaudio2.sys
    // programs the sample rate as part of opening the stream; a read-only fixed
    // clock it cannot set is a known reason it silently declines to stream (the
    // "device mounts + volume works but never streams" symptom). We still
    // support only 48 kHz — the RANGE request pins the host to it, and the
    // SET_CUR handler in uac2_device.c accepts that one value.
    TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ UAC2_ENTITY_CLOCK,
                           /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK,
                           /*_ctrl*/ (AUDIO_CTRL_RW << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS),
                           /*_assocTerm*/ UAC2_ENTITY_INPUT_TERM, /*_stridx*/ 0x00),

    // --- Input Terminal: the USB stream entering the device -----------------
    TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_INPUT_TERM,
                              /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING,
                              /*_assocTerm*/ 0x00, /*_clkid*/ UAC2_ENTITY_CLOCK,
                              /*_nchannelslogical*/ DSPICO_NUM_CHANNELS,
                              /*_channelcfg*/ (AUDIO_CHANNEL_CONFIG_FRONT_LEFT | AUDIO_CHANNEL_CONFIG_FRONT_RIGHT),
                              /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0x0000,
                              /*_stridx*/ 0x00),

    // --- Feature Unit: Volume + Mute (host-controllable) --------------------
    TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL(/*_unitid*/ UAC2_ENTITY_FEATURE_UNIT,
                                            /*_srcid*/ UAC2_ENTITY_INPUT_TERM,
                                            /*_ctrlch0master*/ UAC2_FU_CTRL_MASTER,
                                            /*_ctrlch1*/ 0x00000000,
                                            /*_ctrlch2*/ 0x00000000,
                                            /*_stridx*/ 0x00),

    // --- Output Terminal: toward the (downstream) speaker/DAC ---------------
    TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_OUTPUT_TERM,
                               /*_termtype*/ AUDIO_TERM_TYPE_OUT_GENERIC_SPEAKER,
                               /*_assocTerm*/ 0x00, /*_srcid*/ UAC2_ENTITY_FEATURE_UNIT,
                               /*_clkid*/ UAC2_ENTITY_CLOCK, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),

    // --- Standard AS interface, alt 0 = zero-bandwidth (idle) ---------------
    TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING),
                              /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),

    // --- Standard AS interface, alt 1 = operational (data + feedback EPs) ----
    TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING),
                              /*_altset*/ 0x01, /*_nEPs*/ 0x02, /*_stridx*/ 0x00),

    // --- Class-specific AS interface: PCM, 2 channels -----------------------
    TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_INPUT_TERM,
                             /*_ctrl*/ AUDIO_CTRL_NONE,
                             /*_formattype*/ AUDIO_FORMAT_TYPE_I,
                             /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM,
                             /*_nchannelsphysical*/ DSPICO_NUM_CHANNELS,
                             /*_channelcfg*/ (AUDIO_CHANNEL_CONFIG_FRONT_LEFT | AUDIO_CHANNEL_CONFIG_FRONT_RIGHT),
                             /*_stridx*/ 0x00),

    // --- Type I format: 16-bit in a 2-byte sub-slot (PC-facing) -------------
    // 16-bit is the universally host-renderable format; see board_config.h.
    TUD_AUDIO_DESC_TYPE_I_FORMAT(/*_subslotsize*/ DSPICO_DEV_BYTES_PER_SAMPLE,
                                 /*_bitresolution*/ DSPICO_DEV_RESOLUTION_BITS),

    // --- Isochronous OUT data endpoint (async; rate set by feedback) --------
    TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ EPNUM_AUDIO_OUT,
                                 /*_attr*/ (uint8_t)(TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA),
                                 /*_maxEPsize*/ CFG_TUD_AUDIO_EP_SZ_OUT,
                                 /*_interval*/ 0x01),

    // lock-delay declared in milliseconds (1 ms) like the proven example; an
    // UNDEFINED/0 lock delay is rejected by some hosts for an async data EP.
    TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,
                                /*_ctrl*/ AUDIO_CTRL_NONE,
                                /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,
                                /*_lockdelay*/ 0x0001),

    // --- Isochronous feedback IN endpoint -----------------------------------
    // Full-speed async feedback carries a 3-byte (10.14) value; declare a 4-byte
    // max packet (>= the FS minimum). The descriptor length is 7 regardless.
    TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(/*_ep*/ EPNUM_AUDIO_FB, /*_epsize*/ 4, /*_interval*/ 0x01),

    // --- Vendor interface: the WebUSB EQ config channel (brief §6b) ----------
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64),
};

// Compile-time guard: the hand-declared total length in usb_descriptors.h must
// match what the building-block macros actually produced.
TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
                 "UAC2 config descriptor length mismatch — fix DSPICO_UAC2_DESC_TOTAL_LEN");

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  return desc_configuration;
}

// Boot-time hex dump of our own configuration descriptor, so the exact bytes
// audiod parses (AC interface, CS-AC wTotalLength, the AS alt interfaces) are
// verifiable in the log. Emitted once at startup, before the PC enumerates, so
// it can't be garbled by later traffic.
void dspico_dump_config_desc(void) {
  dlog0("DSPico device: config descriptor (%u bytes):\n",
        (unsigned) sizeof(desc_configuration));
  for (unsigned i = 0; i < sizeof(desc_configuration); i += 16) {
    char line[80];
    int n = snprintf(line, sizeof line, "  %03u:", i);
    for (unsigned j = 0; j < 16 && i + j < sizeof(desc_configuration); j++)
      n += snprintf(line + n, sizeof line - n, " %02X", desc_configuration[i + j]);
    dlog0("%s\n", line);
  }
}

// ---------------------------------------------------------------------------
// String descriptors  (indices defined in the STRID enum above)
// ---------------------------------------------------------------------------
static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: supported language = English (0x0409)
    "DSPico",                    // 1: Manufacturer
    "DSPico EQ Bridge",          // 2: Product (shown in the OS output list)
    NULL,                        // 3: Serial — filled from chip ID at runtime
    "DSPico Config",             // 4: Vendor (WebUSB) interface
};

static uint16_t _desc_str[32 + 1];

// Fill utf16 buffer with the hex of the RP2350 unique board ID. Returns the
// number of 16-bit characters written. Avoids depending on the TinyUSB BSP.
static size_t dspico_get_serial(uint16_t *utf16, size_t max_chars) {
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);

  const char hex[] = "0123456789ABCDEF";
  size_t n = 0;
  for (size_t b = 0; b < PICO_UNIQUE_BOARD_ID_SIZE_BYTES && n + 2 <= max_chars; b++) {
    utf16[n++] = hex[(id.id[b] >> 4) & 0xF];
    utf16[n++] = hex[id.id[b] & 0xF];
  }
  return n;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  switch (index) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      // Derive a unique-per-board serial from the chip unique ID.
      chr_count = dspico_get_serial(_desc_str + 1, 32);
      break;

    default:
      if (index >= TU_ARRAY_SIZE(string_desc_arr)) return NULL;
      const char *str = string_desc_arr[index];
      if (str == NULL) return NULL;

      chr_count = strlen(str);
      if (chr_count > 32) chr_count = 32;
      for (size_t i = 0; i < chr_count; i++) {
        _desc_str[1 + i] = str[i];
      }
      break;
  }

  // First 16-bit word: length (bytes, incl. header) and descriptor type.
  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}

// ---------------------------------------------------------------------------
// BOS + WebUSB + MS OS 2.0 descriptors (driverless Chrome access, brief §4/§6b)
//
// The BOS advertises two platform capabilities: WebUSB (so Chrome can open the
// device and learn the landing page) and MS OS 2.0 (so Windows binds WinUSB to
// the vendor interface with no Zadig). Same pattern as TinyUSB's webusb_serial
// example. The vendor codes are answered in config_usb.c.
// ---------------------------------------------------------------------------
#define VENDOR_REQUEST_WEBUSB    0x21
#define VENDOR_REQUEST_MICROSOFT 0x22

#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

#define MS_OS_20_DESC_LEN 0xB2

static uint8_t const desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),
    // WebUSB: vendor code + landing-page string index (1).
    TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),
    // MS OS 2.0: descriptor-set length + vendor code.
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

uint8_t const *tud_descriptor_bos_cb(void) {
  return desc_bos;
}

// MS OS 2.0 descriptor set: WinUSB compatible ID + DeviceInterfaceGUID for the
// vendor interface (ITF_NUM_VENDOR).
static uint8_t const desc_ms_os_20[] = {
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Configuration subset header: length, type, config index, reserved, total length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    // Function subset header: length, type, first interface, reserved, subset length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

    // Compatible ID descriptor: length, type, compatible ID ("WINUSB"), sub id
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Registry property descriptor: DeviceInterfaceGUIDs (multi-sz)
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // data type (REG_MULTI_SZ), name length
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050), // property data length
    '{', 0x00, '9', 0x00, '7', 0x00, '5', 0x00, 'F', 0x00, '4', 0x00,
    '4', 0x00, 'D', 0x00, '9', 0x00, '-', 0x00, '0', 0x00, 'D', 0x00,
    '0', 0x00, '8', 0x00, '-', 0x00, '4', 0x00, '3', 0x00, 'F', 0x00,
    'D', 0x00, '-', 0x00, '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00,
    '-', 0x00, '1', 0x00, '2', 0x00, '7', 0x00, 'C', 0x00, 'A', 0x00,
    '8', 0x00, 'A', 0x00, 'F', 0x00, 'F', 0x00, 'F', 0x00, '9', 0x00,
    'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,
};
TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "MS OS 2.0 length mismatch");

// WebUSB URL descriptor (landing page shown by Chrome). Points at the GitHub
// Pages deployment of web/. Host + path only; https scheme code = 1.
static const uint8_t desc_url[] = {
    3 + 25, 3, 1,  // bLength, bDescriptorType (URL), bScheme (1 = https)
    'l','h','j','1','0','8','6','.','g','i','t','h','u','b','.','i','o',
    '/','D','S','P','i','c','o','/',
};

const uint8_t *dspico_desc_ms_os_20(uint16_t *len) {
  if (len) *len = sizeof(desc_ms_os_20);
  return desc_ms_os_20;
}
const uint8_t *dspico_desc_webusb_url(uint16_t *len) {
  if (len) *len = sizeof(desc_url);
  return desc_url;
}
