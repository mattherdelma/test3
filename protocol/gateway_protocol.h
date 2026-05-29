/*
 * gateway_protocol.h - Binary UART framing for the accessibility / automated
 * test input gateway.
 *
 * Design goals:
 *   - Stable, low-latency, self-synchronising framing.
 *   - CRC-16 integrity check on every frame.
 *   - Sequence numbers + ACK/NAK so the link can recover from drops/corruption.
 *   - Fixed, bounded buffers (no malloc) so it runs on the Leonardo's AVR.
 *
 * COMPLIANCE NOTE
 * ---------------
 * This protocol carries:
 *   - device->host  notifications (e.g. an accessibility button was pressed),
 *   - host->device  automated-test injection scripts (an explicit, documented
 *     test fixture using the device's OWN USB identity).
 * It deliberately does NOT carry a "merge these deltas into the live mouse
 * stream while hiding the trigger" path. Injected input is sourced by the
 * gateway as a clearly-identified test device, never blended with and disguised
 * as the operator's real hand input.
 *
 * Frame layout (on the wire):
 *
 *   +------+------+------+------+------+------------------+--------+--------+
 *   | SOF  | VER  | TYPE | SEQ  | LEN  |  PAYLOAD[LEN]    | CRC_HI | CRC_LO |
 *   +------+------+------+------+------+------------------+--------+--------+
 *     0xAA  0x01   1B     1B     1B      0..255 bytes        CRC-16/CCITT
 *
 *   CRC is computed over VER..PAYLOAD inclusive (everything between SOF and
 *   the CRC field). The receiver resynchronises by scanning for SOF and then
 *   validating LEN + CRC; a bad frame is dropped and (optionally) NAK'd.
 */
#ifndef GATEWAY_PROTOCOL_H
#define GATEWAY_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include "crc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GW_SOF              0xAAu
#define GW_PROTO_VERSION    0x01u

#define GW_MAX_PAYLOAD      255u
#define GW_HEADER_LEN       5u      /* SOF VER TYPE SEQ LEN */
#define GW_CRC_LEN          2u
#define GW_MAX_FRAME        (GW_HEADER_LEN + GW_MAX_PAYLOAD + GW_CRC_LEN)

/* Message types. device->host (D2H), host->device (H2D), or either (BI). */
typedef enum {
    GW_MSG_PING          = 0x01, /* BI  : link keepalive / latency probe       */
    GW_MSG_PONG          = 0x02, /* BI  : reply to PING (echoes payload)       */
    GW_MSG_ACK           = 0x03, /* BI  : payload[0] = acked SEQ               */
    GW_MSG_NAK           = 0x04, /* BI  : payload[0]=SEQ, payload[1]=reason     */
    GW_MSG_STATUS        = 0x05, /* D2H : device status report                 */

    GW_MSG_BUTTON_EVENT  = 0x10, /* D2H : accessibility button state change    */

    GW_MSG_KEY_SEQUENCE  = 0x20, /* H2D : timed key-event script (keyboard gw) */
    GW_MSG_MOUSE_TEST    = 0x21, /* H2D : explicit test-fixture mouse action   */

    GW_MSG_RESET         = 0x7E  /* BI  : reset sequence counters / abort run  */
} gw_msg_type_t;

/* NAK reason codes. */
typedef enum {
    GW_NAK_CRC          = 0x01,
    GW_NAK_BAD_LEN      = 0x02,
    GW_NAK_BAD_VERSION  = 0x03,
    GW_NAK_UNKNOWN_TYPE = 0x04,
    GW_NAK_BUSY         = 0x05,
    GW_NAK_OVERFLOW     = 0x06
} gw_nak_reason_t;

/* ---- Key event encoding (payload of GW_MSG_KEY_SEQUENCE) ----------------- *
 *
 * Payload = [count] followed by `count` records of 5 bytes:
 *
 *   byte 0      : action  (gw_key_action_t)
 *   byte 1      : keycode (HID usage / Arduino Keyboard key value)
 *   byte 2      : modifier bitmap (gw_key_mod_t, OR-able)
 *   byte 3..4   : delay_before_us, little-endian (microseconds to wait BEFORE
 *                 executing this record; lets the host express precise timing)
 *
 * Records are executed in order. See timing_engine.h for execution semantics.
 */
typedef enum {
    GW_KEY_PRESS   = 0x01, /* hold key down                                  */
    GW_KEY_RELEASE = 0x02, /* release key                                    */
    GW_KEY_TAP     = 0x03, /* press + (optional hold) + release              */
    GW_KEY_RELALL  = 0x04  /* release all currently-held keys/modifiers      */
} gw_key_action_t;

typedef enum {
    GW_MOD_LCTRL  = 0x01,
    GW_MOD_LSHIFT = 0x02,
    GW_MOD_LALT   = 0x04,
    GW_MOD_LGUI   = 0x08,
    GW_MOD_RCTRL  = 0x10,
    GW_MOD_RSHIFT = 0x20,
    GW_MOD_RALT   = 0x40,
    GW_MOD_RGUI   = 0x80
} gw_key_mod_t;

#define GW_KEYREC_SIZE 5u
#define GW_MAX_KEY_RECORDS ((GW_MAX_PAYLOAD - 1u) / GW_KEYREC_SIZE) /* 50 */

/* ---- Parser (incremental, byte-at-a-time, reentrant via context) --------- */

typedef enum {
    GW_WAIT_SOF = 0,
    GW_WAIT_VER,
    GW_WAIT_TYPE,
    GW_WAIT_SEQ,
    GW_WAIT_LEN,
    GW_WAIT_PAYLOAD,
    GW_WAIT_CRC_HI,
    GW_WAIT_CRC_LO
} gw_parse_state_t;

typedef struct {
    gw_parse_state_t state;
    uint8_t  type;
    uint8_t  seq;
    uint8_t  len;
    uint8_t  idx;             /* payload bytes received so far                */
    uint8_t  payload[GW_MAX_PAYLOAD];
    uint16_t crc_calc;        /* running CRC over VER..PAYLOAD                */
    uint16_t crc_rx;
} gw_parser_t;

/* A fully-decoded frame handed back to the caller. */
typedef struct {
    uint8_t  type;
    uint8_t  seq;
    uint8_t  len;
    const uint8_t *payload;   /* points into the parser's buffer             */
} gw_frame_t;

/* Result of feeding one byte to the parser. */
typedef enum {
    GW_PARSE_IDLE  = 0, /* more bytes needed, nothing wrong                  */
    GW_PARSE_FRAME = 1, /* a valid frame is ready in *out                    */
    GW_PARSE_CRC_ERR    /* a frame completed but failed CRC (caller may NAK) */
} gw_parse_result_t;

/* Reset a parser to its initial state. */
void gw_parser_init(gw_parser_t *p);

/* Feed one received byte. On GW_PARSE_FRAME, *out is populated. */
gw_parse_result_t gw_parser_feed(gw_parser_t *p, uint8_t b, gw_frame_t *out);

/* ---- Frame builder ------------------------------------------------------- *
 * Serialises a frame into `buf` (must be >= GW_MAX_FRAME). Returns the number
 * of bytes written, or 0 if payload_len is out of range.
 */
size_t gw_build_frame(uint8_t *buf, uint8_t type, uint8_t seq,
                      const uint8_t *payload, uint8_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_PROTOCOL_H */
