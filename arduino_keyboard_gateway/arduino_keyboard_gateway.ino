/*
 * arduino_keyboard_gateway.ino - Arduino Leonardo (ATmega32U4) keyboard gateway
 * for the accessibility / automated-test input system.
 *
 * ROLE
 * ----
 *   - Acts as a clearly-identified test/accessibility HID keyboard using the
 *     board's OWN Arduino USB identity (see docs/USB_DESCRIPTORS.md). It does
 *     NOT impersonate another vendor's product.
 *   - Receives GW_MSG_KEY_SEQUENCE scripts over UART (from the PC, or from the
 *     Teensy mouse gateway via a serial jumper) and injects the described key
 *     combinations with microsecond-scheduled timing.
 *   - Optionally reads a real keyboard through a USB Host Shield and forwards
 *     every keystroke transparently (full passthrough, nothing hidden). The
 *     host-read path is guarded by ENABLE_USB_HOST below; leave it off if you
 *     have not wired a Host Shield.
 *
 * WIRING (UART link)
 *   PC/CP2102 or Teensy TX  -> Leonardo RX1 (pin 0)
 *   PC/CP2102 or Teensy RX  <- Leonardo TX1 (pin 1)
 *   GND common.
 *   Serial1 is the hardware UART; Serial (USB CDC) is left for debug.
 */

#include <Arduino.h>
#include <Keyboard.h>

extern "C" {
  #include "../protocol/gateway_protocol.h"
}
#include "timing_engine.h"

/* ----- Configuration ------------------------------------------------------ */
#define LINK_BAUD          115200
#define ENABLE_USB_HOST    0       /* set to 1 once a USB Host Shield is wired */
#define DEFAULT_TAP_HOLD_US 8000u  /* >= 1 host poll so taps register          */
#define STATUS_LED         LED_BUILTIN

/* ----- Link state --------------------------------------------------------- */
static gw_parser_t  g_parser;
static uint8_t      g_tx_seq = 0;

static void link_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[GW_MAX_FRAME];
    size_t n = gw_build_frame(frame, type, g_tx_seq++, payload, len);
    if (n) {
        Serial1.write(frame, n);
    }
}

static void send_ack(uint8_t seq)
{
    uint8_t p = seq;
    link_send(GW_MSG_ACK, &p, 1);
}

static void send_nak(uint8_t seq, uint8_t reason)
{
    uint8_t p[2] = { seq, reason };
    link_send(GW_MSG_NAK, p, 2);
}

/* ----- Key sequence execution -------------------------------------------- */
/*
 * Payload format (see gateway_protocol.h):
 *   [count][rec0..recN], each rec = action, keycode, mod, delayLo, delayHi
 */
static bool run_key_sequence(const uint8_t *payload, uint8_t len)
{
    if (len < 1) {
        return false;
    }
    uint8_t count = payload[0];
    if ((uint16_t)count * GW_KEYREC_SIZE + 1u != len) {
        return false; /* malformed: declared count != actual bytes */
    }

    digitalWrite(STATUS_LED, HIGH);
    const uint8_t *rec = &payload[1];
    for (uint8_t i = 0; i < count; i++, rec += GW_KEYREC_SIZE) {
        uint8_t  action  = rec[0];
        uint8_t  keycode = rec[1];
        uint8_t  mods    = rec[2];
        uint16_t delay_us = (uint16_t)rec[3] | ((uint16_t)rec[4] << 8);

        if (!te_execute_record(action, keycode, mods, delay_us,
                               DEFAULT_TAP_HOLD_US)) {
            Keyboard.releaseAll(); /* fail safe: never leave keys stuck */
            digitalWrite(STATUS_LED, LOW);
            return false;
        }
    }
    /* Safety net: ensure nothing is left held after a script completes. */
    Keyboard.releaseAll();
    digitalWrite(STATUS_LED, LOW);
    return true;
}

/* ----- Frame dispatch ----------------------------------------------------- */
static void handle_frame(const gw_frame_t *f)
{
    switch (f->type) {
    case GW_MSG_PING:
        link_send(GW_MSG_PONG, f->payload, f->len);
        break;

    case GW_MSG_KEY_SEQUENCE:
        if (run_key_sequence(f->payload, f->len)) {
            send_ack(f->seq);
        } else {
            send_nak(f->seq, GW_NAK_BAD_LEN);
        }
        break;

    case GW_MSG_RESET:
        Keyboard.releaseAll();
        g_tx_seq = 0;
        send_ack(f->seq);
        break;

    case GW_MSG_ACK:
    case GW_MSG_NAK:
    case GW_MSG_PONG:
        /* informational; nothing to do in this minimal slave role */
        break;

    default:
        send_nak(f->seq, GW_NAK_UNKNOWN_TYPE);
        break;
    }
}

/* ----- Arduino entry points ---------------------------------------------- */
void setup()
{
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    Keyboard.begin();
    Serial1.begin(LINK_BAUD);
    gw_parser_init(&g_parser);

#if ENABLE_USB_HOST
    /* Initialise USB Host Shield here and forward real keystrokes verbatim.
     * Left as an integration point; see docs/HARDWARE.md. Transparent
     * passthrough only — no key is suppressed or rewritten. */
#endif
}

void loop()
{
    while (Serial1.available()) {
        gw_frame_t f;
        gw_parse_result_t r = gw_parser_feed(&g_parser, (uint8_t)Serial1.read(), &f);
        if (r == GW_PARSE_FRAME) {
            handle_frame(&f);
        } else if (r == GW_PARSE_CRC_ERR) {
            send_nak(0xFF, GW_NAK_CRC); /* seq unknown on CRC failure */
        }
    }

#if ENABLE_USB_HOST
    /* Poll USB Host Shield and forward real keyboard reports unchanged. */
#endif
}
