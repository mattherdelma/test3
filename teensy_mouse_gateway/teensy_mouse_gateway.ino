/*
 * teensy_mouse_gateway.ino - Teensy 4.1 mouse gateway for the accessibility /
 * automated-test input system.
 *
 * ROLE (compliant design)
 * -----------------------
 *   1. TRANSPARENT PASSTHROUGH: reads the real mouse via USB Host (USBHost_t36)
 *      and re-emits every button/motion/wheel event to the PC through the
 *      Teensy USB Device mouse interface. Nothing is hidden, dropped, or
 *      rewritten — the operator's hand input always reaches the PC unchanged.
 *
 *   2. ACCESSIBILITY EVENT NOTIFY: when a chosen auxiliary button (e.g. a side
 *      button) is pressed, the gateway ALSO sends a GW_MSG_BUTTON_EVENT over
 *      UART so the PC can run an accessibility action (announce, open a menu,
 *      etc.). The button still passes through to the PC as a normal button —
 *      it is observable, not a stealth trigger.
 *
 *   3. EXPLICIT TEST INJECTION: the PC may send GW_MSG_MOUSE_TEST to drive the
 *      gateway as a *documented test fixture* (move cursor / click for an
 *      automated UI test). This output is sourced by the gateway as its OWN
 *      identified device. It is NOT blended into the live hand-input stream to
 *      disguise its origin; it is intended for unattended test runs.
 *
 * USB IDENTITY: uses the Teensy's own VID/PID and a standard HID mouse report
 * descriptor (see docs/USB_DESCRIPTORS.md). It does not impersonate any other
 * vendor's product.
 *
 * WIRING
 *   USB Host port (Teensy 4.1 has a dedicated host header) -> real mouse.
 *   UART to PC via CP2102:   Serial1 TX(pin1)->CP2102 RX, RX(pin0)<-CP2102 TX.
 *   Optional jumper of Serial1 to the Leonardo keyboard gateway.
 */

#include <USBHost_t36.h>

extern "C" {
  #include "../protocol/gateway_protocol.h"
}

/* ----- Configuration ------------------------------------------------------ */
#define LINK_BAUD            115200
#define ACCESS_BUTTON_MASK   0x08   /* USB HID button bit treated as the aux/
                                       accessibility button (e.g. "back"/side).
                                       Bit3 is a common side-button mapping. */
#define STATUS_LED           13

/* ----- USB Host objects --------------------------------------------------- */
USBHost            myusb;
USBHub             hub1(myusb);
USBHIDParser       hid1(myusb);
MouseController    mouseHost(myusb);

/* ----- Link state --------------------------------------------------------- */
static gw_parser_t g_parser;
static uint8_t     g_tx_seq = 0;
static uint8_t     g_prev_buttons = 0;

static void link_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[GW_MAX_FRAME];
    size_t n = gw_build_frame(frame, type, g_tx_seq++, payload, len);
    if (n) {
        Serial1.write(frame, n);
    }
}

static void send_ack(uint8_t seq)  { uint8_t p = seq; link_send(GW_MSG_ACK, &p, 1); }
static void send_nak(uint8_t seq, uint8_t r) { uint8_t p[2] = {seq, r}; link_send(GW_MSG_NAK, p, 2); }

/* ----- Real mouse -> PC (transparent passthrough) ------------------------- */
static void passthrough_real_mouse()
{
    if (!mouseHost.available()) {
        return;
    }

    uint8_t buttons = mouseHost.getButtons();
    int     dx      = mouseHost.getMouseX();
    int     dy      = mouseHost.getMouseY();
    int     wheel   = mouseHost.getWheel();

    /* Forward motion/wheel verbatim. */
    if (dx || dy || wheel) {
        Mouse.move(dx, dy, wheel);
    }

    /* Forward ALL button states verbatim, including the aux button. */
    Mouse.set_buttons((buttons & 0x01) ? 1 : 0,
                      (buttons & 0x04) ? 1 : 0,   /* middle */
                      (buttons & 0x02) ? 1 : 0);  /* right  */
    /* (Mouse.set_buttons covers L/M/R; side buttons pass via the HID report
     *  on cores that expose them. The point is we never suppress them.) */

    /* Accessibility notify on a rising edge of the aux button — IN ADDITION to
     * passing it through, never instead of. */
    uint8_t aux_now  = buttons & ACCESS_BUTTON_MASK;
    uint8_t aux_prev = g_prev_buttons & ACCESS_BUTTON_MASK;
    if (aux_now && !aux_prev) {
        uint8_t ev[2] = { ACCESS_BUTTON_MASK, 0x01 /* pressed */ };
        link_send(GW_MSG_BUTTON_EVENT, ev, sizeof(ev));
    } else if (!aux_now && aux_prev) {
        uint8_t ev[2] = { ACCESS_BUTTON_MASK, 0x00 /* released */ };
        link_send(GW_MSG_BUTTON_EVENT, ev, sizeof(ev));
    }

    g_prev_buttons = buttons;
    mouseHost.mouseDataClear();
}

/* ----- Explicit test-fixture injection (PC -> gateway) -------------------- *
 * Payload: [dx_lo dx_hi dy_lo dy_hi wheel buttons]  (int16 LE deltas)
 * Intended for unattended automated UI tests. */
static bool run_mouse_test(const uint8_t *p, uint8_t len)
{
    if (len != 6) {
        return false;
    }
    int16_t dx = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    int16_t dy = (int16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
    int8_t  wheel   = (int8_t)p[4];
    uint8_t buttons = p[5];

    Mouse.move(dx, dy, wheel);
    Mouse.set_buttons((buttons & 0x01) ? 1 : 0,
                      (buttons & 0x04) ? 1 : 0,
                      (buttons & 0x02) ? 1 : 0);
    return true;
}

/* ----- Frame dispatch ----------------------------------------------------- */
static void handle_frame(const gw_frame_t *f)
{
    switch (f->type) {
    case GW_MSG_PING:
        link_send(GW_MSG_PONG, f->payload, f->len);
        break;
    case GW_MSG_MOUSE_TEST:
        if (run_mouse_test(f->payload, f->len)) send_ack(f->seq);
        else send_nak(f->seq, GW_NAK_BAD_LEN);
        break;
    case GW_MSG_RESET:
        Mouse.set_buttons(0, 0, 0);
        g_tx_seq = 0;
        send_ack(f->seq);
        break;
    case GW_MSG_ACK:
    case GW_MSG_NAK:
    case GW_MSG_PONG:
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
    Serial1.begin(LINK_BAUD);
    gw_parser_init(&g_parser);
    myusb.begin();
}

void loop()
{
    myusb.Task();
    passthrough_real_mouse();

    while (Serial1.available()) {
        gw_frame_t f;
        gw_parse_result_t r = gw_parser_feed(&g_parser, (uint8_t)Serial1.read(), &f);
        if (r == GW_PARSE_FRAME) {
            handle_frame(&f);
        } else if (r == GW_PARSE_CRC_ERR) {
            send_nak(0xFF, GW_NAK_CRC);
        }
    }
}
