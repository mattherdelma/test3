/*
 * test_protocol.c - Host-side self-test for the gateway protocol.
 * Build: cc -I../protocol test_protocol.c ../protocol/crc.c ../protocol/gateway_protocol.c -o test_protocol
 * Verifies CRC vectors, frame build/parse round-trip, resync, and CRC error.
 */
#include <stdio.h>
#include <string.h>
#include "gateway_protocol.h"

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
    else { printf("ok  : %s\n", msg); } } while (0)

int main(void)
{
    /* 1. Known CRC vector: CRC-16/CCITT-FALSE("123456789") == 0x29B1 */
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1,
          "CRC-16/CCITT-FALSE check value 0x29B1");

    /* 2. Build + parse round-trip of a KEY_SEQUENCE frame */
    uint8_t payload[] = { 0x01, GW_KEY_TAP, 't', GW_MOD_LCTRL | GW_MOD_LALT, 0x00, 0x00 };
    uint8_t frame[GW_MAX_FRAME];
    size_t n = gw_build_frame(frame, GW_MSG_KEY_SEQUENCE, 42, payload, sizeof(payload));
    CHECK(n == GW_HEADER_LEN + sizeof(payload) + GW_CRC_LEN, "frame length correct");

    gw_parser_t p;
    gw_parser_init(&p);
    gw_frame_t out;
    gw_parse_result_t r = GW_PARSE_IDLE;
    for (size_t i = 0; i < n; i++) {
        r = gw_parser_feed(&p, frame[i], &out);
    }
    CHECK(r == GW_PARSE_FRAME, "round-trip yields a frame");
    CHECK(out.type == GW_MSG_KEY_SEQUENCE && out.seq == 42 &&
          out.len == sizeof(payload) &&
          memcmp(out.payload, payload, sizeof(payload)) == 0,
          "round-trip fields match");

    /* 3. Resync: garbage before a valid frame is skipped */
    gw_parser_init(&p);
    uint8_t junk[] = { 0x00, 0xFF, 0x55, 0xAA /* false SOF */, 0x99 /* bad ver */ };
    for (size_t i = 0; i < sizeof(junk); i++) gw_parser_feed(&p, junk[i], &out);
    r = GW_PARSE_IDLE;
    for (size_t i = 0; i < n; i++) r = gw_parser_feed(&p, frame[i], &out);
    CHECK(r == GW_PARSE_FRAME, "resync after junk recovers next valid frame");

    /* 4. CRC error detection: corrupt one payload byte */
    frame[GW_HEADER_LEN] ^= 0xFF;
    gw_parser_init(&p);
    r = GW_PARSE_IDLE;
    for (size_t i = 0; i < n; i++) r = gw_parser_feed(&p, frame[i], &out);
    CHECK(r == GW_PARSE_CRC_ERR, "corrupted frame reported as CRC error");

    /* 5. Zero-length payload frame (e.g. RESET) */
    n = gw_build_frame(frame, GW_MSG_RESET, 7, NULL, 0);
    gw_parser_init(&p);
    r = GW_PARSE_IDLE;
    for (size_t i = 0; i < n; i++) r = gw_parser_feed(&p, frame[i], &out);
    CHECK(r == GW_PARSE_FRAME && out.type == GW_MSG_RESET && out.len == 0,
          "zero-length frame round-trips");

    printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
