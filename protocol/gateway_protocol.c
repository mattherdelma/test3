/*
 * gateway_protocol.c - Implementation of the incremental frame parser and the
 * frame builder declared in gateway_protocol.h.
 */
#include "gateway_protocol.h"

void gw_parser_init(gw_parser_t *p)
{
    p->state    = GW_WAIT_SOF;
    p->type     = 0;
    p->seq      = 0;
    p->len      = 0;
    p->idx      = 0;
    p->crc_calc = 0xFFFF;
    p->crc_rx   = 0;
}

gw_parse_result_t gw_parser_feed(gw_parser_t *p, uint8_t b, gw_frame_t *out)
{
    switch (p->state) {
    case GW_WAIT_SOF:
        if (b == GW_SOF) {
            p->crc_calc = 0xFFFF;
            p->state = GW_WAIT_VER;
        }
        /* else: stay hunting for SOF (resync) */
        break;

    case GW_WAIT_VER:
        if (b != GW_PROTO_VERSION) {
            /* Unexpected version: treat this byte as a possible new SOF. */
            p->state = (b == GW_SOF) ? GW_WAIT_VER : GW_WAIT_SOF;
            if (b == GW_SOF) { p->crc_calc = 0xFFFF; }
            break;
        }
        p->crc_calc = crc16_ccitt_update(p->crc_calc, b);
        p->state = GW_WAIT_TYPE;
        break;

    case GW_WAIT_TYPE:
        p->type = b;
        p->crc_calc = crc16_ccitt_update(p->crc_calc, b);
        p->state = GW_WAIT_SEQ;
        break;

    case GW_WAIT_SEQ:
        p->seq = b;
        p->crc_calc = crc16_ccitt_update(p->crc_calc, b);
        p->state = GW_WAIT_LEN;
        break;

    case GW_WAIT_LEN:
        p->len = b;
        p->idx = 0;
        p->crc_calc = crc16_ccitt_update(p->crc_calc, b);
        p->state = (p->len == 0) ? GW_WAIT_CRC_HI : GW_WAIT_PAYLOAD;
        break;

    case GW_WAIT_PAYLOAD:
        p->payload[p->idx++] = b;
        p->crc_calc = crc16_ccitt_update(p->crc_calc, b);
        if (p->idx >= p->len) {
            p->state = GW_WAIT_CRC_HI;
        }
        break;

    case GW_WAIT_CRC_HI:
        p->crc_rx = (uint16_t)b << 8;
        p->state = GW_WAIT_CRC_LO;
        break;

    case GW_WAIT_CRC_LO:
        p->crc_rx |= b;
        p->state = GW_WAIT_SOF; /* frame complete either way */
        if (p->crc_rx == p->crc_calc) {
            out->type    = p->type;
            out->seq     = p->seq;
            out->len     = p->len;
            out->payload = p->payload;
            return GW_PARSE_FRAME;
        }
        return GW_PARSE_CRC_ERR;
    }

    return GW_PARSE_IDLE;
}

size_t gw_build_frame(uint8_t *buf, uint8_t type, uint8_t seq,
                      const uint8_t *payload, uint8_t payload_len)
{
    /* payload_len is uint8_t, so it is inherently <= GW_MAX_PAYLOAD (255).
     * The bound is enforced by the type; no runtime check is needed here. */
    size_t n = 0;
    buf[n++] = GW_SOF;
    buf[n++] = GW_PROTO_VERSION;
    buf[n++] = type;
    buf[n++] = seq;
    buf[n++] = payload_len;

    uint16_t crc = 0xFFFF;
    crc = crc16_ccitt_update(crc, GW_PROTO_VERSION);
    crc = crc16_ccitt_update(crc, type);
    crc = crc16_ccitt_update(crc, seq);
    crc = crc16_ccitt_update(crc, payload_len);

    for (uint8_t i = 0; i < payload_len; i++) {
        buf[n++] = payload[i];
        crc = crc16_ccitt_update(crc, payload[i]);
    }

    buf[n++] = (uint8_t)(crc >> 8);
    buf[n++] = (uint8_t)(crc & 0xFF);
    return n;
}
