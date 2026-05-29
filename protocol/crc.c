/*
 * crc.c - CRC-16/CCITT-FALSE implementation (bitwise, table-free).
 *
 * The bitwise form is chosen so it stays tiny on the Leonardo's 2.5 KB SRAM
 * budget. At the protocol's frame sizes (<= 260 bytes) the cost is negligible.
 */
#include "crc.h"

uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (uint16_t)((crc << 1) ^ 0x1021);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = crc16_ccitt_update(crc, data[i]);
    }
    return crc;
}
