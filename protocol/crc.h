/*
 * crc.h - CRC-16/CCITT-FALSE implementation for the UART gateway protocol.
 *
 * Portable plain C, no dynamic allocation. Compiles on both the Teensy 4.1
 * (Teensyduino) and the Arduino Leonardo (AVR) toolchains.
 *
 * Polynomial : 0x1021
 * Init       : 0xFFFF
 * RefIn/Out  : false
 * XorOut     : 0x0000
 */
#ifndef GATEWAY_CRC_H
#define GATEWAY_CRC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Compute CRC-16/CCITT-FALSE over a buffer. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* Incremental variant: feed bytes one at a time. Seed with 0xFFFF. */
uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_CRC_H */
