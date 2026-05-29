#!/usr/bin/env python3
"""
gateway_host.py - PC-side reference implementation of the UART gateway protocol.

Builds/parses frames identically to protocol/*.c so you can drive the keyboard
gateway (timed key sequences) and the mouse test fixture, and receive
accessibility button events.

Compliant scope: this sends documented automation scripts and reads device
notifications. It does not implement any "blend injected motion into live hand
input" behaviour.

Usage example:
    python3 gateway_host.py --port /dev/ttyUSB0 --demo
"""
import argparse
import struct
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    serial = None

SOF = 0xAA
VERSION = 0x01

# Message types (mirror gateway_protocol.h)
MSG_PING = 0x01
MSG_PONG = 0x02
MSG_ACK = 0x03
MSG_NAK = 0x04
MSG_STATUS = 0x05
MSG_BUTTON_EVENT = 0x10
MSG_KEY_SEQUENCE = 0x20
MSG_MOUSE_TEST = 0x21
MSG_RESET = 0x7E

# Key actions / modifiers
ACT_PRESS, ACT_RELEASE, ACT_TAP, ACT_RELALL = 1, 2, 3, 4
MOD_LCTRL, MOD_LSHIFT, MOD_LALT, MOD_LGUI = 0x01, 0x02, 0x04, 0x08
MOD_RCTRL, MOD_RSHIFT, MOD_RALT, MOD_RGUI = 0x10, 0x20, 0x40, 0x80


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, matching protocol/crc.c."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too long")
    body = bytes([VERSION, msg_type, seq & 0xFF, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([SOF]) + body + struct.pack(">H", crc)


def key_record(action: int, keycode: int = 0, mods: int = 0, delay_us: int = 0) -> bytes:
    return struct.pack("<BBBH", action, keycode, mods, delay_us & 0xFFFF)


def key_sequence_payload(records: list[bytes]) -> bytes:
    if len(records) > 50:
        raise ValueError("max 50 records per frame")
    return bytes([len(records)]) + b"".join(records)


def mouse_test_payload(dx: int, dy: int, wheel: int = 0, buttons: int = 0) -> bytes:
    return struct.pack("<hhbB", dx, dy, wheel, buttons)


class FrameParser:
    """Incremental parser mirroring gateway_protocol.c (returns dicts)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        frames = []
        for b in data:
            self.buf.append(b)
            self._try(frames)
        return frames

    def _try(self, frames):
        # Resync: drop until SOF leads the buffer.
        while self.buf and self.buf[0] != SOF:
            self.buf.pop(0)
        if len(self.buf) < 5:
            return
        length = self.buf[4]
        total = 5 + length + 2
        if len(self.buf) < total:
            return
        body = bytes(self.buf[1:5 + length])
        rx_crc = struct.unpack(">H", bytes(self.buf[5 + length:total]))[0]
        if crc16_ccitt(body) == rx_crc and self.buf[1] == VERSION:
            frames.append({
                "type": self.buf[2], "seq": self.buf[3],
                "payload": bytes(self.buf[5:5 + length]),
            })
            del self.buf[:total]
        else:
            # Bad frame: drop the leading SOF and resync.
            self.buf.pop(0)


class Gateway:
    def __init__(self, port, baud=115200, timeout=0.05):
        if serial is None:
            raise RuntimeError("pyserial not installed: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.parser = FrameParser()
        self.seq = 0

    def _next_seq(self):
        s = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return s

    def send(self, msg_type, payload=b"", expect_ack=True, retries=4):
        seq = self._next_seq()
        frame = build_frame(msg_type, seq, payload)
        delay = 0.05
        for attempt in range(retries + 1):
            self.ser.write(frame)
            if not expect_ack:
                return True
            deadline = time.time() + 0.2
            while time.time() < deadline:
                for f in self.parser.feed(self.ser.read(64)):
                    if f["type"] == MSG_ACK and f["payload"] and f["payload"][0] == seq:
                        return True
                    if f["type"] == MSG_NAK and len(f["payload"]) >= 1 and f["payload"][0] == seq:
                        break
            time.sleep(delay)
            delay *= 2  # exponential backoff
        return False

    def poll_events(self):
        return self.parser.feed(self.ser.read(128))


def demo(gw: Gateway):
    print("PING ->", "ok" if gw.send(MSG_PING, b"\x01\x02") else "no reply")

    # Accessibility shortcut: Ctrl+Alt+T as a timed TAP sequence.
    seq = key_sequence_payload([
        key_record(ACT_TAP, ord('t'), MOD_LCTRL | MOD_LALT, delay_us=0),
    ])
    print("KEY_SEQUENCE ->", "ack" if gw.send(MSG_KEY_SEQUENCE, seq) else "fail")

    # Test fixture: nudge cursor right by 10px.
    print("MOUSE_TEST ->", "ack" if gw.send(MSG_MOUSE_TEST, mouse_test_payload(10, 0)) else "fail")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--listen", action="store_true", help="print button events")
    args = ap.parse_args()

    gw = Gateway(args.port, args.baud)
    if args.demo:
        demo(gw)
    if args.listen:
        print("Listening for events (Ctrl-C to stop)...")
        try:
            while True:
                for f in gw.poll_events():
                    if f["type"] == MSG_BUTTON_EVENT:
                        mask, state = f["payload"][0], f["payload"][1]
                        print(f"button mask=0x{mask:02x} state={state}")
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    sys.exit(main())
