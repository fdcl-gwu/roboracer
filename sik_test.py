#!/usr/bin/env python3
"""
sik_test.py — SiK radio link verification for Windows (COM3).

Sends zero-throttle / centered-steering packets at 40 Hz and prints any bytes
the car firmware echoes back.  The car will NOT move — throttle is always 0.

Usage:
    python sik_test.py [--port /dev/ttyUSB0] [--count N]
    python sik_test.py --scan            # probe baud rates via AT commands

    --port   Serial port (default: /dev/ttyUSB0)
    --count  Number of packets to send; 0 = run until Ctrl-C (default: 40)
    --scan   Detect baud rate using SiK AT command mode, then exit
"""

import argparse
import struct
import time

import serial
import serial.tools.list_ports


# ── Protocol constants (must match firmware) ──────────────────────────────────
STX          = 0xFE
PAYLOAD_LEN  = 0x04
BAUD_RATE    = 230400
STEER_CENTER = 512

# Common SiK radio serial baud rates to probe during --scan
SCAN_BAUDS = [57600, 115200, 9600, 19200, 38400, 230400]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(seq: int, throttle: int, steering: int) -> bytes:
    seq      &= 0xFF
    throttle  = max(0,   min(throttle, 2047))
    steering  = max(0,   min(steering, 1023))
    payload   = struct.pack('<BBHh', PAYLOAD_LEN, seq, throttle, steering)
    crc       = crc16_ccitt(payload)
    return bytes([STX]) + payload + struct.pack('<H', crc)


def list_ports() -> None:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (no serial ports found)")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")


def scan_baud(port: str) -> None:
    """
    Probe common baud rates using the SiK radio's AT command mode.

    Protocol:
      1. Open port at candidate baud rate.
      2. Wait 1 s (radio guard time).
      3. Send '+++' with no newline.
      4. Wait 1 s.
      5. If the radio responds 'OK', we found the right baud rate.
      6. Send 'ATI5\\r\\n' to dump radio parameters, then 'ATO\\r\\n' to exit
         command mode and return the radio to pass-through.
    """
    print(f"Scanning {port} for SiK radio baud rate...")
    for baud in SCAN_BAUDS:
        print(f"  Trying {baud:7d} baud ... ", end="", flush=True)
        try:
            ser = serial.Serial(port, baud, timeout=1.0)
        except serial.SerialException as e:
            print(f"open failed: {e}")
            continue

        ser.reset_input_buffer()
        time.sleep(1.0)          # guard time before +++
        ser.write(b"+++")
        time.sleep(1.0)          # wait for OK
        resp = ser.read(64).decode("ascii", errors="replace").strip()

        if "OK" in resp:
            print(f"FOUND  (response: {resp!r})")
            # Dump radio config
            ser.write(b"ATI5\r\n")
            time.sleep(0.3)
            config = ser.read(512).decode("ascii", errors="replace").strip()
            if config:
                print("\n  Radio parameters (ATI5):")
                for line in config.splitlines():
                    print(f"    {line}")
            # Return radio to pass-through mode
            ser.write(b"ATO\r\n")
            time.sleep(0.1)
            ser.close()
            print(f"\nSet --baud {baud} (or update BAUD_RATE in the racing scripts).")
            return
        else:
            print(f"no response ({resp!r})")
            ser.close()

    print("\nNo baud rate matched.  Check that the radio is powered and on the correct port.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SiK radio link test — sends zero-throttle packets, does not move the car.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--count", type=int, default=40,
                        help="Packets to send; 0 = run until Ctrl-C")
    parser.add_argument("--scan",  action="store_true",
                        help="Detect baud rate via SiK AT commands, then exit")
    args = parser.parse_args()

    print("Available serial ports:")
    list_ports()
    print()

    if args.scan:
        scan_baud(args.port)
        return

    print(f"Opening {args.port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.05)
    except serial.SerialException as e:
        print(f"ERROR: could not open {args.port}: {e}")
        return

    time.sleep(0.1)
    ser.reset_input_buffer()
    print(f"Port open. Sending {'unlimited' if args.count == 0 else args.count} "
          f"zero-throttle packets at 40 Hz.  Press Ctrl-C to stop.\n")

    seq        = 0
    sent       = 0
    rx_total   = 0
    SAFE_PKT   = build_packet(0, 0, STEER_CENTER)  # preview only
    print(f"Packet format (seq=0): {SAFE_PKT.hex(' ').upper()}")
    print(f"  STX={STX:#04x}  payload_len={PAYLOAD_LEN}  throttle=0  steering={STEER_CENTER}\n")

    try:
        while args.count == 0 or sent < args.count:
            pkt = build_packet(seq, 0, STEER_CENTER)
            ser.write(pkt)

            # Drain any response bytes (firmware may echo status)
            rx = ser.read(64)
            rx_total += len(rx)
            rx_str = rx.hex(' ').upper() if rx else "(none)"

            print(f"[{sent:4d}] TX seq={seq:3d}  {pkt.hex(' ').upper()}"
                  f"  RX: {rx_str}")

            seq = (seq + 1) & 0xFF
            sent += 1
            time.sleep(0.025)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.reset_input_buffer()
        ser.close()

    print(f"\nSummary: {sent} packets sent, {rx_total} bytes received.")
    if rx_total > 0:
        print("Radio link appears active — bytes are coming back from the car.")
    else:
        print("No bytes received.  This is normal if the car firmware does not echo;")
        print("the absence of a TX error means the radio link is up on the laptop side.")


if __name__ == "__main__":
    main()
