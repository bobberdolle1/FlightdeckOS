#!/usr/bin/env python3
"""Bounded one-shot UDP probe for X-Plane BECN/RREF diagnosis (Task 7.1 A/B).

Sends one BECN and one bounded RREF subscription, waits up to TIMEOUT
seconds, prints exact endpoints and framing verdict, exits.
Diagnostic tool only - not production telemetry.
"""
import socket
import struct
import sys

HOST = "127.0.0.1"
PORT = 49000
TIMEOUT = 8  # seconds, bounded per Task 7.1 section 8


def main() -> int:
    results = []
    # --- BECN probe ---
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(TIMEOUT)
    try:
        s.sendto(b"BECN\x00", (HOST, PORT))
        data, addr = s.recvfrom(2048)
        ok = len(data) >= 5 + 16 and data[:5] == b"BECN\x00"
        results.append(
            f"BECN REPLY from {addr[0]}:{addr[1]} -> local {s.getsockname()[0]}:{s.getsockname()[1]} "
            f"bytes={len(data)} framing={'OK' if ok else 'MALFORMED'} beacon_port={struct.unpack_from('>H', data, 5)[0] if len(data) >= 7 else '?'}"
        )
    except socket.timeout:
        results.append(f"BECN no reply within {TIMEOUT}s (sent to {HOST}:{PORT})")
    finally:
        s.close()

    # --- RREF probe ---
    local = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local.bind((HOST, 0))
    local.settimeout(TIMEOUT)
    cmd = (
        b"RREF\x00"
        + struct.pack("<I", 10)  # frequency Hz
        + struct.pack("<I", 1)  # one dataref
        + b"sim/flightmodel/position/latitude".ljust(400, b"\x00")
    )
    try:
        local.sendto(cmd, (HOST, PORT))
        data, addr = local.recvfrom(2048)
        ok = len(data) >= 9 and data[:4] == b"RREF"
        lat = struct.unpack_from("<f", data, 9)[0] if len(data) >= 13 else None
        results.append(
            f"RREF REPLY from {addr[0]}:{addr[1]} -> local {local.getsockname()[0]}:{local.getsockname()[1]} "
            f"bytes={len(data)} framing={'OK' if ok else 'MALFORMED'} lat={lat}"
        )
    except socket.timeout:
        results.append(f"RREF no reply within {TIMEOUT}s (subscribe sent to {HOST}:{PORT})")
    finally:
        local.close()

    for r in results:
        print(r)
    got = sum(1 for r in results if "REPLY" in r)
    print(f"UDP_PROBE: {'PASS' if got == 2 else 'FAIL'} ({got}/2)")
    return 0 if got == 2 else 1


if __name__ == "__main__":
    sys.exit(main())
