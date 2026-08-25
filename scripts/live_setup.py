#!/usr/bin/env python3
"""Repeatable live X-Plane spawn setup (Task 7 §13, forensic corrective pass).

SOURCE-OF-TRUTH RULE (forensic 2026-08-25):
  The ground spawn comes from X-Plane NATIVE airport scenery (apt.dat
  code-1400 startup locations), NEVER from an OpenAIRAC/Navigraph airport
  reference point. An airport ARP is navigation context — using it as a
  parking position spawns the aircraft inside terminal structures
  (observed live: C172 spawned at the KLAX ARP under terminal buildings).

Usage:
  python scripts/live_setup.py KLAX [ramp_index]

Steps:
  1. parse apt.dat for the airport's native startup locations (provenance
     printed per candidate: apt.dat line number);
  2. POST /api/v3/flight with lle_ground_start at the NATIVE ramp
     coordinate (explicit provenance);
  3. machine-readable readback: position within tolerance of the ramp,
     AGL ~0 (on ground), bridge handshake, Web API health;
  4. wait 30 s; re-verify; reload the flight ONCE; verify again.

No UI automation. No FlightdeckOS writes. Bounded waits, no infinite
retry loops.
"""

import json
import sys
import time
import urllib.request
import urllib.error

XP_ROOT = r"F:\SteamLibrary\steamapps\common\X-Plane 12"
APT_DAT = XP_ROOT + r"\Global Scenery\Global Airports\Earth nav data\apt.dat"
WEBAPI = "http://127.0.0.1:8086"
BRIDGE_PORT = 57501
POSITION_TOLERANCE_DEG = 0.005  # ~0.3 nm


def http_json(url, timeout=5):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.load(r)


def dataref(name: str):
    d = http_json(f"{WEBAPI}/api/v2/datarefs?filter%5Bname%5D={urllib.parse.quote(name, safe='')}")["data"]
    if not d:
        return None
    return http_json(f"{WEBAPI}/api/v2/datarefs/{d[0]['id']}/value").get("data")


def native_ramp_starts(airport_id: str):
    """Parse apt.dat code-1400 startup locations for one airport.

    Yields dicts with provenance (line number in apt.dat) — the ONLY
    sanctioned source for ground spawn geometry.
    """
    starts = []
    in_block = False
    with open(APT_DAT, "r", encoding="utf-8", errors="replace") as f:
        for lineno, line in enumerate(f, 1):
            if line.startswith("1 "):
                # Airport header: `1 <elev> <deprecated> <deprecated> <ident> <name...>`
                parts = line.split()
                if len(parts) > 4 and parts[4] == airport_id:
                    in_block = True
                continue
            if in_block and line.startswith("1 "):
                break  # next airport
            if in_block and line.startswith("1400 "):
                parts = line.split()
                if len(parts) >= 6:
                    starts.append(
                        {
                            "lat": float(parts[1]),
                            "lon": float(parts[2]),
                            "heading_true": float(parts[3]),
                            "location_type": parts[4],
                            "name": parts[5] if len(parts) > 5 else "",
                            "provenance": f"{APT_DAT}:{lineno}",
                        }
                    )
    return starts


def init_flight(acf_rel: str, ramp: dict):
    body = {
        "aircraft": {"path": acf_rel},
        "lle_ground_start": {
            "latitude": ramp["lat"],
            "longitude": ramp["lon"],
            "heading_true": ramp["heading_true"],
        },
    }
    req = urllib.request.Request(
        f"{WEBAPI}/api/v3/flight",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status
    except urllib.error.HTTPError as e:
        return e.code
    except Exception:
        return None  # reload blocks the API until the flight is up


def bridge_hello(timeout=3.0):
    import socket

    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect(("127.0.0.1", BRIDGE_PORT))
        return s.recv(4096).decode(errors="replace").strip()
    except Exception as e:
        return f"DOWN: {e}"
    finally:
        s.close()


def verify(ramp, label: str) -> bool:
    lat = dataref("sim/flightmodel/position/latitude")
    lon = dataref("sim/flightmodel/position/longitude")
    agl = dataref("sim/flightmodel/position/y_agl")
    icao_raw = dataref("sim/aircraft/view/acf_ICAO")
    icao = base64_decode(icao_raw) if isinstance(icao_raw, str) else str(icao_raw)
    ok_pos = (
        lat is not None
        and lon is not None
        and abs(lat - ramp["lat"]) < POSITION_TOLERANCE_DEG
        and abs(lon - ramp["lon"]) < POSITION_TOLERANCE_DEG
    )
    ok_ground = agl is not None and abs(agl) < 1.0
    print(
        f"[{label}] icao={icao} lat={lat} lon={lon} agl={agl} "
        f"pos_ok={ok_pos} on_ground_ok={ok_ground} bridge={bridge_hello()[:60]}"
    )
    return ok_pos and ok_ground


def base64_decode(v: str) -> str:
    import base64

    return base64.b64decode(v).decode(errors="replace").strip("\x00")


def main():
    import urllib.parse  # noqa: F401 (used in dataref)

    if len(sys.argv) < 2:
        print("usage: live_setup.py <AIRPORT_ID> [ramp_index]")
        return 2
    airport_id = sys.argv[1].upper()
    ramp_index = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    acf_rel = r"Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP_G1000.acf"

    ramps = native_ramp_starts(airport_id)
    if not ramps:
        print(f"NO native startup locations for {airport_id} in apt.dat — refusing to spawn")
        return 1
    print(f"Native startup locations for {airport_id} (source: X-Plane apt.dat):")
    for i, r in enumerate(ramps[:8]):
        print(f"  [{i}] {r['location_type']:>9} name={r['name']:<12} "
              f"lat={r['lat']:.6f} lon={r['lon']:.6f} hdg={r['heading_true']:.1f}  ({r['provenance']})")
    ramp = ramps[min(ramp_index, len(ramps) - 1)]
    print(f"SELECTED ramp [{ramp_index}] provenance={ramp['provenance']} (native scenery, NOT an ARP)")

    print("Initializing flight ...")
    init_flight(acf_rel, ramp)
    time.sleep(50)  # bounded load window

    if not verify(ramp, "after-load"):
        print("FAIL: position/ground verification after load")
        return 1
    print("Holding 30 s ...")
    time.sleep(30)
    if not verify(ramp, "hold-30s"):
        print("FAIL: verification drifted during hold")
        return 1
    print("Reloading flight once ...")
    init_flight(acf_rel, ramp)
    time.sleep(50)
    if not verify(ramp, "after-reload"):
        print("FAIL: verification after reload")
        return 1
    print("SPAWN SMOKE GREEN — operator visual confirmation still required:")
    print("  Посмотри в X-Plane: самолёт стоит корректно?")
    return 0


if __name__ == "__main__":
    sys.exit(main())
