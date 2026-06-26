#!/usr/bin/env python3
"""
WiCAN BLE application-layer auth test client.

Validates the challenge-response gate implemented in main/ble.c without needing a full client app:
  1. Connect to the WiCAN dongle (open GATT, no pairing).
  2. Subscribe to the data (FEE1) and auth (FEE2) characteristics.
  3. LOCKED CHECK (pre-auth): probe FEE1 and confirm NO telemetry flows before authenticating.
  4. Read the 16-byte per-connection nonce from FEE2.
  5. Send response = HMAC-SHA256(key, b"WICAN-BLE-AUTH-v1" + nonce) to FEE2.
  6. Expect a 0x01 (OK) notification.
  7. UNLOCKED CHECK (post-auth): confirm telemetry now flows (needs live CAN bus traffic).
  8. Optionally send a CAN frame to FEE1 to confirm the data path is unlocked.

The shared key is the 128-bit device key (see the web UI / config_server_get_ble_key_hex),
passed as 32 hex chars.

Usage:
    pip install bleak
    python ble_auth_test.py --key <32-hex-chars> [--name WiC_xxxx] [--send-slcan t1234...] [--window 2.0]

Exit codes:
    0 = auth succeeded AND the data path was locked before auth (gate works)
    1 = auth failed (wrong key?)
    6 = gate LEAKED: telemetry received before auth
    2..5 = usage / connection / protocol errors
"""

import argparse
import asyncio
import hashlib
import hmac
import sys

from bleak import BleakClient, BleakScanner

SVC_UUID = "0000fee0-0000-1000-8000-00805f9b34fb"
CHAR_DATA = "0000fee1-0000-1000-8000-00805f9b34fb"  # CAN data (write/notify), gated
CHAR_AUTH = "0000fee2-0000-1000-8000-00805f9b34fb"  # auth channel
AUTH_LABEL = b"WICAN-BLE-AUTH-v1"
NONCE_LEN = 16
RESP_LEN = 32


def compute_response(key: bytes, nonce: bytes) -> bytes:
    return hmac.new(key, AUTH_LABEL + nonce, hashlib.sha256).digest()


async def find_device(name: str | None):
    print("Scanning for WiCAN...")
    devices = await BleakScanner.discover(timeout=8.0, return_adv=True)
    for dev, adv in devices.values():
        local = adv.local_name or dev.name or ""
        if name:
            if local == name:
                return dev
        elif local.startswith("WiC_") or SVC_UUID in (adv.service_uuids or []):
            return dev
    return None


async def run(args) -> int:
    key = bytes.fromhex(args.key)
    if len(key) != 16:
        print(f"ERROR: key must be 16 bytes (32 hex chars), got {len(key)}")
        return 2

    dev = await find_device(args.name)
    if dev is None:
        print("ERROR: WiCAN not found. Is it powered and advertising?")
        return 3
    print(f"Found {dev.address} ({dev.name}); connecting...")

    auth_result: asyncio.Future = asyncio.get_event_loop().create_future()
    data_count = 0

    async with BleakClient(dev) as client:
        print("Connected.")

        def on_auth_notify(_char, data: bytearray):
            ok = len(data) == 1 and data[0] == 0x01
            if not auth_result.done():
                auth_result.set_result(ok)

        def on_data_notify(_char, data: bytearray):
            nonlocal data_count
            data_count += 1

        await client.start_notify(CHAR_AUTH, on_auth_notify)
        # Subscribe to the gated data characteristic so we can observe whether telemetry leaks.
        await client.start_notify(CHAR_DATA, on_data_notify)

        # ---- LOCKED CHECK (pre-auth) ----
        # The firmware must drop FEE1 writes and withhold FEE1 telemetry until authenticated.
        # Probe with a write (should be silently dropped), then watch for any telemetry.
        print(f"[locked-check] Probing data path before auth (window {args.window:.1f}s)...")
        try:
            await client.write_gatt_char(CHAR_DATA, b"t1002AABB\r", response=False)
        except Exception as e:  # noqa: BLE001 - informational only
            print(f"[locked-check] probe write raised (non-fatal): {e}")
        data_count = 0
        await asyncio.sleep(args.window)
        pre_auth_count = data_count
        if pre_auth_count == 0:
            print("[locked-check] PASS: no telemetry received before auth (data path locked).")
        else:
            print(f"[locked-check] FAIL: received {pre_auth_count} telemetry notifications "
                  "BEFORE auth — gate is leaking!")

        # ---- AUTH ----
        nonce = bytes(await client.read_gatt_char(CHAR_AUTH))
        print(f"Nonce ({len(nonce)} B): {nonce.hex()}")
        if len(nonce) != NONCE_LEN:
            print(f"ERROR: expected {NONCE_LEN}-byte nonce, got {len(nonce)}")
            return 4

        resp = compute_response(key, nonce)
        print(f"Response: {resp.hex()}")
        await client.write_gatt_char(CHAR_AUTH, resp, response=True)

        try:
            ok = await asyncio.wait_for(auth_result, timeout=5.0)
        except asyncio.TimeoutError:
            print("ERROR: no auth result notification within 5s")
            return 5

        if not ok:
            print("AUTH FAILED (wrong key?)")
            return 1
        print("AUTH OK — CAN data path unlocked.")

        if args.send_slcan:
            payload = args.send_slcan.encode() + b"\r"
            await client.write_gatt_char(CHAR_DATA, payload, response=False)
            print(f"Sent SLCAN frame: {args.send_slcan}")

        # ---- UNLOCKED CHECK (post-auth) ----
        data_count = 0
        await asyncio.sleep(args.window)
        post_auth_count = data_count
        if post_auth_count > 0:
            print(f"[unlocked-check] PASS: {post_auth_count} telemetry notifications after auth.")
        else:
            print("[unlocked-check] inconclusive: no telemetry after auth "
                  "(no live CAN bus traffic?). Auth OK already confirms the gate opened.")

        # Verdict: a pre-auth leak is a hard failure even if auth later succeeded.
        if pre_auth_count != 0:
            return 6
        return 0


def main() -> int:
    p = argparse.ArgumentParser(description="WiCAN BLE auth test client")
    p.add_argument("--key", required=True, help="128-bit device key as 32 hex chars")
    p.add_argument("--name", help="exact BLE local name (e.g. WiC_7c2c67b37ce5)")
    p.add_argument("--send-slcan", help="optional SLCAN frame to send after auth, e.g. t1002AABB")
    p.add_argument("--window", type=float, default=2.0,
                   help="seconds to observe telemetry for the locked/unlocked checks (default 2.0)")
    args = p.parse_args()
    return asyncio.run(run(args))


if __name__ == "__main__":
    sys.exit(main())
