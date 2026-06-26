# Local development notes

Notes for building and deploying this repository. Not upstream documentation — see [README.md](README.md) and [Firmware Update docs](https://meatpihq.github.io/wican-fw/config/firmware-update/) for general WiCAN info.

## Pick the right hardware target first

The hardware variant is selected in `CMakeLists.txt` (not `sdkconfig`). **Build the wrong one and OTA will still flash, but pin mappings and behaviour will be wrong.**

| CMakeLists setting | Hardware ver in About | Binary prefix |
|---|---|---|
| `WICAN_V210` | WiCAN-OBD (v2.10) | `wican-fw_obd_*` |
| `WICAN_V300` | WiCAN-OBD | `wican-fw_obd_*` |
| `WICAN_USB_V100` | WiCAN-USB | `wican-fw_usb_*` |
| `WICAN_PRO` | WiCAN-OBD-PRO | `wican-fw_obd_pro_*` |

Default build target in `CMakeLists.txt`: **`WICAN_USB_V100`** (WiCAN-USB). Change to match your hardware before building.

Check the device first: **About → Hardware ver** (e.g. `WiCAN-USB`, Git ver `v4.21u`).

## ESP-IDF setup

- Target chip: **ESP32-C3** (`CONFIG_IDF_TARGET="esp32c3"` in `sdkconfig`)
- Use **ESP-IDF v5.5.x** to match `sdkconfig` (project was generated with 5.5.2)
- Install path used locally: `~/esp/esp-idf`

```bash
# One-time setup (if esp-idf is missing)
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.2 --recursive --depth 1 https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32c3

# Every new shell
source ~/esp/esp-idf/export.sh
```

## Build

Run from the repo root (where `CMakeLists.txt` lives):

```bash
source ~/esp/esp-idf/export.sh
cd /path/to/wican-fw   # adjust to your clone path

idf.py build
```

After changing `HARDWARE_VER` in `CMakeLists.txt`, wipe the build dir first — `idf.py fullclean` only works on a valid CMake build tree:

```bash
rm -rf build && idf.py build
```

Output: `build/wican-fw_<variant>_<git-sha>.bin`

- Git SHA comes from `git describe --tags --always --dirty`
- Uncommitted changes add a `-dirty` suffix to the binary name

### Troubleshooting

| Symptom | Fix |
|---|---|
| `cd: no such file or directory: /path/to/wican-fw` | `/path/to/wican-fw` is a placeholder — use your actual clone path |
| `fullclean` refuses to delete `build/` | `build/` is broken or incomplete — run `rm -rf build` then `idf.py build` |
| `idf.py: command not found` | Run `source ~/esp/esp-idf/export.sh` in the current shell first |

## Deploy

### OTA (easiest when the device is running)

1. Connect to the device network (AP mode is often `192.168.80.1`, or use station IP on your LAN).
2. Upload via web UI: **System → Firmware Update**, or:

```bash
curl -F "file=@build/wican-fw_usb_6735d62-dirty.bin" http://192.168.80.1/upload/ota.bin
```

Replace IP and filename as needed. The upload endpoint expects **multipart form** (`-F "file=@..."`).

**Gotcha:** your dev machine must be on the same network as the device. If the Mac is on home Wi‑Fi (`192.168.1.x`) and the WiCAN AP is `192.168.80.1`, curl will time out — connect to the WiCAN AP first, or use the station IP.

Update takes ~60 seconds; device reboots automatically. Verify on **About** tab.

### USB flash (when OTA is unavailable or for recovery)

1. Put device in **download mode** (short flash pins per hardware docs, plug USB, orange LED on).
2. Find port: `ls /dev/cu.usb*`
3. Flash:

```bash
source ~/esp/esp-idf/export.sh
idf.py -p /dev/cu.usbmodemXXXX flash
```

WiCAN-PRO exposes two COM ports — use **Serial-A**.

USB flash may erase NVS/config. Screenshot settings before flashing if needed.

## Verify after deploy

On the device **About** page, confirm:

- **Hardware ver** matches your target (e.g. `WiCAN-USB`)
- **Git ver** matches your commit (e.g. `fd18e06` or `fd18e06-dirty`)

## Notable changes from upstream

- **BLE pairing:** switched to **open GATT** (no encryption/pairing) to fix iOS/macOS bonding failures — see `main/ble.c` (`BLE_CHAR_PERM`). Both MITM passkey and Just Works pairing failed to complete against Apple CoreBluetooth (`ATT 0x0F` / apple-code 15 "Encryption is insufficient"), even from a clean slate. Also: FEE1/FEE3 advertise Write-Without-Response (`WRITE_NR`) for CAN TX, and advertising uses a stable public address (not RPA) for reliable reconnect.
- **BLE app-layer auth:** because BLE link security is not a usable boundary here (and never gated CAN access anyway), CAN access is gated by an application-layer challenge–response over an open link. See the section below.

## BLE application-layer auth (CAN access gate)

Open GATT means anyone in range could otherwise read/inject CAN. To prevent that, the BLE data path is gated by a shared-secret challenge–response: treat BLE as an untrusted pipe and authenticate at the application layer. Until the client authenticates, **no CAN frames are injected and no telemetry is delivered**.

### Key provisioning

- A random **128-bit device key** is generated on first use and stored at `FS_MOUNT_POINT/ble_key.bin` (see `config_server_get_ble_key` in `main/config_server.c`). It is independent of the JSON config.
- Retrieve/rotate it on the config web server (AP): open **`http://192.168.80.1/ble_key`** for a QR code and copy buttons, or `http://192.168.80.1/ble_key?regen=1` to regenerate.
- **No typing needed:** while connected to the dongle AP, the app can `GET http://192.168.80.1/ble_key.json` and read `key` / `pair_uri` automatically (works on phones and Android head units on the same Wi‑Fi).
- QR / setup link format: `wican://ble-key/<32-hex-chars>` (also shown on `/ble_key`).
- Manual fallback: use `key_grouped` from the JSON/page — 8 blocks of 4 hex chars (e.g. `a1b2-c3d4-e5f6-…`).

### Handshake protocol (characteristic `0xFEE2`, service `0xFEE0`)

1. On each connection the ESP generates a fresh 16-byte random **nonce**.
2. Client **reads `0xFEE2`** → 16-byte nonce.
3. Client **writes `0xFEE2`** ← `HMAC-SHA256(key, "WICAN-BLE-AUTH-v1" || nonce)` (32 bytes).
4. ESP verifies (constant-time) and **notifies `0xFEE2`** with 1 byte: `0x01` OK / `0x00` fail.
5. Only after OK do `0xFEE1` writes reach the CAN bus and does telemetry flow.
- Fresh nonce per connection defeats replay; the key never crosses the wire. Wrong response ×5 or no success within ~10 s → disconnect.
- **Scope:** session gate (unlocks the connection). Per-command counter+MAC and ECDH enrollment are out of scope for this firmware.

### Client app checklist

- Store the device key in secure storage; provision via QR/deep link (`wican://ble-key/…`), Wi‑Fi fetch (`GET /ble_key.json` on AP), or paste grouped key.
- On connect: subscribe to `0xFEE2` notify → read nonce → write `HMAC-SHA256(key, "WICAN-BLE-AUTH-v1"||nonce)` → wait for `0x01` before enabling RealDash/CAN TX.
- Remove the old "enter PIN when macOS prompts" pairing path (no longer used).

### Testing without the app

`tools/ble_auth_test.py` (needs `pip install bleak`) runs the full handshake:

```bash
python tools/ble_auth_test.py --key <32-hex-chars-from-/ble_key> [--name WiC_xxxx] [--send-slcan t1002AABB]
```

Exit code 0 = auth succeeded.

When syncing with upstream, re-check `CMakeLists.txt` hardware selection before building.
