# zxh-bms-zigbee

ESP32-C6 firmware that monitors several LiFePO4 "smart BMS" batteries
(zxhbms/uni-app board family, e.g. Chiconet rebrands) over **Bluetooth LE**
and reports them to Home Assistant over **Zigbee**, through
[zigbee2mqtt](https://www.zigbee2mqtt.io) — no WiFi needed at the battery
location.

Sibling projects (same wire protocol, different transport):

- [`Teejer/zxh-bms`](https://github.com/Teejer/zxh-bms) — Linux CLI + MQTT bridge
- [`Teejer/zxh-bms-esphome`](https://github.com/Teejer/zxh-bms-esphome) — ESPHome/WiFi component

## Why this exists

Home Assistant's Bluetooth *proxy* only forwards advertisements, and these
BMS boards publish no telemetry in advertisements — every reading must be
*requested* over a GATT connection. This firmware is that requester. It
runs on an ESP32-C6, whose second radio does **802.15.4/Zigbee**, so one
USB-powered puck near the batteries joins your Zigbee mesh and reports
through it. BLE and Zigbee coexist on the C6's shared 2.4 GHz front-end
(software coexistence); battery reads are short bursts, so neither stack
starves.

## How it works

- Zigbee **router** device (always on, also meshes) with **one endpoint per
  battery** (`pack_1`, `pack_2`, ...), each carrying a custom cluster
  (`0xFF10`) of plain attributes.
- BLE side: one connection at a time, round-robin. When a pack's
  `update_interval` elapses the scheduler connects to its MAC, auto-detects
  the GATT profile, re-discovers the Modbus bus address, runs the read
  cycle (instrument, cells, basic info, temps), disconnects, and publishes
  by sending explicit ZCL attribute reports to the coordinator.
- A pack that is off or unreachable is abandoned after `cycle_timeout` and
  retried in a minute — it never blocks the other packs. Its entities keep
  the last-known values and `online` goes false.
- Read-only by design: function codes 6/16 (parameter writes) are never
  sent, so protection settings cannot be corrupted.

## Hardware

Any ESP32-C6 module (4 MB flash). C6 specifically: it is the only ESP32
with both BLE and 802.15.4 that ESP-IDF's Zigbee stack supports alongside
BLE (the plain ESP32/S3 have BLE but no 802.15.4; C3/H2 have 802.15.4 but
no BLE).

## Building

Requires ESP-IDF >= 5.2 (tested with 5.5.5):

```bash
idf.py set-target esp32c6
idf.py build flash monitor
```

The `esp-zigbee-lib` dependency is fetched automatically by the IDF
component manager.

## Configuring the firmware

Everything you need is in [`main/zxh_config.h`](main/zxh_config.h):

```c
static const zxh_pack_cfg_t ZXH_PACKS[] = {
    { "pack_1", "F1:17:01:0E:39:31", 0 },   // name, BLE MAC, modbus addr (0 = auto)
    { "pack_2", "F0:5F:00:0E:39:31", 0 },
};

#define ZXH_POLL_INTERVAL_S 300    // per-pack polling period
#define ZXH_CONNECT_TIMEOUT_MS 15000
#define ZXH_CYCLE_TIMEOUT_MS 90000
```

Find battery MACs with `bms-cli scan` from
[`Teejer/zxh-bms`](https://github.com/Teejer/zxh-bms). If a pack never
connects, power-cycle it and watch the log — the firmware alternates
random-static/public address types automatically on connect failure.

## Pairing with zigbee2mqtt

1. Copy [`zigbee2mqtt/zxh-bms.mjs`](zigbee2mqtt/zxh-bms.mjs) into your
   z2m data folder's `external_converters/` directory. Set `MAX_PACKS` in
   the file to your pack count (must match the firmware).
2. Restart z2m, open pairing (`permit join`).
3. Power the ESP32-C6. It scans all 11–26 channels and joins the first
   open network it finds, retrying every 5 s until successful (the log
   says so).
4. z2m interviews it as `ZXH-BMS`; entities appear per endpoint:

   `voltage`, `current`, `power` (W), `soc`, `cell_min/max/delta` (mV),
   `mos_temperature`, `temperature_1/2`, `cycles`, `health`, `capacity`,
   `full_capacity`, `cell_count`, `protection` (raw bitmap),
   `protection_state` (human text), `equilibrium`, `cells` (comma list of
   mV), `balancing`, `online`.

## Attribute map (cluster `0xFF10`)

| attr | name        | type  | scale              |
|------|-------------|-------|--------------------|
| 0x01 | voltage     | u16   | centivolts         |
| 0x02 | current     | s16   | centiamperes (signed) |
| 0x03 | power       | s16   | watts              |
| 0x04 | soc         | u8    | %                  |
| 0x05 | cell_min    | u16   | mV                 |
| 0x06 | cell_max    | u16   | mV                 |
| 0x07 | cell_delta  | u16   | mV                 |
| 0x08 | mos_temp    | s16   | 0.1 °C             |
| 0x09 | temp_1      | s16   | 0.1 °C             |
| 0x0A | temp_2      | s16   | 0.1 °C             |
| 0x0B | cycles      | u16   |                    |
| 0x0C | health      | u8    | %                  |
| 0x0D | capacity    | u16   | 0.1 Ah remaining   |
| 0x0E | full_cap    | u16   | 0.1 Ah             |
| 0x0F | cell_count  | u8    |                    |
| 0x10 | protection  | u16   | bitmap (see CLI's PROTECTION_BITS) |
| 0x11 | equilibrium | u32   | balancing bitmap   |
| 0x12 | cell_mv     | octr  | `[len]` + BE u16 × N |
| 0x13 | online      | u8    | 0/1                |

Current is signed using the vendor convention — compare with
`bms-cli status` once for your pack to see which direction reads negative.

## Notes & limitations

- **No OTA.** Updates are USB flashes, like most DIY Zigbee nodes.
- First poll after power-on waits ~10 s (staggered start per pack).
- The Zigbee radio stays on all the time (router); expect the C6 to be
  warm — normal.
- If z2m shows the device as *unsupported* after pairing, the converter is
  not loaded (check `enable_external_js` and the file location).
- Protocol details: this is the same reverse-engineered protocol as the
  sibling projects; see `bms_cli/protocol.py` there for register maps.
