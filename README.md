# tcp-server

Small, dependency-free Python TCP server that receives and logs telemetry from:

* **Tekelek TEK 811** ultrasonic level sensors (NB-IoT / CAT-M1 / GSM) – binary
  protocol from the *Ultrasonic NB-IoT/CAT-M1 User Manual* (9-5987-03)
* **Teltonika** TMT250 / FMBxxx trackers (Codec 8 / 8 Extended)

The protocol is detected per connection, so both device types can point at the
same port.

## Run

```sh
python3 server.py              # listens on 0.0.0.0:5027
# or
docker compose up -d
```

Point the sensor at the server's IP and port (TEK 811: parameters `S15` / `S16`).

## TEK 811 – what gets logged

Every message is printed to stdout and appended to files under `LOG_DIR`
(default `./data`; the compose file sets `LOG_DIR=` so the container logs to stdout only):

| File | Content |
|------|---------|
| `tek811_messages.jsonl` | one JSON object per message: full header, parsed body, raw hex, CRC result |
| `tek811_measurements.csv` | one row per logged reading (type 4 / 8): time, ullage cm, level cm, SRC, RSSI, temperature |
| `teltonika_records.jsonl` | one JSON object per Teltonika AVL record |

Header fields decoded for every message: product, HW/FW revision, contact
reason (scheduled / alarm / manual / …), alarm flags, CSQ, battery voltage,
RTC-set flag, IMEI, message type, declared length and CRC-16/XMODEM check.

Message types:

| Type | Meaning | Decoded |
|------|---------|---------|
| 4 | scheduled logger upload | RTC, logger interval, 28 readings with per-reading timestamps |
| 8 | alarm / magnet wake-up (sampling buffer) | same layout as type 4 (no timestamps, spacing is the sampling period) |
| 6 | settings dump | `S0..S27` as a dict; `S18` (tank height) is remembered per IMEI to compute fill level |
| 16 | ICCID + modem firmware | both strings |
| 17 | GPS | lat/lon in decimal degrees, altitude, fix, satellites, UTC |

Example stdout for a type 4 upload:

```
[TEK]   866425030672135 TEK 811 type=4 (logger data) len=140 crc=OK hw=4.1 fw=2.0 reason=scheduled alarms=limit1/limit2/active csq=6 batt=5.0V rtc_set=True
[TEK]   866425030672135 rtc=23:15 interval=30min msgcount=1 tickets=4 energy=988mAs network=10s samples=25
[LEVEL] 866425030672135 #00 2026-10-01T23:15:00+00:00 ullage=119cm src=10 rssi=10 temp=15.5C
[LEVEL] 866425030672135 #01 2026-10-01T22:45:00+00:00 ullage=119cm src=10 rssi=10 temp=15.5C
...
```

The unit only reports the time of day, so timestamps are anchored to the
server's receive date (readings whose clock is ahead of the server roll back a
day). When `rtc_set` is false no timestamps are produced, only the age in
minutes relative to the newest reading.

## Configuration

| Variable | Default | Purpose |
|----------|---------|---------|
| `PORT` | `5027` | TCP listen port |
| `PROTOCOL` | `auto` | `auto`, `teltonika` or `tek` |
| `LOG_DIR` | `data` | where the `.jsonl` / `.csv` files go; empty disables file logging |
| `TANK_HEIGHT_CM` | – | fallback tank height for computing `level_cm = tank - ullage` |
| `TEK_REPLY` | – | command(s) sent back after the first valid message, e.g. `R3=ACTIVE` (required once to activate a new unit) or `R3=ACTIVE,R6=03` |
| `TEK_PASSWORD` | `TEK811` | unit password prefixed to commands |
| `TEK_REPLY_CRC` | `0` | append CRC-16/XMODEM to commands (units with S3 CRC checking on) |
| `TEK_IDLE_TIMEOUT` | `300` | seconds before an idle TEK connection is closed |

## Testing without hardware

`tek811_sim.py` replays the manual's sample messages (types 4, 8, 6, 16, 17):

```sh
python3 server.py &
python3 tek811_sim.py 127.0.0.1 5027 --split
```
