#!/usr/bin/env python3
"""
Minimal TCP server for field devices that push binary telemetry.

Supported protocols (auto-detected per connection, see PROTOCOL below):

  * Teltonika TMT250 / FMBxxx (Codec 8 / 8 Extended)
      - Accepts the IMEI handshake and replies 0x01
      - Parses GPS + IO elements and logs them
      - Replies with the record count so the device flushes its buffer

  * Tekelek TEK 811 ultrasonic level sensor (NB-IoT / CAT-M1 / GSM)
      Implements the binary format described in the "Ultrasonic
      NB-IoT/CAT-M1 User Manual" (9-5987-03):
      - 17-byte header (product, HW/FW rev, contact reason, alarms,
        CSQ, battery, IMEI, message type, payload length)
      - Message type 4 / 8 : 28 logged ullage measurements
                             (cm, SRC, RSSI, temperature) + RTC
      - Message type 6     : unit settings dump (S0..S27)
      - Message type 16    : ICCID + modem firmware
      - Message type 17    : GPS fix
      - Verifies the trailing CRC-16/XMODEM
      - Optionally replies with a command (e.g. R3=ACTIVE)

Everything is logged to stdout. Parsed messages are additionally
appended to JSON Lines / CSV files under LOG_DIR so the data can be
picked up by other tools.

Pure standard library. No dependencies.

Environment variables
---------------------
PORT             TCP port to listen on                    (default 5027)
PROTOCOL         auto | teltonika | tek                   (default auto)
LOG_DIR          directory for .jsonl / .csv data files   (default ./data,
                 set to empty string to disable file logging)
TANK_HEIGHT_CM   fallback tank height used to turn TEK ullage into a
                 fill level when the unit has not reported S18
TEK_PASSWORD     unit password used when sending commands (default TEK811)
TEK_REPLY        command(s) to send back to a TEK unit after its first
                 valid message on a connection, comma separated,
                 e.g. "R3=ACTIVE" or "R3=ACTIVE,R6=03"   (default: none)
TEK_REPLY_CRC    1 = append CRC-16/XMODEM to the reply (for units with
                 S3 CRC checking enabled)                 (default 0)
TEK_IDLE_TIMEOUT seconds to keep an idle TEK connection open (default 300)
"""

import csv
import datetime
import json
import os
import socket
import socketserver
import struct
import threading


HOST = "0.0.0.0"
PORT = int(os.environ.get("PORT", "5027"))
PROTOCOL = os.environ.get("PROTOCOL", "auto").strip().lower()
LOG_DIR = os.environ.get("LOG_DIR", "data")
TANK_HEIGHT_CM = os.environ.get("TANK_HEIGHT_CM", "").strip()
TEK_PASSWORD = os.environ.get("TEK_PASSWORD", "TEK811")
TEK_REPLY = os.environ.get("TEK_REPLY", "").strip()
TEK_REPLY_CRC = os.environ.get("TEK_REPLY_CRC", "0").strip() in ("1", "true", "yes")
TEK_IDLE_TIMEOUT = float(os.environ.get("TEK_IDLE_TIMEOUT", "300"))
TEK_BURST_GAP = 1.0   # seconds of silence that ends one TCP "burst"


# --------------------------------------------------------------------------
# Logging helpers
# --------------------------------------------------------------------------

def now():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def utcnow():
    return datetime.datetime.now(datetime.timezone.utc)


def log(tag, msg):
    print(f"[{now()}] [{tag}] {msg}", flush=True)


_file_lock = threading.Lock()


def _log_path(name):
    if not LOG_DIR:
        return None
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
    except OSError as e:
        log("WARN", f"cannot create LOG_DIR {LOG_DIR!r}: {e}")
        return None
    return os.path.join(LOG_DIR, name)


def write_jsonl(name, obj):
    path = _log_path(name)
    if not path:
        return
    try:
        with _file_lock, open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(obj, separators=(",", ":"), default=str) + "\n")
    except OSError as e:
        log("WARN", f"cannot write {path}: {e}")


def write_csv(name, fieldnames, rows):
    path = _log_path(name)
    if not path or not rows:
        return
    try:
        with _file_lock:
            new = not os.path.exists(path) or os.path.getsize(path) == 0
            with open(path, "a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
                if new:
                    w.writeheader()
                w.writerows(rows)
    except OSError as e:
        log("WARN", f"cannot write {path}: {e}")


# --------------------------------------------------------------------------
# Teltonika Codec 8 / 8E
# --------------------------------------------------------------------------

def parse_avl(data):
    """Parse the codec payload (starting at the Codec ID byte)."""
    records = []
    idx = 0
    codec_id = data[idx]; idx += 1
    num_records = data[idx]; idx += 1

    if codec_id not in (0x08, 0x8E):
        log("WARN", f"Unsupported codec 0x{codec_id:02X} (raw logged above)")
        return codec_id, num_records, records

    ext = codec_id == 0x8E  # Codec 8 Extended uses 2-byte IDs/counts

    def rd_u16():
        nonlocal idx
        v = struct.unpack(">H", data[idx:idx + 2])[0]; idx += 2
        return v

    def rd_id_or_count():
        nonlocal idx
        if ext:
            return rd_u16()
        v = data[idx]; idx += 1
        return v

    for _ in range(num_records):
        rec = {}
        ts_ms = struct.unpack(">Q", data[idx:idx + 8])[0]; idx += 8
        rec["time"] = datetime.datetime.fromtimestamp(
            ts_ms / 1000, datetime.timezone.utc
        ).strftime("%Y-%m-%d %H:%M:%S UTC")
        rec["priority"] = data[idx]; idx += 1

        lon = struct.unpack(">i", data[idx:idx + 4])[0]; idx += 4
        lat = struct.unpack(">i", data[idx:idx + 4])[0]; idx += 4
        rec["lon"] = lon / 1e7
        rec["lat"] = lat / 1e7
        rec["alt"] = struct.unpack(">h", data[idx:idx + 2])[0]; idx += 2
        rec["angle"] = struct.unpack(">H", data[idx:idx + 2])[0]; idx += 2
        rec["sat"] = data[idx]; idx += 1
        rec["speed"] = struct.unpack(">H", data[idx:idx + 2])[0]; idx += 2

        # IO element
        rec["event_io_id"] = rd_id_or_count()
        rd_id_or_count()  # total IO count (not needed for parsing)
        io = {}

        for size in (1, 2, 4, 8):
            n = rd_id_or_count()
            for _ in range(n):
                k = rd_id_or_count()
                if size == 1:
                    v = data[idx]; idx += 1
                elif size == 2:
                    v = struct.unpack(">H", data[idx:idx + 2])[0]; idx += 2
                elif size == 4:
                    v = struct.unpack(">I", data[idx:idx + 4])[0]; idx += 4
                else:
                    v = struct.unpack(">Q", data[idx:idx + 8])[0]; idx += 8
                io[k] = v

        if ext:  # variable-length IO section
            n = rd_u16()
            for _ in range(n):
                k = rd_u16()
                ln = rd_u16()
                io[k] = data[idx:idx + ln].hex()
                idx += ln

        rec["io"] = io
        records.append(rec)

    return codec_id, num_records, records


# --------------------------------------------------------------------------
# Tekelek TEK 811
# --------------------------------------------------------------------------

TEK_HEADER_LEN = 17

TEK_PRODUCTS = {
    0x00: "TEK 766", 0x02: "TEK 586", 0x03: "TEK 790", 0x05: "TEK 733",
    0x06: "TEK 643", 0x07: "TEK 811", 0x08: "TEK 822", 0x09: "TEK 733A",
    0xFF: "TEK 764",
}

TEK_MSG_TYPES = {
    4: "logger data", 6: "settings", 8: "alarm/sample data",
    16: "ICCID/modem", 17: "GPS",
}

TEK_CONTACT_REASON_BITS = [
    "scheduled", "alarm", "server_request", "manual", "reboot",
    "tsp_requested", "dynamic_limit", "dynamic_limit2",
]

TEK_ALARM_BITS = [
    "limit1", "limit2", "limit3", "bund_closed", None, None, None, "active",
]

TEK_LOG_SLOTS = 28          # positions in the type 4 / 8 logger buffer
TEK_MEAS_FIELDS = [
    "received_at", "imei", "product", "msg_type", "slot", "timestamp",
    "age_min", "ullage_cm", "level_cm", "src", "rssi", "temp_c", "raw",
]

# Tank height (cm) learned from message type 6 (S18), keyed by IMEI.
_tek_tank_height = {}


def crc16_xmodem(data):
    """CRC-16/XMODEM (poly 0x1021, init 0x0000, no reflection, no xorout).

    This is the checksum found on the manual's sample messages
    (computed over the whole message: header + body, excluding the CRC)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _bits(value, names):
    return [n for i, n in enumerate(names) if n and value & (1 << i)]


def tek_looks_like_header(buf):
    """Heuristic: does buf start with a plausible TEK header?"""
    return (len(buf) >= TEK_HEADER_LEN
            and buf[0] in TEK_PRODUCTS
            and (buf[15] & 0x3F) in TEK_MSG_TYPES)


def tek_declared_len(hdr):
    """Payload length that follows byte 16 (includes the 2-byte CRC).

    The manual states the two high bits of byte 15 extend the length
    (message type lives in the low 6 bits)."""
    return ((hdr[15] >> 6) & 0x03) * 256 + hdr[16]


def parse_tek_header(b):
    """Parse the 17-byte header common to all message types."""
    h = {}
    h["product_id"] = b[0]
    h["product"] = TEK_PRODUCTS.get(b[0], f"unknown(0x{b[0]:02X})")
    h["hw_rev"] = f"{b[1] >> 3}.{b[1] & 0x07}"          # major 5 bits . minor 3 bits
    h["fw_rev"] = f"{b[2] & 0x1F}.{b[2] >> 5}"           # as laid out in the manual
    h["contact_reason"] = _bits(b[3], TEK_CONTACT_REASON_BITS)
    h["contact_reason_raw"] = b[3]
    h["alarms"] = _bits(b[4], TEK_ALARM_BITS)
    h["alarm_raw"] = b[4]
    h["csq"] = b[5]
    h["rtc_set"] = bool(b[6] & 0x20)
    h["battery_v"] = round((30 + (b[6] & 0x1F)) / 10, 1)
    h["imei"] = b[7:15].hex().lstrip("0") or "0"          # 8 BCD bytes, leading 0
    h["msg_type"] = b[15] & 0x3F
    h["msg_type_name"] = TEK_MSG_TYPES.get(h["msg_type"], "unknown")
    h["declared_len"] = tek_declared_len(b)
    return h


def _tek_logger_interval_min(byte):
    """Byte 23 of message type 4/8 -> minutes between logged samples."""
    if byte == 0x00:
        return 1          # special test setting
    if byte == 0x80:
        return 15
    return (byte & 0x7F) * 15


def _rtc_to_datetime(hours, minutes, received):
    """The unit only reports hh:mm, so anchor it to the receive date.

    If the clock reads more than one hour *ahead* of the server the
    reading must be from the previous day (midnight roll-over)."""
    if hours > 23 or minutes > 59:
        return None
    dt = received.replace(hour=hours, minute=minutes, second=0, microsecond=0)
    if dt - received > datetime.timedelta(hours=1):
        dt -= datetime.timedelta(days=1)
    return dt


def parse_tek_measurement(raw):
    """Decode one 4-byte logger entry: RSSI | temp | SRC+cm(hi) | cm(lo)."""
    b0, b1, b2, b3 = raw
    if raw == b"\x00\x00\x00\x00":
        return None                     # empty slot
    return {
        "ullage_cm": ((b2 & 0x03) << 8) | b3,
        "src": (b2 >> 2) & 0x0F,
        "rssi": b0,
        "temp_c": round(b1 / 2 - 30, 1),
        "raw": raw.hex(),
    }


def parse_tek_type4(hdr, body, received):
    """Message type 4 (logger) and 8 (alarm / sampling buffer)."""
    d = {}
    if len(body) < 9:
        d["error"] = f"body too short ({len(body)} bytes)"
        return d
    d["message_count"] = struct.unpack(">H", body[0:2])[0]
    d["try_tickets"] = body[2] >> 5
    rtc_h = body[2] & 0x1F
    d["energy_mas"] = struct.unpack(">H", body[3:5])[0]
    d["interval_min"] = _tek_logger_interval_min(body[6])
    d["interval_raw"] = body[6]
    d["network_time_s"] = body[7] * 10
    rtc_m = body[8]
    d["rtc"] = f"{rtc_h:02d}:{rtc_m:02d}"
    base = _rtc_to_datetime(rtc_h, rtc_m, received) if hdr["rtc_set"] else None
    d["rtc_datetime"] = base.isoformat() if base else None

    # Type 8 carries the fast sampling buffer whose spacing is the
    # sampling period (S0 MSB), not the logger speed, so only type 4
    # timestamps are derived from the interval.
    timed = hdr["msg_type"] == 4

    tank_h = _tek_tank_height.get(hdr["imei"])
    if tank_h is None and TANK_HEIGHT_CM:
        try:
            tank_h = int(float(TANK_HEIGHT_CM))
        except ValueError:
            tank_h = None

    meas = []
    data = body[9:]
    n_slots = min(TEK_LOG_SLOTS, len(data) // 4)
    for i in range(n_slots):
        m = parse_tek_measurement(data[i * 4:i * 4 + 4])
        if m is None:
            continue
        m["slot"] = i
        if timed:
            m["age_min"] = i * d["interval_min"]
            if base is not None:
                m["timestamp"] = (base - datetime.timedelta(minutes=m["age_min"])).isoformat()
            else:
                m["timestamp"] = None
        else:
            m["age_min"] = None
            m["timestamp"] = None
        m["level_cm"] = (tank_h - m["ullage_cm"]) if tank_h is not None else None
        meas.append(m)
    d["measurements"] = meas
    d["tank_height_cm"] = tank_h
    return d


def _tek_ascii(body):
    return body.decode("ascii", errors="replace")


def parse_tek_type6(hdr, body, received):
    """Message type 6: comma separated S-parameter dump, e.g. S0=80,S1=01,..."""
    text = _tek_ascii(body)
    settings = {}
    for part in text.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            settings[k.strip()] = v.strip()
    d = {"text": text, "settings": settings}
    s18 = settings.get("S18")
    if s18 and s18.isdigit():
        _tek_tank_height[hdr["imei"]] = int(s18)
        d["tank_height_cm"] = int(s18)
    s0 = settings.get("S0")
    if s0:
        try:
            v = int(s0, 16)
            d["sampling_period_min"] = 15 if v & 0x80 else 1
            d["logger_speed_h"] = (v & 0x7F) * 0.25
        except ValueError:
            pass
    return d


def parse_tek_type16(hdr, body, received):
    """Message type 16: ,ICCID,modem firmware,"""
    text = _tek_ascii(body)
    f = [x for x in text.split(",")]
    d = {"text": text}
    vals = [x for x in f if x]
    if len(vals) >= 1:
        d["iccid"] = vals[0]
    if len(vals) >= 2:
        d["modem_fw"] = vals[1]
    return d


def _nmea_to_deg(val):
    """'5305.6218N' -> 53.0937 ; '00753.5957W' -> -7.8933"""
    if not val or len(val) < 4:
        return None
    hemi = val[-1].upper()
    num = val[:-1]
    try:
        dot = num.index(".") if "." in num else len(num)
        deg = int(num[:dot - 2])
        minutes = float(num[dot - 2:])
    except ValueError:
        return None
    dd = deg + minutes / 60
    if hemi in ("S", "W"):
        dd = -dd
    return round(dd, 6)


def parse_tek_type17(hdr, body, received):
    """Message type 17: GPS fix as comma separated NMEA-like fields."""
    text = _tek_ascii(body)
    f = text.split(",")
    if f and f[0] == "":
        f = f[1:]
    names = ["gps", "utc", "lat", "lon", "hdop", "alt", "fix", "cog",
             "speed_kmh", "speed_kn", "date", "nsat"]
    d = {"text": text}
    for name, val in zip(names, f):
        d[name] = val
    d["lat_deg"] = _nmea_to_deg(d.get("lat"))
    d["lon_deg"] = _nmea_to_deg(d.get("lon"))
    return d


TEK_PARSERS = {4: parse_tek_type4, 8: parse_tek_type4, 6: parse_tek_type6,
               16: parse_tek_type16, 17: parse_tek_type17}


def parse_tek_message(frame, received=None):
    """Parse one complete TEK message (header + body + CRC)."""
    received = received or utcnow()
    hdr = parse_tek_header(frame)
    msg = {"received_at": received.isoformat(), "header": hdr,
           "frame_len": len(frame), "raw": frame.hex()}

    body = frame[TEK_HEADER_LEN:]
    if len(body) >= 2:
        crc_rx = struct.unpack(">H", body[-2:])[0]
        crc_calc = crc16_xmodem(frame[:-2])
        msg["crc_rx"] = f"{crc_rx:04X}"
        msg["crc_ok"] = crc_rx == crc_calc
        if not msg["crc_ok"]:
            msg["crc_calc"] = f"{crc_calc:04X}"
        body = body[:-2]
    else:
        msg["crc_ok"] = None

    parser = TEK_PARSERS.get(hdr["msg_type"])
    if parser is None:
        msg["data"] = {"error": f"unsupported message type {hdr['msg_type']}",
                       "body_hex": body.hex()}
    else:
        try:
            msg["data"] = parser(hdr, body, received)
        except Exception as e:            # keep serving even on odd payloads
            msg["data"] = {"error": f"{type(e).__name__}: {e}", "body_hex": body.hex()}
    return msg


def split_tek_frames(buf):
    """Split a burst of bytes into TEK messages using the declared length.

    If the bytes that follow a declared frame do not look like a new
    header, they are treated as belonging to the current message (the
    manual's own type 6 sample declares a shorter length than it has)."""
    frames = []
    pos = 0
    while pos < len(buf):
        rest = buf[pos:]
        if len(rest) < TEK_HEADER_LEN:
            frames.append((rest, "short: incomplete header"))
            break
        end = TEK_HEADER_LEN + tek_declared_len(rest)
        if end > len(rest):
            frames.append((rest, f"short: declared {end - TEK_HEADER_LEN} body bytes, got {len(rest) - TEK_HEADER_LEN}"))
            break
        tail = rest[end:]
        if tail and not tek_looks_like_header(tail):
            frames.append((rest, f"declared {end - TEK_HEADER_LEN} body bytes but {len(rest) - TEK_HEADER_LEN} received; taking all"))
            break
        frames.append((rest[:end], None))
        pos += end
    return frames


def build_tek_command(commands, password=TEK_PASSWORD, with_crc=TEK_REPLY_CRC):
    """'<password>,R3=ACTIVE[,S24=05...]' as bytes, optional CRC-16 appended."""
    text = password + "," + commands
    data = text.encode("ascii")
    if with_crc:
        data += struct.pack(">H", crc16_xmodem(data))
    return data


def log_tek_message(msg, peer):
    h = msg["header"]
    imei = h["imei"]
    crc = "OK" if msg["crc_ok"] else ("n/a" if msg["crc_ok"] is None else f"BAD(calc {msg.get('crc_calc')})")
    log("TEK", f"{imei} {h['product']} type={h['msg_type']} ({h['msg_type_name']}) "
               f"len={msg['frame_len']} crc={crc} hw={h['hw_rev']} fw={h['fw_rev']} "
               f"reason={'/'.join(h['contact_reason']) or '-'} "
               f"alarms={'/'.join(h['alarms']) or '-'} csq={h['csq']} "
               f"batt={h['battery_v']}V rtc_set={h['rtc_set']}")
    d = msg["data"]
    if "error" in d:
        log("WARN", f"{imei} {d['error']}")
    t = h["msg_type"]
    if t in (4, 8) and "measurements" in d:
        log("TEK", f"{imei} rtc={d['rtc']} interval={d['interval_min']}min "
                   f"msgcount={d['message_count']} tickets={d['try_tickets']} "
                   f"energy={d['energy_mas']}mAs network={d['network_time_s']}s "
                   f"samples={len(d['measurements'])}"
                   + (f" tank={d['tank_height_cm']}cm" if d.get("tank_height_cm") else ""))
        for m in d["measurements"]:
            when = m["timestamp"] or (f"-{m['age_min']}min" if m["age_min"] is not None else "slot")
            lvl = f" level={m['level_cm']}cm" if m["level_cm"] is not None else ""
            log("LEVEL", f"{imei} #{m['slot']:02d} {when} ullage={m['ullage_cm']}cm{lvl} "
                         f"src={m['src']} rssi={m['rssi']} temp={m['temp_c']}C")
    elif t == 6:
        log("TEK", f"{imei} settings: {d.get('text', '')}")
    elif t == 16:
        log("TEK", f"{imei} iccid={d.get('iccid')} modem_fw={d.get('modem_fw')}")
    elif t == 17:
        log("TEK", f"{imei} gps lat={d.get('lat_deg')} lon={d.get('lon_deg')} "
                   f"alt={d.get('alt')} fix={d.get('fix')} sats={d.get('nsat')} "
                   f"utc={d.get('date')} {d.get('utc')}")


def store_tek_message(msg):
    write_jsonl("tek811_messages.jsonl", msg)
    d = msg["data"]
    h = msg["header"]
    if h["msg_type"] in (4, 8) and "measurements" in d:
        rows = []
        for m in d["measurements"]:
            row = dict(m)
            row.update(received_at=msg["received_at"], imei=h["imei"],
                       product=h["product"], msg_type=h["msg_type"])
            rows.append(row)
        write_csv("tek811_measurements.csv", TEK_MEAS_FIELDS, rows)


# --------------------------------------------------------------------------
# Connection handler
# --------------------------------------------------------------------------

class Handler(socketserver.BaseRequestHandler):
    def _recv_exact(self, n, first=b""):
        buf = first
        while len(buf) < n:
            chunk = self.request.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def handle(self):
        self.peer = f"{self.client_address[0]}:{self.client_address[1]}"
        log("CONN", f"New connection from {self.peer}")
        self.request.settimeout(180)
        try:
            head = self._recv_exact(2)
            if not head:
                log("CONN", f"{self.peer} closed before sending anything")
                return
            proto = PROTOCOL
            if proto == "auto":
                # Teltonika opens with a 2-byte IMEI length (0x000F).
                # A TEK header opens with the product ID byte.
                proto = "teltonika" if head[0] == 0x00 and 8 <= head[1] < 0x20 else "tek"
            log("CONN", f"{self.peer} protocol={proto}")
            if proto == "teltonika":
                self.handle_teltonika(head)
            else:
                self.handle_tek(head)
        except socket.timeout:
            log("CONN", f"{self.peer} idle timeout")
        except Exception as e:
            log("ERROR", f"{self.peer} {type(e).__name__}: {e}")
        finally:
            log("CONN", f"{self.peer} handler closed")

    # ---- Teltonika ------------------------------------------------------

    def handle_teltonika(self, head):
        peer = self.peer
        imei_len = struct.unpack(">H", head)[0]
        imei = (self._recv_exact(imei_len) or b"").decode(errors="replace")
        log("IMEI", f"{peer} -> {imei}")
        self.request.sendall(b"\x01")        # accept the device
        log("IMEI", f"{peer} accepted (sent 0x01)")

        while True:
            header = self._recv_exact(8)     # 4B preamble + 4B length
            if not header:
                log("CONN", f"{peer} ({imei}) disconnected")
                return
            _, data_len = struct.unpack(">II", header)
            payload = self._recv_exact(data_len)
            self._recv_exact(4)              # CRC-16 (not verified here)
            log("RAW", f"{imei} len={data_len} {payload.hex()}")

            codec, n, records = parse_avl(payload)
            log("DATA", f"{imei} codec=0x{codec:02X} records={n}")
            for i, r in enumerate(records, 1):
                log("GPS",
                    f"{imei} #{i} {r['time']} "
                    f"lat={r['lat']:.6f} lon={r['lon']:.6f} "
                    f"alt={r['alt']}m spd={r['speed']}km/h "
                    f"sat={r['sat']} ang={r['angle']} io={r['io']}")
                write_jsonl("teltonika_records.jsonl",
                            dict(r, imei=imei, received_at=utcnow().isoformat()))

            self.request.sendall(struct.pack(">I", n))
            log("ACK", f"{imei} acknowledged {n} record(s)")

    # ---- Tekelek TEK 811 -----------------------------------------------

    def _recv_burst(self, first):
        """Collect bytes until the line goes quiet for TEK_BURST_GAP seconds.

        Returns (bytes, still_open)."""
        buf = bytearray(first)
        self.request.settimeout(TEK_BURST_GAP)
        while True:
            try:
                chunk = self.request.recv(4096)
            except socket.timeout:
                return bytes(buf), True
            if not chunk:
                return bytes(buf), False
            buf += chunk

    def handle_tek(self, head):
        peer = self.peer
        imei = None
        replied = False
        first = head
        while True:
            if first is None:
                self.request.settimeout(TEK_IDLE_TIMEOUT)
                try:
                    first = self.request.recv(4096)
                except socket.timeout:
                    log("CONN", f"{peer} ({imei}) idle for {TEK_IDLE_TIMEOUT:.0f}s, closing")
                    return
                if not first:
                    log("CONN", f"{peer} ({imei}) disconnected")
                    return
            buf, open_ = self._recv_burst(first)
            first = None
            received = utcnow()
            log("RAW", f"{peer} len={len(buf)} {buf.hex()}")

            for frame, note in split_tek_frames(buf):
                if note:
                    log("WARN", f"{peer} {note}")
                if len(frame) < TEK_HEADER_LEN:
                    continue
                msg = parse_tek_message(frame, received)
                msg["peer"] = peer
                if note:
                    msg["framing_note"] = note
                imei = msg["header"]["imei"]
                log_tek_message(msg, peer)
                store_tek_message(msg)

                if TEK_REPLY and not replied and open_:
                    cmd = build_tek_command(TEK_REPLY)
                    self.request.sendall(cmd)
                    replied = True
                    log("CMD", f"{imei} sent {cmd!r}")

            if not open_:
                log("CONN", f"{peer} ({imei}) disconnected")
                return


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    log("BOOT", f"Listening on {HOST}:{PORT} (TCP) protocol={PROTOCOL} "
                f"log_dir={LOG_DIR or '-'} tek_reply={TEK_REPLY or '-'}")
    with ThreadedTCPServer((HOST, PORT), Handler) as srv:
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            log("BOOT", "Shutting down")
