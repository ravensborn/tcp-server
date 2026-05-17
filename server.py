#!/usr/bin/env python3
"""
Minimal Teltonika TCP server for testing TMT250 / FMBxxx devices.

It does the one thing a raw `netcat` listener CANNOT do: the IMEI
handshake. A Teltonika device sends its IMEI first and waits for a
single 0x01 byte before it will transmit any AVL data. If it does
not get that byte it disconnects and you see "nothing", which is
exactly the symptom you're describing.

This server:
  1. Accepts the IMEI handshake and replies 0x01
  2. Receives AVL packets (Codec 8 and Codec 8 Extended)
  3. Parses GPS + IO elements and logs them
  4. Replies with the record count so the device flushes its buffer

Pure standard library. No dependencies.
"""

import os
import socket
import socketserver
import struct
import datetime


HOST = "0.0.0.0"
PORT = int(os.environ.get("PORT", "5027"))


def now():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def log(tag, msg):
    print(f"[{now()}] [{tag}] {msg}", flush=True)


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


class Handler(socketserver.BaseRequestHandler):
    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.request.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def handle(self):
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        log("CONN", f"New connection from {peer}")
        imei = None
        self.request.settimeout(180)
        try:
            head = self._recv_exact(2)
            if not head:
                log("CONN", f"{peer} closed before sending IMEI")
                return
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

                self.request.sendall(struct.pack(">I", n))
                log("ACK", f"{imei} acknowledged {n} record(s)")

        except socket.timeout:
            log("CONN", f"{peer} ({imei}) idle timeout")
        except Exception as e:
            log("ERROR", f"{peer} ({imei}) {type(e).__name__}: {e}")
        finally:
            log("CONN", f"{peer} ({imei}) handler closed")


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    log("BOOT", f"Teltonika test server listening on {HOST}:{PORT} (TCP)")
    with ThreadedTCPServer((HOST, PORT), Handler) as srv:
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            log("BOOT", "Shutting down")
