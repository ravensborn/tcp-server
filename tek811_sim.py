#!/usr/bin/env python3
"""
Replay the TEK 811 sample messages from the user manual (9-5987-03)
against the server, so the parser can be exercised without a sensor.

    python3 tek811_sim.py [host] [port] [--split]

--split sends the first message in several TCP segments to show the
burst re-assembly working.  Each message is recomputed with a valid
CRC-16/XMODEM so the server reports crc=OK; pass --raw-crc to send
the manual's (partly mistyped) trailers instead.
"""

import socket
import struct
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0] or ".")
from server import crc16_xmodem  # noqa: E402

# Message type 4: 1 record, logger speed 30 min, RTC 23:15, 25 readings.
TYPE4 = ("072102018B06340866425030672135" "047B"
         "00018D03DC3C82010F"
         + "0A5B2877" * 3 + "0A5B2876" + "0A5B2877" + "0A5B2876" + "0A5B2877"
         + "0A5B2876" * 3 + "0A5B2877" + "0A5B2876" * 3 + "0A5D2877" * 10
         + "0A5F2877" + "00000000" * 3 + "EEBA")

# Message type 8 built from the type 4 sample (alarm contact, limit1 set).
TYPE8 = ("072102028906340866425030672135" "087B"
         "00018D03DC3C82010F"
         + "0A5B2866" * 10 + "00000000" * 18 + "0000")

# Message type 6: settings dump (manual declares 0xA5 but body is 265 bytes).
TYPE6 = ("07206204611718086642503008238406A5"
         "53303D38302C53313D30312C53323D3746303033382C53333D36342C53343D303831452C"
         "53353D383833322C53363D383834362C53373D30302C53383D30302C53393D2B333533"
         "3836313735363336342C5331303D2B3335333836313735363336342C5331313D54454B"
         "3733332C5331323D73747265616D2E636F2E756B2C5331333D73747265616D69702C53"
         "31343D73747265616D69702C5331353D38342E35312E3235302E3130342C5331363D39"
         "3030302C5331373D303034393030323832382C5331383D3530302C5331393D30303030"
         "2C5332303D30302C5332313D2C5332323D2C5332333D31332C5332343D30302C533235"
         "3D30302C5332363D31302C5332373D3838D9BA")

# Message type 16: ICCID + modem firmware (body built from the manual's ASCII,
# its hex listing is mistyped).
TYPE16 = ("0721A10060171B0866425030678884" "1028"
          + ",8935302120590072401F,BG96MAR02A07M1G,".encode().hex() + "858D")

# Message type 17: GPS fix.
TYPE17 = ("0721DF04830B380866425030679882" "114A"
          "2C34352C3131323330342E302C353330352E363231384E2C30303735332E35393537572C"
          "312E302C36322E302C322C3134322E34352C302E302C302E302C3039303531392C30392C"
          "E998")


def fix(hexstr, raw_crc=False):
    """Fix up the declared length and (unless raw_crc) the CRC."""
    b = bytearray(bytes.fromhex(hexstr))
    body_len = len(b) - 17
    b[15] = (b[15] & 0x3F) | (((body_len >> 8) & 0x03) << 6)
    b[16] = body_len & 0xFF
    if not raw_crc:
        b[-2:] = struct.pack(">H", crc16_xmodem(bytes(b[:-2])))
    return bytes(b)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    host = args[0] if len(args) > 0 else "127.0.0.1"
    port = int(args[1]) if len(args) > 1 else 5027
    split = "--split" in sys.argv
    raw_crc = "--raw-crc" in sys.argv

    msgs = [("type 4", TYPE4), ("type 8", TYPE8), ("type 6", TYPE6),
            ("type 16", TYPE16), ("type 17", TYPE17)]
    for name, h in msgs:
        data = fix(h, raw_crc)
        with socket.create_connection((host, port), timeout=10) as s:
            if split and name == "type 4":
                for i in range(0, len(data), 40):
                    s.sendall(data[i:i + 40])
                    time.sleep(0.2)
            else:
                s.sendall(data)
            print(f"sent {name}: {len(data)} bytes")
            s.settimeout(2.5)
            try:
                reply = s.recv(256)
                if reply:
                    print(f"  server replied: {reply!r}")
            except socket.timeout:
                pass
        time.sleep(0.3)


if __name__ == "__main__":
    main()
