#!/usr/bin/env python3
"""Decode a .wfl capture written by the standalone capture on the M5StickC.

The board writes a binary log because a ride produces ~72 records per second
and hex over a 115200 baud console cannot keep up. This turns one back into
something greppable, and checks the Fardriver CRC while it is at it.

    ./scripts/wfl.py cap0001.wfl                 # tagged one-line records
    ./scripts/wfl.py cap0001.wfl --header        # just the header
    ./scripts/wfl.py cap0001.wfl --csv out.csv   # frames as CSV
    ./scripts/wfl.py cap0001.wfl --type 0x80     # only Fardriver type 0x80
"""
import argparse
import csv
import datetime
import struct
import sys

MAGIC = b"WFCAP1\0"
HDR_FMT = "<8sHHIqII18s18s32s"
REC_FMT = "<BBI"
TELEM_FMT = "<HbbIII"
IMU_FMT = "<hhhhhh"

WFREC_MCU, WFREC_BMS, WFREC_EVENT, WFREC_TELEM, WFREC_IMU = 0x01, 0x02, 0x10, 0x11, 0x12

# The IMU runs at fixed full scales, so the raw counts convert with constants.
ACCEL_LSB_PER_G = 4096.0
GYRO_LSB_PER_DPS = 16.4


def decode_daly(payload):
    """Decode a 0xd2 Modbus read-holding-registers response from the Daly BMS.

    Register map confirmed against cap0007 (docs/fardriver-fields.md, Daly
    BMS section) - a 129-byte response to the capture's poll of 62 registers
    starting at 0. Returns None for anything else (short frames, a different
    address/count, or the truncated fragments an older firmware produced)."""
    if len(payload) < 5 or payload[0] != 0xd2 or payload[1] != 0x03:
        return None
    n = payload[2]
    if len(payload) < 3 + n + 2 or n % 2:
        return None
    regs = struct.unpack(f">{n // 2}H", payload[3:3 + n])

    def reg(i):
        return regs[i] if i < len(regs) else None

    pack_v, current_raw, soc = reg(40), reg(41), reg(42)
    temp_hi, temp_lo = reg(45), reg(46)
    return dict(
        pack_v=pack_v / 10.0 if pack_v is not None else None,
        current_a=(current_raw - 30000) / 10.0 if current_raw is not None else None,
        soc_pct=soc / 10.0 if soc is not None else None,
        cell_max_mv=reg(43), cell_min_mv=reg(44),
        temp_hi_c=temp_hi - 40 if temp_hi is not None else None,
        temp_lo_c=temp_lo - 40 if temp_lo is not None else None,
        cell_count=reg(49), avg_cell_mv=reg(55),
        cell_mv=regs[0:28],
    )


def crc16(data, init=0x7F3C):
    """Modbus CRC-16 with the Fardriver's unusual initial value. Run over all
    16 bytes of a frame it leaves a residue of 0."""
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def read_header(f):
    raw = f.read(struct.calcsize(HDR_FMT))
    if len(raw) < struct.calcsize(HDR_FMT):
        sys.exit("file too short to hold a header")
    (magic, version, hdr_len, seq, unix_start, boot_ms, duration_ms,
     mcu, bms, note) = struct.unpack(HDR_FMT, raw)
    if not magic.startswith(MAGIC[:6]):
        sys.exit(f"not a wfl capture (magic {magic!r})")
    h = dict(version=version, hdr_len=hdr_len, seq=seq, unix_start=unix_start,
             boot_ms=boot_ms, duration_ms=duration_ms,
             mcu=mcu.split(b"\0")[0].decode(errors="replace"),
             bms=bms.split(b"\0")[0].decode(errors="replace"),
             note=note.split(b"\0")[0].decode(errors="replace"))
    f.seek(hdr_len)
    return h


def records(f):
    size = struct.calcsize(REC_FMT)
    while True:
        raw = f.read(size)
        if len(raw) < size:
            return
        rtype, length, t_ms = struct.unpack(REC_FMT, raw)
        payload = f.read(length)
        if len(payload) < length:
            return   # truncated tail, e.g. the bike lost power mid-write
        yield rtype, t_ms, payload


def when(unix_start):
    if not unix_start:
        return "unknown"
    return datetime.datetime.fromtimestamp(
        unix_start, datetime.timezone.utc).isoformat()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--header", action="store_true", help="print the header and stop")
    ap.add_argument("--csv", metavar="OUT", help="write frames to a CSV file")
    ap.add_argument("--imu-csv", metavar="OUT", dest="imu_csv",
                    help="write IMU samples to a CSV file, in g and dps")
    ap.add_argument("--src", choices=["mcu", "bms"], help="only one link")
    ap.add_argument("--type", help="only this Fardriver frame type, e.g. 0x80")
    ap.add_argument("--quiet", action="store_true", help="summary only")
    args = ap.parse_args()

    want_type = int(args.type, 0) if args.type else None

    with open(args.file, "rb") as f:
        h = read_header(f)
        print("HDR seq={seq} version={version} mcu={mcu} bms={bms} note={note}".format(**h),
              f"start={when(h['unix_start'])} duration_ms={h['duration_ms']}")
        if args.header:
            return

        counts = {WFREC_MCU: 0, WFREC_BMS: 0, WFREC_EVENT: 0, WFREC_TELEM: 0,
                  WFREC_IMU: 0}
        markers = []
        crc_bad = 0
        types = {}
        writer = None
        out = None
        imu_writer = None
        imu_out = None
        if args.imu_csv:
            imu_out = open(args.imu_csv, "w", newline="")
            imu_writer = csv.writer(imu_out)
            imu_writer.writerow(["t_ms", "ax_g", "ay_g", "az_g",
                                 "gx_dps", "gy_dps", "gz_dps"])
        if args.csv:
            out = open(args.csv, "w", newline="")
            writer = csv.writer(out)
            writer.writerow(["t_ms", "src", "type", "hex"])

        for rtype, t_ms, payload in records(f):
            counts[rtype] = counts.get(rtype, 0) + 1

            if rtype == WFREC_EVENT:
                text = payload.split(b"\0")[0].decode(errors="replace")
                if text.startswith("marker"):
                    markers.append((t_ms, text))
                if not args.quiet:
                    print(f"EVT t={t_ms} {text}")
                continue

            if rtype == WFREC_IMU and len(payload) >= struct.calcsize(IMU_FMT):
                ax, ay, az, gx, gy, gz = struct.unpack(
                    IMU_FMT, payload[:struct.calcsize(IMU_FMT)])
                acc = [v / ACCEL_LSB_PER_G for v in (ax, ay, az)]
                gyr = [v / GYRO_LSB_PER_DPS for v in (gx, gy, gz)]
                if imu_writer:
                    imu_writer.writerow([t_ms] + [f"{v:.4f}" for v in acc + gyr])
                elif not args.quiet:
                    print(f"IMU t={t_ms} a=({acc[0]:+.3f},{acc[1]:+.3f},{acc[2]:+.3f})g "
                          f"g=({gyr[0]:+.1f},{gyr[1]:+.1f},{gyr[2]:+.1f})dps")
                continue

            if rtype == WFREC_TELEM and len(payload) >= struct.calcsize(TELEM_FMT):
                mv, r_mcu, r_bms, n_mcu, n_bms, drop = struct.unpack(
                    TELEM_FMT, payload[:struct.calcsize(TELEM_FMT)])
                if not args.quiet:
                    print(f"TLM t={t_ms} batt_mv={mv} rssi_mcu={r_mcu} rssi_bms={r_bms} "
                          f"mcu={n_mcu} bms={n_bms} dropped={drop}")
                continue

            src = "mcu" if rtype == WFREC_MCU else "bms"
            if args.src and args.src != src:
                continue

            if src == "bms" and want_type is None:
                d = decode_daly(payload)
                if d is not None:
                    if not args.quiet:
                        print(f"BMS t={t_ms} pack_v={d['pack_v']:.1f} "
                              f"current_a={d['current_a']:+.1f} soc_pct={d['soc_pct']:.1f} "
                              f"cell_max_mv={d['cell_max_mv']} cell_min_mv={d['cell_min_mv']} "
                              f"temp_hi_c={d['temp_hi_c']} temp_lo_c={d['temp_lo_c']} "
                              f"cells={d['cell_count']} avg_mv={d['avg_cell_mv']}")
                    continue

            ftype = payload[1] if src == "mcu" and len(payload) > 1 else None
            if src == "mcu":
                types[ftype] = types.get(ftype, 0) + 1
                if len(payload) == 16 and crc16(payload) != 0:
                    crc_bad += 1
            if want_type is not None and ftype != want_type:
                continue

            hexs = payload.hex()
            if writer:
                writer.writerow([t_ms, src, f"0x{ftype:02x}" if ftype is not None else "", hexs])
            elif not args.quiet:
                print(f"FRM t={t_ms} src={src} len={len(payload)} data={hexs}")

        if out:
            out.close()
        if imu_out:
            imu_out.close()

        span = h["duration_ms"] / 1000.0 if h["duration_ms"] else 0.0
        summary = (f"SUM mcu={counts.get(WFREC_MCU, 0)} bms={counts.get(WFREC_BMS, 0)} "
                   f"events={counts.get(WFREC_EVENT, 0)} "
                   f"telem={counts.get(WFREC_TELEM, 0)} imu={counts.get(WFREC_IMU, 0)} "
                   f"crc_bad={crc_bad}")
        if span:
            summary += (f" seconds={span:.1f} "
                        f"mcu_rate={counts.get(WFREC_MCU, 0) / span:.1f}/s")
        print(summary)

        for t_ms, text in markers:
            print(f"MARK t={t_ms} {text}")

        if types:
            seen = " ".join(f"{t:02x}" for t in sorted(k for k in types if k is not None))
            print(f"TYPES n={len(types)} {seen}")


if __name__ == "__main__":
    main()
