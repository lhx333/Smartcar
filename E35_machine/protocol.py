"""Protocol v1: COBS + CCITT-FALSE CRC, little endian, 5 ms sample ticks."""
import binascii
import csv
import json
import struct
import time
from datetime import datetime
from pathlib import Path

FAST = struct.Struct("<HBBhhhhhbbBBhhHBH")
STATES = ("INIT", "ARMED", "RUN", "DONE", "ERROR")
FIELDS = ("time_s", "tick", "segment", "missing", "state", "error", "flags",
          "yaw", "yaw_error", "gz", "vl", "vr", "turn", "servo_offset",
          "duty_l", "duty_r", "x", "y", "progress", "path", "speed_target")

def cobs_decode(data):
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if not code or i + code - 1 > len(data):
            raise ValueError("invalid COBS")
        out.extend(data[i:i + code - 1])
        i += code - 1
        if code != 255 and i < len(data):
            out.append(0)
    return bytes(out)

class Decoder:
    def __init__(self):
        self.buf = bytearray()
        self.discard = False
        self.bad = self.missing = self.segment = 0
        self.previous = None
        self.extended = 0
        self.last_sample = None
        self.new_segment = False

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        result = []
        for byte in data:
            if byte:
                if not self.discard:
                    self.buf.append(byte)
                    if len(self.buf) > 192:
                        self.buf.clear()
                        self.discard = True
                continue
            if self.discard:
                self.discard = False
                self.bad += 1
                continue
            if not self.buf:
                continue
            encoded = bytes(self.buf)
            self.buf.clear()
            try:
                raw = cobs_decode(encoded)
                if len(raw) < 3 or binascii.crc_hqx(raw[:-2], 0xffff) != int.from_bytes(raw[-2:], "little"):
                    raise ValueError("CRC")
                if raw[0] >> 4 != 1:
                    raise ValueError("version")
                kind, payload = raw[0] & 15, raw[1:-2]
                if kind == 1:
                    if len(payload) != FAST.size:
                        raise ValueError("fast length")
                    tick, status, flags, yaw, error, gz, vl, vr, turn, servo, dl, dr, x, y, progress, path, target = FAST.unpack(payload)
                    if status >> 4 >= len(STATES):
                        raise ValueError("state")
                    gap = 0 if self.last_sample is None else now - self.last_sample
                    delta = 0 if self.previous is None else (tick - self.previous) & 65535
                    # A >500 ms receive outage or boot makes a new, explicit segment.
                    if self.new_segment or gap > .5:
                        self.segment += 1
                        self.previous = None
                        self.extended = 0
                        delta = 0
                        self.new_segment = False
                    if self.previous is not None and delta == 0:
                        continue  # duplicate state report, no duplicate sample
                    if self.previous is not None and delta > 32768:
                        self.segment += 1  # reset without a received BOOT
                        self.extended = 0
                        delta = 0
                    missing = max(0, delta - 1)
                    self.missing += missing
                    self.extended += delta
                    self.previous, self.last_sample = tick, now
                    values = (self.extended * .005, tick, self.segment, missing,
                              STATES[status >> 4], status & 15, flags, yaw / 100,
                              error / 100, gz / 10, vl / 1000, vr / 1000, turn / 127,
                              servo / 5, dl / 5, dr / 5, x / 1000, y / 1000,
                              progress / 1000, path, target / 1000)
                    result.append(("sample", dict(zip(FIELDS, values))))
                elif kind == 2:
                    text = payload.decode("utf-8", "strict")
                    if text.startswith("BOOT,"):
                        self.new_segment = self.previous is not None
                    result.append(("event", text))
                elif kind == 3:
                    if len(payload) != 32:
                        raise ValueError("diagnostic length")
                    names = ("tick32", "enc_l", "enc_r", "control_peak_cycles", "missed_ticks", "tx_dropped", "tx_errors", "rx_errors")
                    result.append(("diagnostic", dict(zip(names, struct.unpack("<IiiIIIII", payload)))))
                else:
                    raise ValueError("type")
            except (ValueError, struct.error, UnicodeError):
                self.bad += 1
        return result

class Recorder:
    """Buffered disk I/O in the serial reader thread; GUI never drops log rows."""
    def __init__(self):
        directory = Path(__file__).resolve().parent / "logs"
        directory.mkdir(exist_ok=True)
        stem = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = directory / (stem + ".csv")
        self.csvfile = self.path.open("w", encoding="utf-8-sig", newline="")
        self.eventfile = self.path.with_suffix(".jsonl").open("w", encoding="utf-8")
        self.rawfile = self.path.with_suffix(".bin").open("wb")
        self.writer = csv.DictWriter(self.csvfile, fieldnames=("host_time", *FIELDS))
        self.writer.writeheader()
        self.last_flush = time.monotonic()
        self.event("session", {"protocol": 1, "tick_ms": 5, "fast_hz": 200})

    def event(self, kind, value):
        self.eventfile.write(json.dumps({"host_time": datetime.now().isoformat(timespec="milliseconds"), "kind": kind, "data": value}, ensure_ascii=False) + "\n")

    def write(self, raw, frames, bad):
        self.rawfile.write(raw)
        for kind, value in frames:
            if kind == "sample":
                self.writer.writerow({"host_time": datetime.now().isoformat(timespec="milliseconds"), **value})
                if value["missing"]:
                    self.event("missing", {"tick": value["tick"], "count": value["missing"]})
            else:
                self.event(kind, value)
        if time.monotonic() - self.last_flush >= 1:
            self.event("parser", {"invalid_frames": bad})
            for f in (self.csvfile, self.eventfile, self.rawfile):
                f.flush()
            self.last_flush = time.monotonic()

    def close(self):
        for f in (self.csvfile, self.eventfile, self.rawfile):
            f.close()
