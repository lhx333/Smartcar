"""Offline protocol/logging checks; no serial device is opened."""
import binascii
import csv
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import protocol

def frame(kind, payload):
    raw = bytes([0x10 | kind]) + payload
    raw += binascii.crc_hqx(raw, 0xffff).to_bytes(2, "little")
    out = bytearray([0]); at = 0; code = 1
    for byte in raw:
        if byte:
            out.append(byte); code += 1
        else:
            out[at] = code; at = len(out); out.append(0); code = 1
    out[at] = code
    return bytes(out) + b"\0"

def fast(tick=10):
    return frame(1, protocol.FAST.pack(tick, 0x20, 3, -1234, 567, -123,
        1001, 999, -127, 75, 60, 61, -250, 1500, 3000, 12, 1000))

class Checks(unittest.TestCase):
    def test_crc_reference(self):
        self.assertEqual(binascii.crc_hqx(b"123456789", 0xffff), 0x29b1)

    def test_fast_size_and_units(self):
        self.assertEqual(protocol.FAST.size, 27)
        self.assertEqual(len(fast()), 32)
        r = protocol.Decoder().feed(fast(), 1)[0][1]
        self.assertEqual((r["state"], r["yaw"], r["vl"], r["servo_offset"], r["duty_l"]),
                         ("RUN", -12.34, 1.001, 15, 12))

    def test_byte_fragmentation_and_concatenation(self):
        d=protocol.Decoder(); result=[]
        for byte in fast(10)+fast(11):result += d.feed(bytes([byte]), 1)
        self.assertEqual([v["tick"] for k,v in result], [10,11])

    def test_corruption_and_resynchronization(self):
        d=protocol.Decoder(); bad=bytearray(fast()); bad[8] ^= 1
        result=d.feed(bytes(bad)+b"\x08bad\0"+fast(11), 1)
        self.assertEqual(len(result),1)
        self.assertGreaterEqual(d.bad,2)
        self.assertEqual(result[0][1]["tick"],11)

    def test_rollover_loss_and_duplicates(self):
        d=protocol.Decoder()
        d.feed(fast(65535),1)
        r=d.feed(fast(0)+fast(0)+fast(4),1.02)
        self.assertEqual(len(r),2)
        self.assertEqual(r[-1][1]["missing"],3)
        self.assertAlmostEqual(r[-1][1]["time_s"],.025)

    def test_outage_and_boot_segment(self):
        d=protocol.Decoder(); d.feed(fast(10),1)
        self.assertEqual(d.feed(fast(11),330)[0][1]["segment"],1)
        d.feed(frame(2,b"BOOT,PROTO1,5MS,200HZ\r\n"),330.01)
        self.assertEqual(d.feed(fast(1),330.02)[0][1]["segment"],2)

    def test_oversized_and_diagnostic(self):
        d=protocol.Decoder()
        data=frame(3,protocol.struct.pack("<IiiIIIII",100,-20,30,6000,0,1,2,3))
        r=d.feed(b"x"*400+b"\0"+data,1)
        self.assertEqual(d.bad,1)
        self.assertEqual(r[0][1]["enc_l"],-20)

    def test_every_sample_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(protocol,"__file__",str(Path(tmp)/"protocol.py")):
                recorder=protocol.Recorder(); d=protocol.Decoder()
                for i in range(200):
                    raw=fast(i); recorder.write(raw,d.feed(raw,1+i*.005),d.bad)
                output=recorder.path; recorder.close()
                with output.open(encoding="utf-8-sig",newline="") as f:rows=list(csv.DictReader(f))
                self.assertEqual(len(rows),200)
                self.assertEqual(rows[-1]["tick"],"199")
                self.assertEqual(output.with_suffix(".bin").stat().st_size,6400)

if __name__ == "__main__":unittest.main(verbosity=2)
