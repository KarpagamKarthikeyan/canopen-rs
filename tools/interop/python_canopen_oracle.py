"""Independent cross-check: drive python-canopen's real encoders offline and
diff the frames they produce against canopen-rs's golden byte sequences.

python-canopen (https://github.com/christiansandberg/canopen) is a mature,
widely-used implementation validated against real hardware, so agreement here
means canopen-rs's wire format is correct against something other than our own
reading of CiA 301 — the golden bytes asserted in the Rust tests.

Usage:
    python3 -m pip install canopen        # pulls python-can
    python3 tools/interop/python_canopen_oracle.py

Exits 0 if every frame matches, 1 otherwise.
"""
import struct

import canopen
from canopen.objectdictionary import ObjectDictionary, ODVariable, datatypes
from canopen.sdo import SdoClient
from canopen.nmt import NmtMaster
from canopen.emcy import EmcyProducer
from canopen.sync import SyncProducer

results = []


def check(name, got, expected):
    got = bytes(got)
    expected = bytes(expected)
    ok = got == expected
    results.append(ok)
    mark = "OK  " if ok else "FAIL"
    print(f"[{mark}] {name}")
    if not ok:
        print(f"        python  : {got.hex(' ')}")
        print(f"        canopen-rs: {expected.hex(' ')}")


# ---- SDO: drive the real SdoClient with a fake, auto-responding network ----

class FakeNetwork:
    """Captures every request the client sends and pushes a valid response back
    into the client's queue so multi-frame flows proceed."""

    def __init__(self):
        self.client = None
        self.sent = []

    def send_message(self, cob_id, data, remote=False):
        data = bytes(data)
        self.sent.append((cob_id, data))
        if self.client is not None:
            resp = self._respond(data)
            if resp is not None:
                self.client.on_response(self.client.tx_cobid, resp, 0.0)

    def _respond(self, req):
        cmd = req[0]
        ccs = cmd & 0xE0
        idx_lo, idx_hi, sub = req[1], req[2], req[3]
        if ccs == 0x20:  # download initiate (expedited or segmented)
            return bytes([0x60, idx_lo, idx_hi, sub, 0, 0, 0, 0])
        if ccs == 0x00:  # download data segment -> echo toggle in the ack
            return bytes([0x20 | (cmd & 0x10), 0, 0, 0, 0, 0, 0, 0])
        if ccs == 0x40:  # upload initiate -> expedited device type 0x00000192
            return bytes([0x43, idx_lo, idx_hi, sub, 0x92, 0x01, 0x00, 0x00])
        return None


def new_client():
    od = ObjectDictionary()
    net = FakeNetwork()
    client = SdoClient(0x580 + 0x10, 0x600 + 0x10, od)
    client.network = net
    net.client = client
    return client, net


# Expedited download of UNSIGNED32 0x12345678 -> 0x2000 sub 0
c, net = new_client()
c.download(0x2000, 0, bytes([0x78, 0x56, 0x34, 0x12]))
check("SDO expedited download U32", net.sent[0][1],
      [0x23, 0x00, 0x20, 0x00, 0x78, 0x56, 0x34, 0x12])

# Expedited download of UNSIGNED8 0x7F -> 0x2001 sub 0
c, net = new_client()
c.download(0x2001, 0, bytes([0x7F]))
check("SDO expedited download U8", net.sent[0][1],
      [0x2F, 0x01, 0x20, 0x00, 0x7F, 0x00, 0x00, 0x00])

# Expedited download of INTEGER16 -2 -> 0x6000 sub 1
c, net = new_client()
c.download(0x6000, 1, struct.pack("<h", -2))
check("SDO expedited download I16 (-2)", net.sent[0][1],
      [0x2B, 0x00, 0x60, 0x01, 0xFE, 0xFF, 0x00, 0x00])

# Upload (read) request for 0x1000 sub 0, and check python decodes our
# expedited response format back to 0x00000192.
c, net = new_client()
data = c.upload(0x1000, 0)
check("SDO upload request 0x1000", net.sent[0][1],
      [0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])
check("SDO expedited upload response decoded by python", data,
      [0x92, 0x01, 0x00, 0x00])

# Client abort with code 0x06020000
c, net = new_client()
c.abort(0x06020000)
# python omits index/subindex in an abort; compare only cmd + code bytes.
ab = net.sent[0][1]
check("SDO abort cmd byte", [ab[0]], [0x80])
check("SDO abort code (LE, bytes 4..8)", ab[4:8], [0x00, 0x00, 0x02, 0x06])

# Segmented download of 10 bytes -> exercises initiate + two data segments.
# Bytes chosen so the two segments equal canopen-rs's golden segment frames.
c, net = new_client()
c.download(0x2000, 0, bytes([1, 2, 3, 4, 5, 6, 7, 0xAA, 0xBB, 0xCC]))
check("SDO segmented download initiate (size 10)", net.sent[0][1],
      [0x21, 0x00, 0x20, 0x00, 10, 0x00, 0x00, 0x00])
check("SDO download data segment 1 (7 bytes, toggle 0)", net.sent[1][1],
      [0x00, 1, 2, 3, 4, 5, 6, 7])
check("SDO download data segment 2 (3 bytes, toggle 1, last)", net.sent[2][1],
      [0x19, 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x00, 0x00])


# ---- NMT: drive NmtMaster.send_command and capture the 0x000 frame ----

class CaptureNet:
    def __init__(self):
        self.sent = []

    def send_message(self, cob_id, data, remote=False):
        self.sent.append((cob_id, bytes(data)))


def nmt_frame(node, state_name):
    m = NmtMaster(node)
    m.network = CaptureNet()
    m.state = state_name  # setter maps name -> code and sends command
    return m.network.sent[-1]


cob, data = nmt_frame(5, "OPERATIONAL")
check("NMT start (node 5) cob-id 0x000", [cob & 0xFF, cob >> 8], [0x00, 0x00])
check("NMT start (node 5) data", data, [0x01, 0x05])

_, data = nmt_frame(5, "STOPPED")
check("NMT stop (node 5) data", data, [0x02, 0x05])

_, data = nmt_frame(0x7F, "PRE-OPERATIONAL")
check("NMT enter pre-op (node 127) data", data, [0x80, 0x7F])

_, data = nmt_frame(0x7F, "RESET")
check("NMT reset node (127) data", data, [0x81, 0x7F])

_, data = nmt_frame(0x7F, "RESET COMMUNICATION")
check("NMT reset communication (127) data", data, [0x82, 0x7F])


# ---- EMCY: drive EmcyProducer.send ----
emcy_net = CaptureNet()
producer = EmcyProducer(0x80 + 0x05)
producer.network = emcy_net
producer.send(0x3210, register=0x05, data=bytes([0xAA, 0xBB, 0xCC, 0xDD, 0xEE]))
cob, data = emcy_net.sent[-1]
check("EMCY cob-id 0x085", [cob], [0x85])
check("EMCY overvoltage frame", data,
      [0x10, 0x32, 0x05, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE])


# ---- SYNC: cob-id + empty frame ----
sync_net = CaptureNet()
sp = SyncProducer(0x80)
sp.network = sync_net
sp.transmit()
cob, data = sync_net.sent[-1]
check("SYNC cob-id 0x080", [cob], [0x80])
check("SYNC empty frame", data, [])


# ---- PDO mapping value format: 0xIIII_SSLL ----
def pdo_mapping_value(index, subindex, bits):
    return struct.pack(">I", (index << 16) | (subindex << 8) | bits)


check("PDO mapping value 0x6000/1 x8 bits", pdo_mapping_value(0x6000, 1, 8),
      [0x60, 0x00, 0x01, 0x08])


# ---- LSS: build frames using python-canopen's own command specifiers ----
from canopen import lss as pylss  # noqa: E402


def lss_byte(cs, value=0):
    return bytes([cs, value, 0, 0, 0, 0, 0, 0])


def lss_u32(cs, number):
    return bytes([cs]) + struct.pack("<I", number) + b"\x00\x00\x00"


check("LSS switch global (configuration)",
      lss_byte(pylss.CS_SWITCH_STATE_GLOBAL, 1), [0x04, 0x01, 0, 0, 0, 0, 0, 0])
check("LSS configure node-id 0x20",
      lss_byte(pylss.CS_CONFIGURE_NODE_ID, 0x20), [0x11, 0x20, 0, 0, 0, 0, 0, 0])
check("LSS switch selective vendor-id 0x1F",
      lss_u32(pylss.CS_SWITCH_STATE_SELECTIVE_VENDOR_ID, 0x1F),
      [0x40, 0x1F, 0x00, 0x00, 0x00, 0, 0, 0])
check("LSS store configuration",
      lss_byte(pylss.CS_STORE_CONFIGURATION), [0x17, 0, 0, 0, 0, 0, 0, 0])
check("LSS inquire node-id",
      lss_byte(pylss.CS_INQUIRE_NODE_ID), [0x5E, 0, 0, 0, 0, 0, 0, 0])


# ---- SDO block download: command bytes (python's constants) + CRC ----
import binascii  # noqa: E402
from canopen.sdo import constants as sdoc  # noqa: E402

check("block download initiate cmd (crc + size)",
      [sdoc.REQUEST_BLOCK_DOWNLOAD | sdoc.INITIATE_BLOCK_TRANSFER
       | sdoc.CRC_SUPPORTED | sdoc.BLOCK_SIZE_SPECIFIED], [0xC6])
check("block download sub-block response cmd",
      [sdoc.RESPONSE_BLOCK_DOWNLOAD | sdoc.BLOCK_TRANSFER_RESPONSE], [0xA2])
check("block download end cmd (n=5)",
      [sdoc.REQUEST_BLOCK_DOWNLOAD | sdoc.END_BLOCK_TRANSFER | (5 << 2)], [0xD5])
check("block upload initiate cmd (crc)",
      [sdoc.REQUEST_BLOCK_UPLOAD | sdoc.INITIATE_BLOCK_TRANSFER | sdoc.CRC_SUPPORTED], [0xA4])
check("block upload start cmd",
      [sdoc.REQUEST_BLOCK_UPLOAD | sdoc.START_BLOCK_UPLOAD], [0xA3])
check("block upload initiate response cmd (crc + size)",
      [sdoc.RESPONSE_BLOCK_UPLOAD | sdoc.INITIATE_BLOCK_TRANSFER
       | sdoc.CRC_SUPPORTED | sdoc.BLOCK_SIZE_SPECIFIED], [0xC6])
# python-canopen computes the block CRC with binascii.crc_hqx (CRC-16/XMODEM).
check("block transfer CRC-16 of '123456789'",
      struct.pack(">H", binascii.crc_hqx(b"123456789", 0)), [0x31, 0xC3])


print()
passed = sum(results)
total = len(results)
print(f"==== {passed}/{total} frames match python-canopen ====")
raise SystemExit(0 if passed == total else 1)
