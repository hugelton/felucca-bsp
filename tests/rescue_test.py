#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Charles Vestal
"""tools/fm1_rescue.py's write policy against a simulated FM-1 in the loader: the version whose head the FM-1
has is chosen, only its app area is written, the head and the data above it are never touched.
Made-up images stand in for M-VAVE's packages.  python3 tests/rescue_test.py"""
import os
import random
import struct
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import fm1_rescue as r  # noqa: E402


class FakeFM1(r.Uboot):
    """The loader's commands on a 1 MiB bytearray; everything above the transport is the real code."""

    def __init__(self, flash):
        self.flash = bytearray(flash)
        self.erased, self.ran = [], False

    def inquiry(self):
        return "WL82", "UBOOT1.00"

    def start_loader(self, blob):
        pass

    def chip(self):
        return 0x980F, 3, 0x856014

    def read(self, addr, n):
        return bytes(self.flash[addr:addr + n])

    def cmd(self, cmd, args=b""):
        if cmd == r.CMD_ERASE_SECTOR:
            a = int.from_bytes(args[:4], "big")
            self.erased.append(a)
            self.flash[a:a + r.SECTOR] = b"\xFF" * r.SECTOR
        return b""

    def write_block(self, cmd, addr, data):
        assert cmd == r.CMD_WRITE_FLASH
        self.flash[addr:addr + len(data)] = data

    def run_app(self):
        self.ran = True


def image(seed, end):
    rnd = random.Random(seed)
    return bytes(rnd.getrandbits(8) for _ in range(end))


def entry(name, off, size, data, index):
    e = bytearray(struct.pack("<HHIIBBH", 0, r.crc16(data), off, size, 2, 0, index) + name.ljust(16, b"\0"))
    e[0:2] = struct.pack("<H", r.crc16(e[2:]))
    r.enc(e)
    return bytes(e)


def with_head(img, boot_seed):
    """IMG with a flash head laid out as the FM-1's: header, then the entries of uboot.boot and isd_config.ini"""
    img = bytearray(img)
    img[0x60:0x80] = b"\xFF" * 0x20                        # after the last entry: no valid CRC
    img[0xA0:0x3938] = image(boot_seed, 0x3938 - 0xA0)
    set_isd(img, image(boot_seed + 100, 0x2F3))
    img[0x20:0x40] = entry(b"uboot.boot", 0xA0, 0x3898, img[0xA0:0x3938], 0)
    return bytes(img)


def set_isd(img, data, entry_crc=True, data_crc=True):
    """isd_config.ini's contents and its entry, the CRCs written as asked"""
    img[0x3938:0x3938 + len(data)] = data
    e = bytearray(entry(b"isd_config.ini", 0x3938, 0x2F3, data if data_crc else b"x", 1))
    if not entry_crc:
        e[0] ^= 1
    img[0x40:0x60] = e


V15 = with_head(image(15, 0x93000), 150)
V16 = with_head(image(16, 0xA9000), 160)
PKGS = [("V15", V15), ("V16", V16)]


def device(head, rest_seed=1):
    """an FM-1 with HEAD's bootloader and some other firmware and data after it"""
    other = bytearray(image(rest_seed, r.FLASH_SIZE))
    other[:r.WRITE_MIN] = head[:r.WRITE_MIN]
    return FakeFM1(other)


def run(dev, write=True, pkgs=PKGS):
    with tempfile.TemporaryDirectory() as d:
        return r.rescue(dev, pkgs, b"", write, d, say=lambda *a, **k: None)


def fails(fn, text):
    try:
        fn()
    except r.Fail as e:
        assert text in str(e), e
        return
    raise AssertionError(f"no Fail ({text})")


def main():
    for name, img in PKGS:
        dev = device(img)
        before = bytes(dev.flash)
        assert run(dev) is True and dev.ran
        end = len(img)
        assert bytes(dev.flash[:end]) == img, name
        assert dev.flash[end:] == before[end:], f"{name}: data above the app area touched"
        assert min(dev.erased) >= r.WRITE_MIN and max(dev.erased) < end, name
        # the order the packages come in does not matter
        dev = device(img, 2)
        assert run(dev, pkgs=PKGS[::-1]) and bytes(dev.flash[:end]) == img

    # V15's head with only V16 given (and the other way round): nothing written
    for head, pkgs in ((V15, [PKGS[1]]), (V16, [PKGS[0]]), (image(99, 0x4000), PKGS)):
        dev = device(head)
        before = bytes(dev.flash)
        fails(lambda: run(dev, pkgs=pkgs), "not writing")
        assert bytes(dev.flash) == before and not dev.erased

    # V16's head with isd_config.ini rewritten by another tool (fm1-x0x#55): written, the head kept as it is
    for img in (V15, V16):
        head = bytearray(img[:r.WRITE_MIN])
        set_isd(head, bytes(img[0x3938:0x3938 + 0x2F3 - 34]) + b"\x10" * 34)
        dev = device(head)
        before = bytes(dev.flash)
        assert r.isd_changed(img, before) == 34
        assert run(dev) is True and dev.ran
        end = len(img)
        assert bytes(dev.flash[:r.WRITE_MIN]) == bytes(head), "head touched"
        assert bytes(dev.flash[r.WRITE_MIN:end]) == img[r.WRITE_MIN:end]
        assert dev.flash[end:] == before[end:]
        assert min(dev.erased) >= r.WRITE_MIN

    # ... but not when its CRCs are wrong, the entry moved, or anything else in the head differs
    def bad(change):
        head = bytearray(V16[:r.WRITE_MIN])
        change(head)
        dev = device(head)
        before = bytes(dev.flash)
        assert r.isd_changed(V16, before) == 0
        fails(lambda: run(dev), "not writing")
        assert bytes(dev.flash) == before and not dev.erased
    new_isd = b"\x10" * 0x2F3
    bad(lambda h: set_isd(h, new_isd, data_crc=False))           # stale data CRC
    bad(lambda h: set_isd(h, new_isd, entry_crc=False))          # broken entry
    bad(lambda h: h.__setitem__(slice(0x3938, 0x3938 + 0x2F3), new_isd))   # contents changed, CRCs not
    bad(lambda h: h.__setitem__(0x1000, h[0x1000] ^ 1))          # the bootloader's code

    def moved(h):
        e = bytearray(h[0x40:0x60])
        r.enc(e)
        e[8:12] = struct.pack("<I", 0x2F4)
        e[0:2] = struct.pack("<H", r.crc16(e[2:]))
        r.enc(e)
        h[0x40:0x60] = e
    bad(moved)                                                   # size changed

    def both(h):
        set_isd(h, new_isd)
        h[0x2000] ^= 1
    bad(both)                                                    # isd_config.ini and the bootloader

    # without --write: a check, nothing written
    dev = device(V16)
    before = bytes(dev.flash)
    assert run(dev, write=False) is False and bytes(dev.flash) == before and not dev.erased

    # the sector guard
    dev = device(V16)
    sec = b"\0" * r.SECTOR
    fails(lambda: dev.write_sector(0x3000, sec, 0xA9000), "refusing")
    fails(lambda: dev.write_sector(0xA9000, sec, 0xA9000), "refusing")
    fails(lambda: dev.write_sector(0xE0000, sec, 0xE1000), "refusing")
    dev.write_sector(0xA8000, sec, 0xA9000)

    # the real packages, when they are around (FM1_STOCK_DIR: V15 and V16's .fwsc)
    sd = os.environ.get("FM1_STOCK_DIR")
    if sd:
        import hashlib
        seen = {}
        for f in sorted(os.listdir(sd)):
            if f.endswith(".fwsc"):
                raw = open(os.path.join(sd, f), "rb").read()
                n = r.STOCK.get(hashlib.sha256(raw).hexdigest())
                if n:
                    seen[n] = len(r.package_flash_image(raw))
        assert seen.get("V15", 0x93000) == 0x93000 and seen.get("V16", 0xA9000) == 0xA9000, seen
        # their heads read as the rescue reads them, and V16's with its isd_config.ini rewritten
        for f in sorted(os.listdir(sd)):
            raw = open(os.path.join(sd, f), "rb").read() if f.endswith(".fwsc") else b""
            if r.STOCK.get(hashlib.sha256(raw).hexdigest()) == "V16":
                img = r.package_flash_image(raw)
                at, e = r.head_entries(img)[b"isd_config.ini"]
                off, size = struct.unpack_from("<II", e, 4)
                head = bytearray(img[:r.WRITE_MIN])
                head[off:off + 34] = b"\x10" * 34
                e = bytearray(e)
                e[2:4] = struct.pack("<H", r.crc16(head[off:off + size]))
                e[0:2] = struct.pack("<H", r.crc16(e[2:]))
                r.enc(e)
                head[at:at + 0x20] = e
                assert r.isd_changed(img, bytes(head) + img[r.WRITE_MIN:]) > 0
        print(f"  stock packages: {seen}")
    print("rescue_test: ok")


if __name__ == "__main__":
    main()
