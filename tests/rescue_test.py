#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Charles Vestal
"""tools/fm1_rescue.py's write policy against a simulated FM-1 in the loader: the version whose head the FM-1
has is chosen, only its app area is written, the head and the data above it are never touched.
Made-up images stand in for M-VAVE's packages.  python3 tests/rescue_test.py"""
import os
import random
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


V15 = image(15, 0x93000)
V16 = image(16, 0xA9000)
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
        print(f"  stock packages: {seen}")
    print("rescue_test: ok")


if __name__ == "__main__":
    main()
