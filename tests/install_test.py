#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
"""tools/fm1_install.py against a simulated FM-1 (no hardware, no mido).
The fake device is the one in web/test_web.mjs: identity on the handshake,
then "device asks, host answers" reads of the logical image. Run from the repo root:
  python3 tests/install_test.py
Also checks logical_image/product_of against web/fm1pkg.js (needs node and
build/felucca.fwsc, skipped otherwise)."""
import io
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import fm1_install as I  # noqa: E402

I.DELAY.update(open=0, start=0.01, start_fast=0.01, fast_hello=0.05, reply=0, loader=0.01, reboot=0.01, retry=0.02, poll=0.05, hs=0.1, hs_full=0.1,
               idle_check=0.4, idle_write=0.4, wait_loader=2, wait_reboot=2)
failed = 0


def ok(cond, what):
    global failed
    print(f"{what:<64} {'ok' if cond else 'FAIL'}")
    failed += not cond


# ---------------------------------------------------------- simulated FM-1 ---

class FakeLink:
    def __init__(self, dev, gen):
        self.dev, self.gen, self.q, self.closed = dev, gen, queue.Queue(), False

    @property
    def lost(self):
        return self.gen != self.dev.gen or not self.dev.connected

    def send(self, pkt):
        if self.lost:
            return False
        self.dev.sent.append(bytes(pkt))
        self.dev.rx(bytes(pkt))
        return True

    def read(self, timeout):
        try:
            return self.q.get(timeout=max(timeout, 0))
        except queue.Empty:
            return None

    def drain(self):
        while not self.q.empty():
            self.q.get_nowait()

    def close(self):
        self.closed = True


class FakeFM1:
    """a MIDI backend with one FM-1 on it"""

    def __init__(self, image, identity="FM-1_015", name="FM-1", unplug_after=None, after_write="FM-1_900",
                 stall_after=None, bad_addr=None, new_ota=(), jedec=0, loader="ota-FM-1_900"):
        self.image, self.unplug_after, self.after_write, self.loader = image, unplug_after, after_write, loader
        self.new_ota, self.jedec = set(new_ota), jedec   # identities whose ota.c cuts the plain answer; PR #29's bytes
        self.stall_after, self.bad_addr = stall_after, bad_addr
        self.served = self.bad = self.upgrades = self.hs_full = 0
        self.sent, self.links, self.gen, self.lock = [], [], 0, threading.Lock()
        self.boot(identity, name)

    def boot(self, identity, name):
        with self.lock:
            self.gen += 1
            self.identity, self.name, self.connected = identity, name, True
            self.waiting, self.queue = None, []

    def input_names(self):
        return [self.name] if self.connected else []

    output_names = input_names

    def open(self, in_name, out_name):
        if in_name != self.name or not self.connected:
            raise IOError("no such port")
        link = FakeLink(self, self.gen)
        self.links.append(link)
        return link

    def tx(self, pkt):
        for link in self.links:
            if link.gen == self.gen and not link.closed:
                link.q.put(bytes(pkt))

    def ident(self, text):
        b = bytearray(text.encode() + bytes(27 - len(text)))
        b[24:27] = self.jedec.to_bytes(3, "big")
        self.tx(b"\xF0" + I.pack7(bytes([0, 0x59, 0x11, 27, 0, 0]) + b + bytes([~sum(b) & 0xFF])) + b"\xF7")

    def rx(self, d):
        new = self.identity in self.new_ota       # chip/ota.c from M-UPGRADE 3.11's fix on: NAME_ + 3 digits to HS_QUERY
        if d == I.HS_QUERY:
            self.ident(self.identity[:self.identity.index("_") + 4] if new else self.identity)
        elif d == I.HS_FULL:
            self.hs_full += 1
            if new:
                self.ident(self.identity)
        elif d == I.UPGRADE:
            self.upgrades += 1
            first = self.bad_addr if self.bad_addr is not None else 0
            self.queue = ([(first + k * 512, 512) for k in range(6)] + [(I.FINISH_WRITE, 8)]
                          if self.identity.startswith("ota-")
                          else [(0, 64), (0x40, 160), (0x1000, 512), (I.FINISH_CHECK, 8)])
            self.next()
        elif self.waiting:
            u = I.unpack7(d[1:-1])
            addr, n = self.waiting
            fin = addr >= I.FINISH_CHECK
            want = b"success\0" if fin else self.image[addr:addr + n]
            if u[14:14 + len(want)] != want or len(u) != 15 + len(want):
                self.bad += 1
            self.waiting = None
            self.served += 1
            if self.unplug_after and self.served >= self.unplug_after:
                self.connected = False
                return
            if self.stall_after and self.served >= self.stall_after:
                return
            if addr == I.FINISH_CHECK:
                threading.Timer(0.05, self.boot, (self.loader, "Felucca Update")).start()
            elif addr == I.FINISH_WRITE:
                threading.Timer(0.05, self.boot, (self.after_write, "Felucca")).start()
            else:
                self.next()

    def next(self):
        if not self.queue:
            return
        self.waiting = addr, n = self.queue.pop(0)
        u = bytearray([0, 0x59, 0x30, 0, 0, 0, 0]) + addr.to_bytes(4, "little") + bytes([n & 0xFF, n >> 8, 0])
        u.append(~sum(u[6:14]) & 0xFF)
        self.tx(b"\xF0" + I.pack7(u) + b"\xF7")


# ----------------------------------------------------------------- helpers ---

def package(product="FM-1_900", marker=True, size=0x2000):
    raw = bytearray((i * 7) & 0xFF for i in range(size + I.BLOCKS))
    for i in range(I.BLOCKS):
        raw[i * I.BLK + I.KEEP] = (ord(product[i]) + i + 1) & 0xFF if i < len(product) else 0x7D
    raw[0x1800:0x1800 + 16] = I.LOADER_MARK if marker else bytes(16)
    return bytes(raw)


TMP = Path(tempfile.mkdtemp(prefix="felucca-install-"))


def pkgfile(name, raw):
    p = TMP / name
    p.write_bytes(raw)
    return str(p)


def cli(args, dev, answer=True):
    out, err = io.StringIO(), io.StringIO()
    old, sys.stderr = sys.stderr, err
    try:
        rc = I.main(args, backend=dev, out=out, ask=lambda _p: answer)
    finally:
        sys.stderr = old
    return rc, out.getvalue(), err.getvalue()


# ------------------------------------------------------------------- tests ---

def wire():
    data = bytes(range(256)) * 3
    ok(I.unpack7(I.pack7(data))[:len(data)] == data, "pack7 / unpack7 round trip")
    pkt = I.response(0x12345, b"\x01\x02\x03", fl=5)
    u = I.unpack7(pkt[1:-1])
    ok(u[:3] == b"\x00\x59\x30" and int.from_bytes(u[7:11], "little") == 0x12345 and u[14:17] == b"\x01\x02\x03"
       and ~sum(u[6:17]) & 0xFF == u[17], "response: header, address, data, checksum")
    raw = package("FM-1_906")
    ok(I.product_of(raw) == "FM-1_906" and len(I.logical_image(raw)) == len(raw) - 20, "product_of / logical_image (synthetic)")
    ok(I.HS_FULL == b"\xF0" + I.pack7(bytes([0, 0x59, 0x11, 1, 0, 0, 1, 0xFE])) + b"\xF7"
       and I.unpack7(I.HS_FULL[1:-1])[:8] == bytes([0, 0x59, 0x11, 1, 0, 0, 1, 0xFE]),
       "HS_FULL: 00 59 11, length 1, body 01, checksum FE")
    b = bytearray(b"FM-1_6010000" + bytes(15))
    b[24:27] = b"\xB3\x60\x14"                   # felucca-bsp#29: the flash's JEDEC id after the text
    i = I.parse_identity(b"\xF0" + I.pack7(bytes([0, 0x59, 0x11, 27, 0, 0]) + b + bytes([~sum(b) & 0xFF])) + b"\xF7")
    ok(i and i.text == "FM-1_6010000" and i.version == 6010000, "parse_identity: the text up to its first zero (#29's bytes after it)")


def installs():
    raw = package()
    image = I.logical_image(raw)
    p = pkgfile("ok.fwsc", raw)

    dev = FakeFM1(image)
    rc, out, err = cli([p, "--yes"], dev)
    ok(rc == 0 and dev.bad == 0 and dev.upgrades == 2 and dev.served == 11 and "done: the FM-1 runs FM-1_900" in out
       and "100%" in out, f"install: running -> loader -> Felucca ({dev.served} reads)")

    dev = FakeFM1(image, identity="ota-FM-1_900", name="Felucca Update")
    rc, out, err = cli([p, "--yes"], dev)
    ok(rc == 0 and dev.bad == 0 and dev.upgrades == 1 and "update mode" in out, "install: device already in update mode -> finishes the write")
    ok(dev.sent.count(I.FAST_HELLO) == 1 and "classic install" in out,
       "install: the fast install's hello, unanswered (an older loader): the classic install")
    dev = FakeFM1(image, identity="ota-FM-1_900", name="Felucca Update")
    rc, out, err = cli([p, "--yes", "--classic"], dev)
    ok(rc == 0 and dev.bad == 0 and I.FAST_HELLO not in dev.sent, "install --classic: no hello")

    dev = FakeFM1(image)
    rc, out, err = cli([p], dev, answer=False)
    ok(rc == 1 and dev.upgrades == 0 and "cancelled" in out, "install: answer no -> nothing sent but the handshake")

    dev = FakeFM1(image, identity="FM-1_905", name="Felucca")
    rc, out, err = cli(["--info"], dev)
    ok(rc == 0 and "FM-1_905" in out and "running" in out and dev.upgrades == 0, "--info: identity of the connected FM-1")

    dev = FakeFM1(image, identity="ota-FM-1_900", name="Felucca Update")
    rc, out, err = cli(["--info"], dev)
    ok(rc == 0 and "update loader" in out and dev.hs_full == 0, "--info: device in update mode (never asked with 01)")

    # M-UPGRADE 3.11 takes NAME_ and 1-3 digits: chip/ota.c answers HS_QUERY with FM-1_601, HS_FULL with the whole
    rawl = package("FM-1_6010100")
    dev = FakeFM1(I.logical_image(rawl), identity="FM-1_6010000", name="FM-1", after_write="FM-1_6010100",
                  new_ota={"FM-1_6010000", "FM-1_6010100"}, jedec=0xB36014)
    rc, out, err = cli([pkgfile("long.fwsc", rawl), "--yes"], dev)
    ok(rc == 0 and dev.bad == 0 and "device:  FM-1_6010000" in out and "done: the FM-1 runs FM-1_6010100" in out,
       "install: FM-1_601 to the plain query, the whole identity to HS_FULL")
    rawl = package("FM-1_6010000")
    dev = FakeFM1(I.logical_image(rawl), identity="FM-1_6000900", name="FM-1", after_write="FM-1_6010000",
                  new_ota={"FM-1_6010000"})
    rc, out, err = cli([pkgfile("long.fwsc", rawl), "--yes"], dev)
    ok(rc == 0 and dev.bad == 0 and "done: the FM-1 runs FM-1_6010000" in out,
       "install: from a build before it (whole identity to the plain query) to one after")
    dev = FakeFM1(image, identity="FM-1_600", name="FM-1", new_ota={"FM-1_600"})
    rc, out, err = cli(["--info"], dev)
    ok(rc == 0 and "FM-1_600" in out and dev.hs_full == 1, "--info: a dev build's FM-1_600, the same both ways")
    dev = FakeFM1(image, identity="FM-1_015", name="FM-1")
    t0 = time.monotonic()
    rc, out, err = cli(["--info"], dev)
    ok(rc == 0 and "FM-1_015" in out and dev.hs_full == 1 and time.monotonic() - t0 < 2,
       "--info: the stock firmware doesn't answer HS_FULL: its plain answer")


def errors():
    raw = package()
    image = I.logical_image(raw)
    p = pkgfile("ok.fwsc", raw)

    rc, out, err = cli([p, "--yes"], FakeFM1(image, name="IAC Driver Bus 1"))
    ok(rc == 3 and "not found" in err and "IAC Driver" in err, "no FM-1 port -> exit 3, lists the MIDI inputs")
    dev = FakeFM1(image)
    rc, out, err = cli([p, "--yes", "--port", "Felucca"], dev)
    ok(rc == 3 and dev.upgrades == 0, "--port that matches nothing -> exit 3")
    dev = FakeFM1(image, name="My Interface")
    rc, out, err = cli(["--info", "--port", "my int"], dev)
    ok(rc == 0 and "FM-1_015" in out, "--port picks a port the default match skips")

    t0 = time.monotonic()
    rc, out, err = cli([p, "--yes"], FakeFM1(image, unplug_after=2))
    ok(rc == 4 and "disconnected" in err and "nothing was written" in err and time.monotonic() - t0 < 3,
       "unplugged in step 1 -> exit 4 'lost', nothing written")
    t0 = time.monotonic()
    rc, out, err = cli([p, "--yes"], FakeFM1(image, identity="ota-FM-1_900", name="Felucca Update", unplug_after=3))
    ok(rc == 4 and "run the install again" in err and time.monotonic() - t0 < 3, "unplugged during the write -> exit 4 at once")
    rc, out, err = cli([p, "--yes"], FakeFM1(image, identity="ota-FM-1_900", name="Felucca Update", stall_after=3))
    ok(rc == 4 and "stopped answering" in err, "loader stops answering -> exit 4 after the idle time")
    rc, out, err = cli([p, "--yes"], FakeFM1(image, identity="ota-FM-1_900", name="Felucca Update", bad_addr=len(image)))
    ok(rc == 4 and "outside the package" in err, "read past the package -> exit 4")

    rc, out, err = cli([p, "--yes"], FakeFM1(image, after_write="FM-1_015"))
    ok(rc == 6 and "reports FM-1_015" in err, "another identity after the restart -> exit 6")
    dev = FakeFM1(image, identity="XY-9_001", name="usb-midi")
    rc, out, err = cli([p, "--yes"], dev)
    ok(rc == 6 and dev.upgrades == 0, "another model -> exit 6, nothing sent")

    class Stuck(FakeFM1):          # the loader never shows up
        def boot(self, identity, name):
            super().boot(identity, name)
            if identity.startswith("ota-"):
                self.connected = False
    rc, out, err = cli([p, "--yes"], Stuck(image))
    ok(rc == 5 and "loader did not appear" in err, "no loader after step 1 -> exit 5")

    plain = pkgfile("plain.fwsc", package(marker=False))
    dev = FakeFM1(image)
    rc, out, err = cli([plain, "--yes"], dev)
    ok(rc == 2 and "no update loader of this family" in err and dev.sent == [], "package without the loader marker -> exit 2, no MIDI")
    dev = FakeFM1(I.logical_image(package(marker=False)))
    rc, out, err = cli([plain, "--yes", "--force"], dev)
    ok(rc == 0 and dev.bad == 0 and I.FAST_HELLO not in dev.sent,
       "... installs with --force (no fast install's hello: not this family's loader)")
    rc, out, err = cli([pkgfile("short.fwsc", b"\0" * 100), "--yes"], FakeFM1(image))
    ok(rc == 2 and "too short" in err, "not a package -> exit 2")
    rc, out, err = cli([str(TMP / "missing.fwsc"), "--yes"], FakeFM1(image))
    ok(rc == 2, "missing file -> exit 2")


def smk37():
    """the SMK-37 (boards/smk37/, loader/ldr_smk37.h): packages "SMK-37 Pro_", "SMK-37 Elite_", "MKE-P37_", the
    loader "ota-<model>_900"; its ports are found; a package goes only to its own model, both ways, nothing sent else"""
    for product, port in (("SMK-37 Elite_1100", "Melodee SMK-37"), ("MKE-P37_1100", "MKE-P37")):
        raw = package(product)
        model = product.split("_")[0]
        ok(I.product_of(raw) == product, f"product_of: {product}")
        dev = FakeFM1(I.logical_image(raw), identity=f"{model}_1000", name=port, after_write=product,
                      loader=f"ota-{model}_900")
        rc, out, err = cli([pkgfile("smk.fwsc", raw), "--yes"], dev)
        ok(rc == 0 and dev.bad == 0 and dev.upgrades == 2 and f"done: the FM-1 runs {product}" in out,
           f"install: {model}'s package on its own keyboard (port {port!r}), through its loader")
    elite = package("SMK-37 Elite_1100")
    for ident, port, what in (("SMK-37 Pro_1100", "SMK-37 Pro", "an Elite package on a Pro"),
                              ("FM-1_900", "FM-1", "an SMK-37 package on an FM-1"),
                              ("ota-FM-1_900", "Felucca Update", "an SMK-37 package on an FM-1's update loader")):
        dev = FakeFM1(I.logical_image(elite), identity=ident, name=port)
        rc, out, err = cli([pkgfile("elite.fwsc", elite), "--yes"], dev)
        ok(rc == 6 and dev.upgrades == 0 and "the package is for SMK-37 Elite_1100" in err, f"{what}: exit 6, nothing sent")
    fm1 = package()
    for ident, port, what in (("SMK-37 Elite_1100", "Melodee SMK-37", "an FM-1 package on an SMK-37"),
                              ("ota-SMK-37 Elite_900", "Felucca Update", "an FM-1 package on an SMK-37's update loader")):
        dev = FakeFM1(I.logical_image(fm1), identity=ident, name=port)
        rc, out, err = cli([pkgfile("ok.fwsc", fm1), "--yes"], dev)
        ok(rc == 6 and dev.upgrades == 0 and "the package is for FM-1_900" in err, f"{what}: exit 6, nothing sent")


def against_js():
    clean = ROOT / "build/felucca.fwsc"
    if not shutil.which("node") or not clean.exists():
        print("logical image vs fm1pkg.js: skipped (needs node and build/felucca.fwsc)")
        return
    js = ("import { logicalImage, productOf } from %r; import { readFileSync } from 'node:fs';"
          "const p = readFileSync(process.argv[1]); process.stderr.write(productOf(p));"
          "process.stdout.write(logicalImage(p));") % str(ROOT / "web/fm1pkg.js")
    r = subprocess.run(["node", "--input-type=module", "-e", js, str(clean)], capture_output=True, check=True)
    raw = clean.read_bytes()
    ok(I.logical_image(raw) == r.stdout and I.product_of(raw) == r.stderr.decode(),
       f"logical_image / product_of == fm1pkg.js ({clean.name}, {I.product_of(raw)})")
    ok(I.LOADER_MARK in raw, f"{clean.name} carries the Felucca loader marker")


def official():
    """#32: back to the official V15 from Felucca, without --force (only the unmodified file)"""
    v15 = ROOT / "FM-1_v15.fwsc"
    if not v15.exists():
        print("official V15 restore: skipped (needs FM-1_v15.fwsc in the repo root)")
        return
    raw = v15.read_bytes()
    dev = FakeFM1(I.logical_image(raw), identity="FM-1_900", name="Felucca", after_write="FM-1_015")
    rc, out, err = cli([str(v15), "--yes"], dev)
    ok(rc == 0 and dev.bad == 0 and "FM-1_015" in out and dev.identity == "FM-1_015",
       "official V15 (FM-1.fwsc) from Felucca: installed without --force, back as FM-1_015")
    bad = bytearray(raw)
    bad[-1] ^= 1
    dev = FakeFM1(I.logical_image(bytes(bad)), identity="FM-1_900", name="Felucca")
    rc, out, err = cli([pkgfile("v15mod.fwsc", bytes(bad)), "--yes"], dev)
    ok(rc == 2 and "official V15" in err and dev.sent == [], "a modified V15 is still refused (no MIDI)")


wire()
installs()
errors()
smk37()
against_js()
official()
shutil.rmtree(TMP, ignore_errors=True)
print(f"INSTALL TESTS FAILED ({failed})" if failed else "install tests passed")
sys.exit(1 if failed else 0)
