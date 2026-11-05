#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Charles Vestal
"""tools/fm1_install.py's step 2 against the real update loader (loader/ldr_core.c, chip/ota.c) on a pipe
(tests/ldr_pipe.c, built here with the C compiler: CC, default cc): the classic install from an older host, the fast
install, a loader from before the fast install, the same package again, a power cut halfway and the install resumed,
the host gone halfway while the loader keeps its session, and a damaged table refused before anything is erased.
Run from the repo root:  python3 tests/install_fast_test.py
(FM1_INSTALL_TOOLS=DIR: the fm1_install.py in DIR instead of tools/)"""
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# FM1_INSTALL_TOOLS: another tools directory's fm1_install.py (fm1-base's, with this repo as its bsp/)
sys.path.insert(0, os.environ.get("FM1_INSTALL_TOOLS") or str(ROOT / "tools"))
import fm1_install as I  # noqa: E402

I.DELAY.update(open=0, start=0.05, start_fast=0.01, fast_hello=0.5, reply=0, retry=0.02, poll=0.05, hs=0.3,
               hs_full=0.1, idle_write=3)
APP_LO, APP_HI, FL_OFF = 0x4000, 0x93000, 0x400
failed = 0


def ok(cond, what):
    global failed
    print(f"{what:<78} {'ok' if cond else 'FAIL'}")
    failed += not cond


class Loader:
    """one FM-1 in its update loader: the pipe process, and the requests it made"""

    def __init__(self, exe, flash, cut_after=None):
        self.exe, self.flash, self.cut_after = exe, flash, cut_after
        self.link = None
        self.start()

    def start(self):
        self.proc = subprocess.Popen([self.exe, "run", self.flash], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE)
        self.reqs, self.checks, self.hellos = [], [], 0
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        buf = bytearray()
        fd = self.proc.stdout.fileno()
        while True:
            try:
                b = os.read(fd, 65536)
            except OSError:
                return
            if not b:
                return
            for x in b:
                if x == 0xF0:
                    buf = bytearray()
                buf.append(x)
                if x == 0xF7 and buf[:1] == b"\xF0":
                    self.got(bytes(buf))
                    buf = bytearray()

    def got(self, pkt):
        r = I.parse_request(pkt)
        f = I.parse_frame(pkt)
        if r:
            self.reqs.append(r[1:])
            if self.cut_after is not None and sum(n == I.FAST_CHUNK for _, n in self.reqs) > self.cut_after:
                self.proc.kill()                 # the power goes, between two flash operations
                return
        elif f and f[0] == I.CHECK_CMD:
            self.checks.append((int.from_bytes(f[1][:2], "little"), int.from_bytes(f[1][2:], "little")))
        elif f and f[0] == I.FAST_CMD:
            self.hellos += 1
        if self.link and not self.link.closed:
            self.link.q.put(pkt)

    # the MIDI backend
    def input_names(self):
        return ["Felucca Update"] if self.proc.poll() is None else []

    output_names = input_names

    def open(self, in_name, out_name):
        if self.proc.poll() is not None:
            raise IOError("no such port")
        self.link = Link(self)
        return self.link

    def wait(self):
        try:
            return self.proc.wait(5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            return None

    def sectors(self):
        return [a for a, n in self.reqs if n == I.FAST_CHUNK and a < I.TABLE]


class Link:
    def __init__(self, dev):
        self.dev, self.q, self.closed, self.unplugged = dev, queue.Queue(), False, False

    @property
    def lost(self):
        return self.unplugged or self.dev.proc.poll() is not None

    def send(self, pkt):
        if self.lost:
            return False
        try:
            self.dev.proc.stdin.write(pkt)
            self.dev.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError):
            return False

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


TMP = Path(tempfile.mkdtemp(prefix="felucca-fast-"))


def build(name, fast):
    exe = TMP / name
    flags = ["-DLDR_FAST_CHUNK=4096u", "-DOTA_FRAME_MAX=4200u"] if fast else []
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-D_DEFAULT_SOURCE", "-O1", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-function", "-Ichip", *flags, "tests/ldr_pipe.c", "-o", str(exe)], cwd=ROOT,
                   check=True)
    return str(exe)


def device(exe, name):
    flash, image = TMP / f"{name}.nor", TMP / f"{name}.img"
    subprocess.run([exe, "init", str(flash), str(image)], check=True)
    return str(flash), image.read_bytes()


def app(flash):
    return Path(flash).read_bytes()[APP_LO:APP_HI]


def done(flash, image):
    """the app area is the package's, the loader's record (0xE4F00) is gone with its sector"""
    nor = Path(flash).read_bytes()
    return nor[APP_LO:APP_HI] == image[FL_OFF + APP_LO:FL_OFF + APP_HI] and nor[0xE4000:0xE5000] == b"\xFF" * 0x1000


def write(dev, image, classic=False):
    """step 2 as fm1_install.py runs it: find the loader, write; -> (error code or None, the bar's values)"""
    up = I.Updater(dev)
    ota = up.find(lambda i: i.loader)
    if not ota:
        return "notfound", []
    bar = []
    try:
        up.write(ota, image, lambda k, a=None: bar.append(a) if k == "write" else None, classic)
    except I.InstallError as e:
        return e.code, bar
    return None, bar


def main():
    fast_exe, old_exe = build("ldr_fast", True), build("ldr_old", False)
    n_app = (APP_HI - APP_LO) // I.FAST_CHUNK

    flash, image = device(fast_exe, "classic")
    dev = Loader(fast_exe, flash)
    err, bar = write(dev, image, classic=True)
    ok(err is None and dev.wait() == 0 and done(flash, image) and dev.hellos == 0 and not dev.checks
       and max(n for _, n in dev.reqs if _ < I.TABLE) <= I.MAXDATA and not any(a >= I.TABLE for a, _ in dev.reqs
                                                                                  if a < I.FINISH_CHECK),
       f"classic host (--classic), new loader: the classic install ({len(dev.reqs)} reads of 512 B at most)")

    flash, image = device(old_exe, "old")
    dev = Loader(old_exe, flash)
    err, bar = write(dev, image)
    ok(err is None and dev.wait() == 0 and done(flash, image) and dev.hellos == 0
       and max(n for _, n in dev.reqs) <= I.MAXDATA,
       f"new host, loader from before the fast install: no answer, classic ({len(dev.reqs)} reads)")

    flash, image = device(fast_exe, "fast")
    before = app(flash)
    changed = sum(before[k:k + 4096] != image[FL_OFF + APP_LO + k:FL_OFF + APP_LO + k + 4096]
                  for k in range(0, APP_HI - APP_LO, 4096))
    dev = Loader(fast_exe, flash)
    err, bar = write(dev, image)
    secs = dev.sectors()
    ok(err is None and dev.wait() == 0 and done(flash, image) and dev.hellos == 1 and len(secs) == changed
       and any(a >= I.TABLE and a < I.TABLE_END for a, _ in dev.reqs),
       f"fast install: the table, then only the {changed} changed sectors of {n_app}, 4 KiB each")
    ok(dev.checks and dev.checks[-1] == (n_app, n_app) and bar == sorted(bar) and bar[-1] == 100,
       f"fast install: the loader's 0x32 frames ({len(dev.checks)}), the bar never goes back")

    subprocess.run([fast_exe, "rearm", flash], check=True)
    dev = Loader(fast_exe, flash)
    err, bar = write(dev, image)
    ok(err is None and dev.wait() == 0 and done(flash, image) and dev.sectors() == [],
       "the same package again: the table only, no sector sent")

    flash, image = device(fast_exe, "cut")
    dev = Loader(fast_exe, flash, cut_after=40)
    err, bar = write(dev, image)
    first = len(dev.sectors())
    mid = app(flash)
    ok(err == "lost" and not done(flash, image) and Path(flash).read_bytes()[0xE4F00:0xE4F08] != b"\xFF" * 8,
       f"power cut after {first - 1} sectors: the install stops, the record stays (the loader runs again)")
    dev = Loader(fast_exe, flash)                 # the next power-on: the SPL starts the loader again
    err, bar = write(dev, image)
    rest = sum(mid[k:k + 4096] != image[FL_OFF + APP_LO + k:FL_OFF + APP_LO + k + 4096]
               for k in range(0, APP_HI - APP_LO, 4096))
    ok(err is None and dev.wait() == 0 and done(flash, image) and len(dev.sectors()) == rest and rest < changed,
       f"... resumed: only the {rest} sectors still different are sent")

    flash, image = device(fast_exe, "kept")
    dev = Loader(fast_exe, flash)
    up = I.Updater(dev)
    ota = up.find(lambda i: i.loader)
    seen = []

    def unplug(k, a=None):
        if k == "write" and len(dev.sectors()) >= 20:
            ota.link.unplugged = True             # the host goes (the cable, the command killed)
        seen.append(k)
    try:
        up.write(ota, image, unplug)
        err = None
    except I.InstallError as e:
        err = e.code
    n1 = len(dev.sectors())
    err2, bar = write(dev, image)                 # back within the loader's read tries: it kept its session
    ok(err == "lost" and err2 is None and dev.wait() == 0 and done(flash, image) and dev.hellos == 1,
       f"host gone after {n1} sectors, back while the loader kept its fast session: finished")

    flash, image = device(fast_exe, "damaged")
    before = Path(flash).read_bytes()
    real = I.sector_table

    def damaged(img, off, n):
        t = bytearray(real(img, off, n))
        t[9 * 6] ^= 1
        return bytes(t)
    I.sector_table = damaged
    dev = Loader(fast_exe, flash)
    I.DELAY["idle_write"] = 1.0
    err, bar = write(dev, image)
    I.sector_table = real
    dev.proc.kill()
    err_text = dev.proc.stderr.read().decode()
    ok(err == "stopped" and "session -12" in err_text and Path(flash).read_bytes() == before,
       "a damaged table: refused (-12) with nothing erased or written")

    ok(I.FAST_HELLO.hex() == "f0003245190000400000203807f7", "the hello: the vector shared with the other hosts")
    data = bytes(((i * 31) ^ (i >> 8)) & 0xFF for i in range(8192))
    ok(I.sector_table(data, 0, 2).hex() == "6204a25f925f5cc59b283583", "the table: the vector shared with the hosts")


if not shutil.which(os.environ.get("CC", "cc")):
    print("install_fast_test: skipped (no C compiler)")
    sys.exit(0)
try:
    main()
finally:
    shutil.rmtree(TMP, ignore_errors=True)
print(f"FAST INSTALL TESTS FAILED ({failed})" if failed else "fast install tests passed")
sys.exit(1 if failed else 0)
