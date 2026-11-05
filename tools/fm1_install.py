#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
"""Install a firmware package (.fwsc) on an FM-1 over USB-MIDI.

Step 1, the
running firmware reads parts of the package and starts the update loader;
step 2, the loader reads the whole package and writes it. In both steps the
device asks (SysEx read requests on the logical image) and we answer. Then
the FM-1 restarts and the installed identity is checked.

Step 2 is the fast install when the loader can (loader/ldr_core.c,
LDR_FAST_CHUNK): before the upgrade command we say cmd 0x31 [1, 4096]; a
loader that answers it reads a table of flash.bin's 4 KiB sectors (CRC16,
CRC-32), checks it against the package before anything is erased, and asks
only for the sectors that differ, 4 KiB at a time (a cut-off install resumes:
what was written is skipped). A loader that does not answer within 0.5 s
gets the classic protocol, unchanged; --classic never asks, nor does a
package without this family's loader (the official V15: M-VAVE's loader).

  fm1_install.py PACKAGE.fwsc [--port NAME] [--yes] [--force] [--classic]
  fm1_install.py FM-1.fwsc            (the official V15 file: back to the stock firmware)
  fm1_install.py --info [--port NAME]

If the FM-1 is still in update mode (an earlier install was cut off), the
install finishes the write. Needs mido with python-rtmidi.

Exit codes: 0 done, 1 cancelled or other error, 2 bad arguments or package,
3 FM-1 not found, 4 connection lost or the device stopped, 5 timeout (no
loader / no restart), 6 wrong model, or another identity after the install.
"""
import argparse
import binascii
import hashlib
import queue
import re
import sys
import time
import zlib

HS_QUERY = bytes([0xF0, 0x00, 0x32, 0x45, 0x00, 0x00, 0x00, 0x40, 0x7F, 0xF7])
# the handshake with the one-byte body 01 (00 59 11, length 1, 01, checksum FE): a firmware on chip/ota.c answers
# HS_QUERY with its name, '_' and three digits only ("FM-1_601": M-UPGRADE 3.11 refuses more) and this one with its
# whole identity ("FM-1_6010000"). The stock firmware, the update loader and older builds don't answer it
HS_FULL = bytes([0xF0, 0x00, 0x32, 0x45, 0x08, 0x00, 0x00, 0x40, 0x00, 0x7E, 0x01, 0xF7])
UPGRADE = bytes([0xF0, 0x22, 0x24, 0x35, 0x7F, 0xF7])
FINISH_CHECK, FINISH_WRITE = 0xE0000000, 0xF0000000
MAXDATA = 512
# the fast install (loader/ldr_core.c): the hello and the loader's answer (cmd 0x31 [version 1, read size 16-bit LE]),
# the loader's progress (cmd 0x32 [sectors compared, all sectors: 16-bit LE each]), its reads of one 4 KiB sector and
# of the sector table at TABLE + flash.bin's offset (per 4 KiB sector of flash.bin: CRC16 and CRC-32, little-endian)
FAST_CMD, CHECK_CMD = 0x31, 0x32
FAST_CHUNK = 4096
TABLE, TABLE_END = 0xD0000000, 0xE0000000
BLOCKS, BLK, KEEP = 20, 0x30, 0x2F
LOADER_MARK = b"FELUCCA-LOADER-1"
# the unmodified official FM-1 V15 (FM-1.fwsc from M-VAVE): returning to it is allowed without --force,
# as Felucca's web installer does
STOCK_V15_SHA256 = "db1642b2b6fa5c2cccb11ffd13878068bb28601678d3644049f99dc40e7edb8a"
PORT_RE = re.compile(r"fm-1|felucca|ota|composite|sinco|usb-midi", re.I)   # never probe other gear

# seconds; the tests shorten them
DELAY = {"open": 0.3, "start": 2.0, "start_fast": 0.05, "fast_hello": 0.5, "reply": 0.01, "loader": 3.0, "reboot": 3.0,
         "retry": 1.0,
         "poll": 1.0, "hs": 1.0, "hs_full": 0.6, "idle_check": 8.0, "idle_write": 180.0,
         "wait_loader": 30.0, "wait_reboot": 40.0}

EXIT = {"usage": 2, "badpkg": 2, "notfound": 3, "model": 6, "lost": 4, "stopped": 4, "badreq": 4,
        "noloader": 5, "noreturn": 5, "mismatch": 6}


class InstallError(Exception):
    def __init__(self, code, msg):
        super().__init__(msg)
        self.code = code


# ------------------------------------------------------------ wire format ---

def pack7(data):
    out, acc, nb = [], 0, 0
    for b in data:
        acc |= b << nb
        nb += 8
        while nb >= 7:
            out.append(acc & 0x7F)
            acc >>= 7
            nb -= 7
    if nb:
        out.append(acc & 0x7F)
    return bytes(out)


def unpack7(s):
    out, acc, nb = [], 0, 0
    for b in s:
        acc |= (b & 0x7F) << nb
        nb += 7
        while nb >= 8:
            out.append(acc & 0xFF)
            acc >>= 8
            nb -= 8
    return bytes(out)


def response(addr, data, fl=0):
    n = len(data)
    body = bytearray([0x00, 0x59, 0x30, (n + 8) & 0xFF, ((n + 8) >> 8) & 0xFF, 0, fl])
    body += addr.to_bytes(4, "little") + bytes([n & 0xFF, (n >> 8) & 0xFF, 0]) + bytes(data)
    body.append(~sum(body[6:]) & 0xFF)
    return b"\xF0" + pack7(body) + b"\xF7"


def frame(cmd, body):
    """a host frame in the update protocol's framing: F0 pack7(00 59 cmd len24 body chk) F7"""
    n = len(body)
    u = bytes([0x00, 0x59, cmd, n & 0xFF, (n >> 8) & 0xFF, n >> 16]) + bytes(body)
    return b"\xF0" + pack7(u + bytes([~sum(body) & 0xFF])) + b"\xF7"


FAST_HELLO = frame(FAST_CMD, bytes([1, FAST_CHUNK & 0xFF, FAST_CHUNK >> 8]))


def parse_frame(pkt):
    """-> (cmd, body) of any frame in the update protocol's framing, checksum checked, or None"""
    if len(pkt) < 2 or pkt[0] != 0xF0 or pkt[-1] != 0xF7:
        return None
    u = unpack7(pkt[1:-1])
    if len(u) < 7 or u[:2] != b"\x00\x59":
        return None
    n = int.from_bytes(u[3:6], "little")
    if len(u) != 7 + n or ~sum(u[6:6 + n]) & 0xFF != u[6 + n]:
        return None
    return u[2], u[6:6 + n]


def crc16(data, c=0):
    """the UFW's CRC16 (XMODEM: poly 0x1021, init 0), as chip/ota.c ota_crc16"""
    return binascii.crc_hqx(data, c)


def sector_table(image, off, n):
    """the fast install's table: n entries, one per 4 KiB sector of the image from off, [CRC16, CRC-32] LE"""
    out = bytearray()
    for k in range(n):
        sec = image[off + k * FAST_CHUNK:off + (k + 1) * FAST_CHUNK]
        out += crc16(sec).to_bytes(2, "little") + zlib.crc32(sec).to_bytes(4, "little")
    return bytes(out)


def parse_request(pkt):
    """-> (flags, addr, len) or None"""
    if len(pkt) < 2 or pkt[0] != 0xF0 or pkt[-1] != 0xF7:
        return None
    u = unpack7(pkt[1:-1])
    if len(u) != 15 or u[:3] != b"\x00\x59\x30" or ~sum(u[6:14]) & 0xFF != u[14]:
        return None
    return u[6], int.from_bytes(u[7:11], "little"), int.from_bytes(u[11:14], "little")


class Identity:
    def __init__(self, text, model, version):
        self.text, self.model, self.version = text, model, version

    @property
    def loader(self):
        return self.model.lower().startswith("ota-")


def parse_identity(pkt):
    if len(pkt) < 2 or pkt[0] != 0xF0 or pkt[-1] != 0xF7:
        return None
    d = unpack7(pkt[1:-1])
    if len(d) != 34 or d[:3] != b"\x00\x59\x11":
        return None
    txt = d[6:33].split(b"\0", 1)[0].decode("latin-1")   # the text up to its first zero (bytes after it may carry data)
    m = re.fullmatch(r"([^_]+)_(\d+)", txt)
    return Identity(txt, m[1], int(m[2])) if m else None


# --------------------------------------------------------------- package ---

def product_of(raw):
    """the package identity ("FM-1_906"): one marker byte after each of the first 20 blocks"""
    if len(raw) < BLOCKS * BLK:
        raise InstallError("badpkg", "not an FM-1 package (too short)")
    return "".join(chr((m - i - 1) & 0xFF) for i in range(BLOCKS) if (m := raw[i * BLK + KEEP]) != 0x7D)


def logical_image(raw):
    """the image the device reads during an update: the package without the 20 marker bytes"""
    out = bytearray()
    for i in range(BLOCKS):
        out += raw[i * BLK:i * BLK + KEEP]
    return bytes(out + raw[BLOCKS * BLK:])


def model_of(text):
    return text.split("_")[0].removeprefix("ota-")


# ------------------------------------------------------------------ MIDI ---

class MidoLink:
    """one MIDI in/out pair with a SysEx queue"""

    def __init__(self, backend, in_name, out_name):
        import mido
        self.backend, self.name, self.mido = backend, in_name, mido
        self.q = queue.Queue()
        self.inp = mido.open_input(in_name, callback=self._rx)
        try:
            self.out = mido.open_output(out_name)
        except Exception:
            self.inp.close()
            raise
        time.sleep(DELAY["open"])   # CoreMIDI may drop a message sent just after opening

    def _rx(self, msg):
        if msg.type == "sysex":
            self.q.put(bytes([0xF0, *msg.data, 0xF7]))

    @property
    def lost(self):
        return self.name not in self.backend.input_names()

    def send(self, pkt):
        try:
            self.out.send(self.mido.Message("sysex", data=pkt[1:-1]))
            return True
        except Exception:
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
        for p in (self.inp, self.out):
            try:
                p.close()
            except Exception:
                pass


class MidoBackend:
    def __init__(self):
        import mido
        self.mido = mido

    def input_names(self):
        return list(dict.fromkeys(self.mido.get_input_names()))

    def output_names(self):
        return list(dict.fromkeys(self.mido.get_output_names()))

    def open(self, in_name, out_name):
        return MidoLink(self, in_name, out_name)


def pair_output(name, outs):
    if name in outs:
        return name
    base = re.sub(r"\s+\d+$", "", name)     # Windows numbers each port
    same = [o for o in outs if re.sub(r"\s+\d+$", "", o) == base]
    if len(same) == 1:
        return same[0]
    return outs[0] if len(outs) == 1 else None


# --------------------------------------------------------------- updater ---

class Device:
    def __init__(self, link, ident, name):
        self.link, self.id, self.name = link, ident, name


def ask(link, query, secs):
    """send a handshake -> the identity answered, None (no answer in secs), False (the send failed)"""
    if not link.send(query):
        return False
    end = time.monotonic() + secs
    while (left := end - time.monotonic()) > 0:
        p = link.read(left)
        if p is None:
            break
        ident = parse_identity(p)
        if ident:
            return ident
    return None


def handshake(link, tries=3):
    """the identity: HS_QUERY, then HS_FULL when the answer is NAME_ and three digits (a firmware on chip/ota.c
    tells its whole identity so); the plain answer when nothing answers HS_FULL (stock, an older build)"""
    link.drain()
    for _ in range(tries):
        ident = ask(link, HS_QUERY, DELAY["hs"])
        if ident is False:
            return None
        if ident is None:
            continue
        if ident.loader or not re.fullmatch(r"[^_]+_\d{3}", ident.text):
            return ident
        full = ask(link, HS_FULL, DELAY["hs_full"])
        return full if full and full.model == ident.model and full.text.startswith(ident.text) else ident
    return None


class Updater:
    def __init__(self, backend, port=None):
        self.backend = backend
        self.port = port

    def candidates(self, only_port):
        names = self.backend.input_names()
        if self.port:
            mine = [n for n in names if self.port.lower() in n.lower()]
            if only_port:
                return mine
            return mine + [n for n in names if n not in mine and PORT_RE.search(n)]
        return [n for n in names if PORT_RE.search(n)]

    def find(self, want=None, only_port=True):
        """the first matching port that answers the handshake (and want(identity))"""
        outs = self.backend.output_names()
        for name in self.candidates(only_port):
            out = pair_output(name, outs)
            if not out:
                continue
            try:
                link = self.backend.open(name, out)
            except Exception:
                continue
            ident = handshake(link, 2)
            if ident and (want is None or want(ident)):
                return Device(link, ident, name)
            link.close()
        return None

    def wait_for(self, want, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            dev = self.find(want, only_port=False)
            if dev:
                return dev
            time.sleep(DELAY["retry"])
        return None

    def serve(self, link, image, finish, idle, progress, loader=False, fast=False, checked=None):
        """answer read requests until the device asks for `finish`; -> (served, finished, lost).
        The loader (loader=True) may also ask 4 KiB reads and the sector table (the fast install: one it said yes to,
        fast=True, or a fast session it kept from before: the hello went unheard then) and say how far it compared
        (cmd 0x32 -> checked(k, n))"""
        served, last = 0, time.monotonic()
        while True:
            pkt = link.read(DELAY["poll"])
            if pkt is None:
                if link.lost:
                    return served, False, True
                if time.monotonic() - last > idle:
                    return served, False, False
                continue
            r = parse_request(pkt)
            if not r:
                f = parse_frame(pkt) if loader and checked else None
                if f and f[0] == CHECK_CMD and len(f[1]) == 4:
                    last = time.monotonic()
                    checked(int.from_bytes(f[1][:2], "little"), int.from_bytes(f[1][2:], "little"))
                continue
            fl, addr, n = r
            last = time.monotonic()
            if addr in (FINISH_CHECK, FINISH_WRITE):
                link.send(response(addr, b"success\0"))
                if addr == finish:
                    return served, True, False
                continue
            if loader and TABLE <= addr < TABLE_END:
                off, k = addr - TABLE, n // 6
                if n == 0 or n % 6 or n > FAST_CHUNK or off + k * FAST_CHUNK > len(image):
                    raise InstallError("badreq", f"the device asked for the table at {off:#x} for {n} B, "
                                                 "outside the package")
                data = sector_table(image, off, k)
            else:
                if n > (FAST_CHUNK if loader else MAXDATA) or addr + n > len(image):
                    raise InstallError("badreq", f"the device asked for {addr:#x}+{n}, outside the package")
                data = image[addr:addr + n]
            if not fast:
                time.sleep(DELAY["reply"])
            if not link.send(response(addr, data, fl)):
                return served, False, True
            served += 1
            progress(served, addr, n)

    def check(self, dev, image, step):
        """step 1: the running firmware checks the package and starts the loader"""
        step("start", dev.id.text)
        dev.link.send(UPGRADE)
        time.sleep(DELAY["start"])
        served, finished, lost = self.serve(dev.link, image, FINISH_CHECK, DELAY["idle_check"],
                                            lambda k, addr, n: step("check", k))
        dev.link.close()
        if not finished:
            raise InstallError("lost" if lost else "stopped",
                               f"the FM-1 {'was disconnected' if lost else 'stopped answering'} after "
                               f"{served} requests; nothing was written")
        step("loader")
        time.sleep(DELAY["loader"])
        ota = self.wait_for(lambda i: i.loader, DELAY["wait_loader"])
        if not ota:
            raise InstallError("noloader", "the update loader did not appear. Replug the USB cable and "
                                           "run the install again: the FM-1 stays in update mode until it is done")
        return ota

    def fast_hello(self, link):
        """the fast install's word: True when the loader answers it within DELAY["fast_hello"] (it holds for the next
        session); an older loader, or one still in a session it kept, says nothing"""
        link.drain()
        if not link.send(FAST_HELLO):
            return False
        end = time.monotonic() + DELAY["fast_hello"]
        while (left := end - time.monotonic()) > 0:
            p = link.read(left)
            if p is None:
                break
            f = parse_frame(p)
            if f and f[0] == FAST_CMD and len(f[1]) == 3 and f[1][0] == 1 and \
                    int.from_bytes(f[1][1:], "little") >= FAST_CHUNK:
                return True
        return False

    def write(self, ota, image, step, classic=False):
        """step 2: the loader reads and writes the package (the fast install when it says yes), then restarts"""
        step("write", 0)
        fast = not classic and self.fast_hello(ota.link)
        step("mode", "fast" if fast else "classic")
        bar = {"pct": 0, "off": None, "n": 0}

        def show(pct):                                  # the bar never goes back
            if pct > bar["pct"]:
                bar["pct"] = pct
                step("write", pct)

        def read(k, addr, n):
            if TABLE <= addr < TABLE_END:               # the table: flash.bin's offset and its sectors
                bar["off"], bar["n"] = addr - TABLE, n // 6
            elif bar["off"] is not None:                # a sector of the fast install
                if bar["n"] and addr >= bar["off"]:
                    show(min(99, ((addr - bar["off"]) // FAST_CHUNK + 1) * 100 // bar["n"]))
            else:                                       # the classic install: its two passes, as they come
                step("write", min(99, (addr + n) * 100 // len(image)))

        def checked(k, n):                              # cmd 0x32: the app area's sectors compared, kept or written
            if n:
                show(min(99, k * 100 // n))

        ota.link.send(UPGRADE)
        time.sleep(DELAY["start_fast" if fast else "start"])
        served, finished, lost = self.serve(ota.link, image, FINISH_WRITE, DELAY["idle_write"], read,
                                            loader=True, fast=fast, checked=checked)
        ota.link.close()
        if not finished:
            raise InstallError("lost" if lost else "stopped",
                               f"the loader {'was disconnected' if lost else 'stopped answering'} after "
                               f"{served} requests. Replug the USB cable and run the install again to finish")
        step("write", 100)

    def verify(self, product, step):
        step("reboot")
        time.sleep(DELAY["reboot"])
        back = self.wait_for(lambda i: not i.loader, DELAY["wait_reboot"])
        if not back:
            raise InstallError("noreturn", "the FM-1 did not come back: power-cycle it")
        back.link.close()
        if back.id.text != product:
            raise InstallError("mismatch", f"written, but the FM-1 reports {back.id.text} (expected {product})")
        step("done", back.id.text)
        return back.id.text

    def install(self, dev, image, product, step=lambda *a: None, classic=False):
        """running firmware -> loader -> new firmware; dev from find(). A device already in
        update mode only needs the write."""
        if model_of(dev.id.text) != model_of(product):
            dev.link.close()
            raise InstallError("model", f"the device is {dev.id.text}, the package is for {product}")
        ota = dev if dev.id.loader else self.check(dev, image, step)
        self.write(ota, image, step, classic)
        return self.verify(product, step)


# ------------------------------------------------------------------- CLI ---

def load_package(path, force):
    try:
        raw = open(path, "rb").read()
    except OSError as e:
        raise InstallError("badpkg", f"cannot read {path}: {e.strerror}")
    product = product_of(raw)
    if not re.fullmatch(r"[^_]+_\d+", product):
        raise InstallError("badpkg", f"{path}: not an FM-1 package (identity {product!r})")
    official = hashlib.sha256(raw).hexdigest() == STOCK_V15_SHA256
    if LOADER_MARK not in raw and not official and not force:
        raise InstallError("badpkg", f"{path}: no update loader of this family in this package; only its own "
                                     "packages and the unmodified official V15 (FM-1.fwsc) are installed "
                                     "(--force overrides)")
    return product, logical_image(raw)


def not_found(up):
    names = up.backend.input_names()
    where = f"no MIDI port matching {up.port!r} answered" if up.port else "FM-1 not found"
    return InstallError("notfound", f"{where} (USB data cable? another app using it?)"
                        f"\n  MIDI inputs: {', '.join(names) if names else '(none)'}")


class Progress:
    def __init__(self, out):
        self.out, self.line = out, False

    def __call__(self, k, a=None):
        if k == "check":
            self.put(f"checking the package ({a} reads)")
        elif k == "write":
            self.put(f"writing {a:3d}%")
        elif k == "mode":
            self.end()
            print("the loader writes only the sectors that changed (fast install)" if a == "fast" else
                  "the loader reads and writes the whole package (classic install)", file=self.out, flush=True)
        else:
            self.end()
            msg = {"start": f"starting the update on {a}", "loader": "switching to update mode",
                   "reboot": "restarting", "done": f"done: the FM-1 runs {a}"}[k]
            print(msg, file=self.out, flush=True)

    def put(self, s):
        print(f"\r{s}\033[K", end="", file=self.out, flush=True)
        self.line = True

    def end(self):
        if self.line:
            print(file=self.out, flush=True)
            self.line = False


def run(a, backend, out, ask):
    up = Updater(backend, a.port)
    if a.info:
        dev = up.find()
        if not dev:
            raise not_found(up)
        dev.link.close()
        mode = "update loader (update not finished)" if dev.id.loader else "running"
        print(f"{dev.id.text}  [{mode}]  port: {dev.name}", file=out)
        return 0
    product, image = load_package(a.package, a.force)
    print(f"package: {product}  ({a.package})", file=out)
    dev = up.find()
    if not dev:
        raise not_found(up)
    print(f"device:  {dev.id.text}  ({dev.name})" + ("  in update mode: the write will be finished" if dev.id.loader else ""),
          file=out)
    if not a.yes and not ask("Install? Do not unplug the FM-1 while writing. [y/N] "):
        dev.link.close()
        print("cancelled", file=out)
        return 1
    prog = Progress(out)
    try:
        # the fast install's hello only to a loader of this family (the package's: its marker); M-VAVE's loader,
        # which the official V15 brings, never hears it
        up.install(dev, image, product, prog, a.classic or LOADER_MARK not in image)
    finally:
        prog.end()
    return 0


def ask_tty(prompt):
    try:
        return input(prompt).strip().lower() in ("y", "yes")
    except EOFError:
        return False


def main(argv=None, backend=None, out=sys.stdout, ask=ask_tty):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("package", nargs="?", help="package (.fwsc)")
    ap.add_argument("--info", action="store_true", help="print the identity of the connected FM-1")
    ap.add_argument("--port", metavar="NAME", help="MIDI port to use (part of its name)")
    ap.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    ap.add_argument("--force", action="store_true", help="install a package without the update-loader marker")
    ap.add_argument("--classic", action="store_true", help="never ask the loader for the fast install")
    a = ap.parse_args(argv)
    if bool(a.info) == bool(a.package):
        ap.error("give a PACKAGE.fwsc or --info")
    try:
        if backend is None:
            try:
                backend = MidoBackend()
            except ImportError:
                raise InstallError("usage", "needs mido and python-rtmidi (pip install mido python-rtmidi)")
        return run(a, backend, out, ask)
    except InstallError as e:
        print(f"error: {e}", file=sys.stderr)
        return EXIT.get(e.code, 1)
    except KeyboardInterrupt:
        print("\ninterrupted: if the FM-1 is in update mode, run the install again to finish", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
