# Flash access

`chip/fm1_flash.h` provides runtime SPI NOR access. Its RAM routines temporarily
switch off the SFC/XIP mapping; normal firmware must use the `fl_*` wrappers.
The application must copy `.ram_text` into RAM before calling them, initialize
TIMER4, and provide RAM buffers. The example linker script and startup show this.

## Configure an application's regions

Define configuration before the first inclusion of `fm1_flash.h` (or
`fm1_clock.h`, which includes it):

```c
/* All offsets are physical flash offsets; high bounds are exclusive. */
#define FM1_FLASH_WRITE_REGIONS(X) \
    X(0x97000u, 0xB0000u)          /* application data */
#include "fm1_flash.h"
```

A write/erase must fit wholly within one region, even if two regions touch.
An empty `FM1_FLASH_WRITE_REGIONS(X)` allows reads only. Regions must be
constant expressions: no partition table or callback may be fetched from XIP
while it is disabled. The physical-capacity check always applies, including
when firmware supplies the older `FL_RANGE_OK(off, n)` override. If both are
specified, `FL_RANGE_OK` takes precedence.

For source compatibility, omitting both selects the former Felucca layout:

| Physical offsets | Existing use |
| --- | --- |
| `0x93000..0x97000` | Extra data |
| `0x97000..0xE0000` | Main store |
| `0xE0000..0xE5000` | OTA staging |
| `0xE5000..0xE7000` | Autosave |
| `0xE7000..0xFC000` | Extra data |
| `0xFC000..0xFF000` | Globals |

The legacy `FL_DATA_*`, `FL_GLOB_*`, `FL_AUTO_*`, `FL_OTA_*`, `FL_XDATA*`,
`FL_STORE_OK` and `FL_XDATA_OK` names remain available in `fm1_flash_policy.h`.
They do not add write permissions when an application supplies its own policy.

The updater has a separate, unchanged write policy: application sectors
`0x4000..0x97000` and update records `0xE0000..0xFC000`. Changing runtime data
regions does not change what the updater overwrites or preserves. Its app image
ends below `0x93000`, but it can erase the rest of the containing application
window. In the extra-data regions, keep bytes `0xF00..0xF07` of every 4 KiB sector
erased (or unlike an update record): the loader searches there for records to
clear. Boot head, OTA layout and loader compatibility remain board contracts;
an application's region list must respect them.

## API and errors

| Function | Behavior |
| --- | --- |
| `fl_jedec()` | JEDEC ID; `0` if access is refused or the XIP mode is unsupported |
| `fl_status()` | SR1 and SR2; `UINT32_MAX` on refusal/unsupported mode |
| `fl_uid(dst)` | 16 UID bytes into RAM; enabled by defining `FL_UID` |
| `fl_read(off, dst, n)` | Raw physical bytes, including the boot head; 256-byte interrupt-masked chunks |
| `fl_write(off, src, n)` | Programs page-sized chunks; does not erase |
| `fl_erase4k(off, &took_us)` | Erases one aligned 4 KiB sector; timing destination must be in RAM |
| `fl_plain_window_init()` | Configures unencrypted XIP data reads and invalidates cached data; returns status |
| `fl_verify_xip(off, ref, n)` | Compares a mapped plain-data range against RAM; `0` on match, negative mismatch position otherwise |

Integer operations return `0` on success. Errors are `FL_ERR_RANGE` (-1),
`FL_ERR_MODE` (-2), `FL_ERR_WEL` (-3), `FL_ERR_TIMEOUT` (-4), `FL_ERR_BUSY` (-5),
and `FL_ERR_BUFFER` (-6). Verification's historical negative mismatch result
shares the error-code space; use it as a success/failure check.

The complete read/write range and buffer are checked before the first chunk.
Invalid ranges, arithmetic wraparound and buffers outside on-chip RAM are
rejected. A zero-length read/write is a no-op if its range and buffer are valid;
no flash transaction occurs. A later hardware failure or CPU1 job can still stop
a multi-page operation after earlier pages changed: these operations are not
atomic storage transactions. Applications retain responsibility for journals,
checksums and power-loss recovery of their own data.

Every transaction saves and restores the caller's full ICFG value, including
when interrupts were already disabled. `fl_plain_window_init()` follows the same
rule. Wrappers release the mask between chunks, allowing input/USB service.
Call these blocking APIs from the main loop, after arranging audio silence where
necessary; sector erase can take hundreds of milliseconds.

## CPU1 and RAM-only execution

The driver rejects access before changing SFC or flash pins unless:

- The caller is CPU0 with global interrupts masked.
- CPU1 is held/off, or the BSP worker has acknowledged its RAM-only idle state.

`fm1_cpu1_run()` revokes idle permission before publishing a job. The RAM worker
publishes idle after the job's final barrier and before publishing completion,
so a following dispatch cannot be overwritten by a late idle acknowledgement.
A busy or unknown worker causes `FL_ERR_BUSY`; the driver neither aborts a job
nor waits indefinitely. Arrange a job-free interval, then retry. The loader
holds CPU1 at entry, including when entered directly through SPL/ROM.

The BSP uses a single compilation unit. Include the flash driver, CPU1 worker
and stop helper in that same unit so they share their access state. Custom CPU1
workers are refused while running; they need an equivalent RAM-only idle
protocol before integration. The BSP worker leaves its interrupt bank disabled;
custom interrupts or DMA that fetch from flash must also be quiescent. Faults,
undocumented silicon behavior and failed flash hardware still require hardware
qualification; the software protocol does not establish those guarantees.

Low-level `fl_*_ram` entry points remain for the RAM loader and established
callers. Their arguments/results must be in RAM, CPU0 IRQs must already be
masked, and calls from XIP must use `FL_FAR`. They apply the same CPU1 gate and
range/buffer checks as the wrappers. Never bypass the gate by calling `fl_enter`
or manipulating its state from application code.

## Replacement flash parts

`FM1_FLASH_SIZE` defaults to 1 MiB. For a physically verified replacement, set
its capacity (at most 16 MiB for the driver's three-byte addresses) and explicitly
grant its extra region with `FM1_FLASH_WRITE_REGIONS`. For example:

```c
#define FM1_FLASH_SIZE 0x400000u
#define FM1_FLASH_WRITE_REGIONS(X) X(0x100000u, 0x400000u)
#include "fm1_flash.h"
```

`fm1_flash_size_from_jedec()` in `fm1_flash_ext.h` can check the detected capacity
against the build's configuration before permitting writes. Discovery does not
automatically widen permissions. Vendor quad-enable handling and replacement
parts remain unqualified; this does not add four-byte addressing.

Physical SPI capacity and mapped XIP capacity are separate. `FM1_FLASH_XIP_SIZE`
defaults to 1 MiB even on larger parts. Raw reads/programs work within the
configured physical capacity; cache invalidation is clipped to the declared
mapping. Do not increase the XIP limit without measuring the hardware mapping.
`FM1_FLASH_PLAIN_LO` and `FM1_FLASH_PLAIN_HI` select the unencrypted data window
(default `0x93000..0x100000`); they must exclude executable/encrypted content.
XIP verification is restricted to that declared window.

## Verification

```sh
tests/run_flash_tests.sh
JIELI_TOOLCHAIN=/path/to/toolchain python3 tests/check_flash_target.py
```

Host tests exercise default/custom/read-only/loader/expanded policies, whole-range
rejection, page splitting, buffer bounds, nested interrupt restoration, busy CPU1
refusal, and error exits. The target check builds/links the safe and CPU1 examples,
fallback mode, an expanded-flash configuration with UID, and the loader. It
checks flash RAM routines for calls and direct branches into XIP and confirms
that the loader lives in RAM. These checks do not replace on-device erase/read/
write, interrupt-state and simultaneous audio/CPU1 validation.
