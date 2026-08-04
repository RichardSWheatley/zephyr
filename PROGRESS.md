# PROGRESS — Renode platform for Apollo510 EVB + MVE validation

Working branch: `claude/apollo510-renode-platform-tfnb7q` (RichardSWheatley/zephyr).
Handoff: "Renode platform for Apollo510 EVB + zscilib MVE validation" (owner richard.wheatley@ambiq.com).

## T0 — Environment probe (DONE)

| Tool | Version |
|---|---|
| Renode | **v1.16.1.16908** (portable, build d66b0c2a-202602160923) — gate ≥ 1.16.1 PASSES; 1.16.1 is the latest release (1.16.2/1.17 do not exist) |
| Zephyr tree | v4.4.99 development tree (this branch) |
| Zephyr SDK | 1.0.1 (arm-zephyr-eabi GCC 14.3.0); hosttools QEMU 
| west | 1.5.0 (python 3.12 venv) |
| hal_ambiq | this branch @ bea5e88 ("Merge pull request #22 … apollo510-i2s-fd"); note west.yml pins 3d7044ea — the branch checkout is intentionally used as the source of truth |
| cmsis_6 | b2dfbe1a (== west.yml pin) |

`west list` module layout used (no `west update` — network-restricted environment):
`modules/hal/ambiq` → local hal_ambiq checkout, `modules/hal/cmsis_6` → shallow clone at the pinned revision. All other manifest projects absent ⇒ skipped by the build; none are needed for `apollo510_evb/apollo510` or `mps3/corstone300/an547`.

## Deviations from the handoff document (per §3 "sources of truth")

1. **§0 was never filled in**: no zscilib MVE RFC link and no patch file exist in this environment. T5b therefore runs the **unpatched** upstream zscilib suite on both platforms as the QEMU↔Renode parity baseline; the MVE-patched rerun is recorded in BLOCKERS.md pending the actual patch.
2. **`ambiq-stimer-fixes` branch does not exist** on RichardSWheatley/zephyr (checked `git ls-remote`). The in-tree `drivers/timer/ambiq_stimer.c` already contains the 32-bit wrap-extension logic (`update_tick_counter()`, ambiq_stimer.c:69-88), so T2 validates against this branch's driver.
3. **Board yaml filename**: handoff §4 names `apollo510_evb_apollo510.yaml`; the live tree uses `apollo510_evb.yaml` — the tree wins.
4. **UART driver**: handoff §1 says Zephyr drives the Ambiq UART via `uart_pl011.c`; this tree uses the dedicated `drivers/serial/uart_ambiq.c` (same PL011-derivative register contract, via the Ambiq HAL). Renode's stock `UART.PL011` model covers it, including the vendor CLKEN/CLKSEL bits in CR[6:3] which the model stores as read/write "Vendor specific" flags.

## T1 — Skeleton platform (DONE)

Deliverables in `boards/ambiq/apollo510_evb/`:
- `support/apollo510_evb.repl` — cortex-m55 + NVIC; ITCM/DTCM/SSRAM/MRAM/INFO1 memories; PL011 uart0 @ 0x40039000 IRQ 15; STIMER model @ 0x40008800 IRQ 32-40; PWRCTRL model @ 0x40021000; CLKGEN/TIMER/OTP+TRNG/MCUCTRL/WDT/GPIO/SECURITY as reads-as-written scratch RAM.
- `support/apollo510_evb.resc` — ELF load, `EnableZephyrMode`, VTOR from `_vector_table`, plus the 4-word pre-seed making every HAL boot spin-wait fall through (DEVPWREN UARTs+DBG, MEMPWREN 0x3F, SSRAMPWREN 0x7, CLKGEN CLOCKENSTAT HFRC2READY).
- `support/AmbiqApollo5_SystemTimer.cs` — adapted from Renode's MIT `AmbiqApollo4_SystemTimer` (Apollo510 STIMER map verified register-identical at all driver-touched offsets). Changes: true XTAL frequencies (32768/16384/1024 Hz vs the stock model's rounded 32000 — removes a 2.4% skew vs `CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=32768`); monitor-settable `CounterValue` for wrap-preload testing; 0x58 HALSTATES scratch; 0x5C serves the **RSTGEN→STAT alias** (RSTGEN base 0x40000000 + offset 0x885C lands inside the STIMER page).
- `support/AmbiqApollo5_PowerController.cs` — EN→STATUS mirroring (identity for DEVPWREN/AUDSSPWREN/SSRAMPWREN; remapped bits for MEMPWREN per apollo510.h), MCUPERFREQ immediate-ack, VRSTATUS = all-ACT (0x3F), everything else reads-as-written. Mirroring (not static seeding) is required because `CONFIG_PM_DEVICE_RUNTIME=y` polls DEVPWRSTATUS for *clears* on the UART suspend path.
- `board.cmake` — `SUPPORTED_EMU_PLATFORMS renode`, `RENODE_SCRIPT`, `RENODE_UART sysbus.uart0`, sim/robot runners (jlink kept).
- `apollo510_evb.yaml` — `simulation: renode` + `testing.renode` entries (schema copied from live boards: hifive1, stm32h7_renode_reference_board).

Key modelling decisions (sources cited in the model files):
- **SCMPR write = delta** latched against the live counter, readback = absolute target — confirmed from `am_hal_stimer_compare_delta_set()` (hal am_hal_stimer.c:415-494) *and* the CMSIS SCMPR0 register description; the model keeps the stock +3-cycle write-latency compensation matching the HAL's `delta -= 3`.
- **MCUCTRL reads all-zero on purpose**: CHIPREV==0 makes every APOLLO5_A0/B0/B1/B2 macro false ⇒ `am_hal_spotmgr_init` leaves its dispatch table NULL ⇒ all SPOT-manager hooks no-op (skips VRSTATUS/trim polling); SHADOWVALID.INFO1SELOTP==0 keeps OTP unpowered and routes INFO1 reads to plain loads from the zero-filled 0x42000000 window (trim version 0 = simplest HAL paths).

**Accept check PASSED**: `west build -b apollo510_evb/apollo510 samples/hello_world` then Renode:
```
uart0: *** Booting Zephyr OS build 746eb4060b3e ***
uart0: Hello World! apollo510_evb/apollo510
```
Banner at 0.4 ms virtual time; PM idle path (`am_hal_sysctrl_sleep` → WFI) runs clean for 3+ virtual seconds; no boot hangs, no debug-loop iterations needed.

## T2 — STIMER validation (twister timer+sleep + wrap run) — DONE, green

`west twister -p apollo510_evb/apollo510 --simulation renode -T tests/kernel/timer -T tests/kernel/sleep`:

| Scenario | Result |
|---|---|
| kernel.common.timing | passed |
| kernel.common.timing.minimallibc | passed |
| kernel.timer (timer_api, 18 cases) | **passed** (after the two findings below) |
| kernel.timer.error_case | passed |
| kernel.timer.starve | passed |
| kernel.timer.timeout_churn | passed |
| kernel.timer.timer_observer | passed |
| kernel.timer.starve.soak / kernel.timer.timer (timer_behavior) | not run (slow/build-only filtered by twister defaults) |

**Finding 1 — upstream Renode model defect (fixed in our vendored model).** The stock
`AmbiqApollo4_SystemTimer` models the 3-cycle COMPARE write latency with
`systemTimer.Increment(3)`, which advances the base counter but not the per-comparator
`ComparingTimer`s — the two clocks drift 3 cycles apart on **every** SCMPR write.
In re-arm-heavy workloads this accumulates: `tests/kernel/timer/timeout_churn`'s
`test_timeout_churn_near_tick_arms` observed a near-boundary timeout firing **14 ms late**
(arm #68). Our `AmbiqApollo5_SystemTimer` instead pushes the compare *target* 3 cycles past
the current counter (`CompareValue = Value + delta + 3`), modelling the same hardware
latency without splitting the clocks. timeout_churn passes with the fix. Worth upstreaming
to renode-infrastructure for the Apollo4 model too.

**Finding 2 — latent margin bug in tests/kernel/timer/timer_api (fixed on this branch,
upstreamable).** `test_timer_remaining` bounds `k_timer_remaining_get()` by
`DURATION/2 + ceil_ms(1 tick)`, but the legitimate worst case is **two** ticks of error
(+1 partial-tick alignment in `z_add_timeout()`, +1 from the ceil conversions — remaining_get
is `k_ticks_to_ms_ceil32`). Real hardware hides this behind busy-wait overhead; a cycle-exact
simulator does not. Measured on Renode (probe app): 100 ms timer → expiry tick 104
(= ceil(102.4)+1), exact 50 ms busy-wait → 51 elapsed ticks (dcyc=1638), remaining 53 ticks
→ reported 52 ms vs bound 51 ms. Fix: bound widened to `ceil_ms(2 ticks)` with a full
comment (tests/kernel/timer/timer_api/src/main.c). Not a tolerance-loosening to mask
simulator error — the arithmetic overshoot is exact and platform-independent.

**Wrap-extension (F-16) validation — PASSED.** `sysbus.stimer CounterValue 0xFFFEFFFF`
preloads STTMR 65,537 cycles (2.0 s) before the 32-bit wrap; timer_api then runs 3.88
virtual seconds (~127k cycles), so the wrap occurs mid-suite. Result: `SUITE PASS - 100.00%
[timer_api]: pass = 18, fail = 0`, final STTMR read back 0x000DFFF2 (wrapped). The driver's
`update_tick_counter()` wrap extension handled it correctly. This is repeatable on demand —
something the physical EVB cannot do (a 32768 Hz counter takes ~36 h to wrap naturally).

twister.json archives: `PROGRESS-artifacts/twister-t2.json` (first run, pre-fix, shows the
two findings failing) and `PROGRESS-artifacts/twister-t2-fix.json` (kernel.timer green).

## T3 — Scoped kernel set

_pending_

## T5a — MVE smoke — DONE (QEMU PASS; Renode FAIL → contingency invoked)

App: `tests/renode-mve-smoke/` — vld1q/vaddq/vst1q int32, vfmaq float32, VCTP
tail-predicated loop over 13 elements with an overwrite canary, explicit DLS/LE
low-overhead-branch loop; every stage asserts exact results; compiler barriers defeat
constant folding so the vector instructions execute at runtime. (Two portability notes,
commented in the source: GCC 14's pragma-generated MVE intrinsics use the compiler-native
'long' as int32 — incompatible with Zephyr's `zephyr_stdint.h` remap, hence the
`mve_s32_t` lane typedef; `vldrwq/vstrwq` are declared with the header-level int32_t and
need casts the other way.)

| Platform | Result |
|---|---|
| QEMU `mps3/corstone300/an547` (full-MVE build) | **MVE smoke: PASS** |
| Renode `apollo510_evb/apollo510` | **FAIL — two distinct Renode 1.16.1 MVE defects, fully characterized in BLOCKERS.md B1** |

Renode findings (details + reproducers in BLOCKERS.md): (1) spurious BusFault on the MVE
scalar 64-bit shift LSLL (0xEA50118F) that GCC emits in the Ambiq HAL's baud-rate math —
any CONFIG_FPU=y image dies at console init; (2) MVE vector instructions inside DLS/LE
low-overhead loops execute only their first beat (silent lane corruption), while the same
instructions in straight-line code — including VCTP predication — execute correctly.
Per handoff §T5a, QEMU is the sole MVE platform for T5b.

## T5b — zscilib parity baseline — DONE (adapted per BLOCKERS B1/B2)

zscilib @ upstream master bf1cbf1 ("general: update zephyr libraries"), **unpatched**
(no MVE patch exists — B2), built as an extra module with `CONFIG_ZSL_SINGLE_PRECISION=y`
`CONFIG_ZSL_PLATFORM_OPT=0`. The suite builds cleanly on this 4.4.99 tree (the handoff's
"may not build on current Zephyr" caveat did not materialize).

| Platform | Build | Result |
|---|---|---|
| QEMU `mps3/corstone300/an547` (full MVE-capable flags) | ok | **SUITE PASS 100% — 332/332** (vector/matrix/fusion/orientation all included in zsl_tests) |
| Renode `apollo510_evb/apollo510` (`EXTRA_CFLAGS=-mcpu=cortex-m55+nomve` to dodge B1.1; scalar FPv5) | ok | **SUITE PASS 100% — 332/332** |

**Per-test parity: IDENTICAL** — the sorted per-case PASS/FAIL lists from both platforms
diff clean (669 result lines each). Zero numeric divergence at assertion level between
QEMU's and Renode's FP arithmetic on these suites. Logs:
`PROGRESS-artifacts/zscilib-qemu-an547.log`, `PROGRESS-artifacts/zscilib-renode-apollo510.log`.

Still blocked (B2): the MVE-patched rerun — requires the actual §0 patch; and any
MVE-accelerated zscilib path on Renode — requires the B1 Renode fixes.

## T6 — Upstream prep notes

- In-tree precedent for partial-SoC Renode board support files: `boards/antmicro/stm32h7_renode_reference_board` (Cortex-M7, own .repl+.resc in support/), `boards/renode/riscv32_virtual`. Maintainer sign-off on the layout remains an open item for the PR thread (no interactive channel from this environment).
- hal_ambiq required **no changes** for this work.
