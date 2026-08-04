# BLOCKERS — Renode platform for Apollo510 EVB + MVE validation

## B1 — Renode 1.16.1 MVE defects (T5a contingency invoked: QEMU is the sole MVE platform)

Per handoff §T5a: Renode faults/misexecutes MVE instructions; exact instructions and PCs
recorded below. MVE validation (T5b) proceeds on QEMU `mps3/corstone300/an547` only. The
apollo510_evb Renode platform itself is unaffected (kernel suites run with `CONFIG_FPU=n`
builds, which compile `+nofp+nomve`).

### B1.1 — Spurious BusFault on MVE scalar 64-bit shift (LSLL)

* Instruction: `lsll r0, r1, #6` — MVE scalar 64-bit shift, encoding `0xEA50118F`
  (binutils 2.45 objdump misdecodes it as `orrs.w r1, r0, pc, lsl #6`; encoding verified by
  assembling `lsll r0, r1, #6` with `-march=armv8.1-m.main+mve`).
* PC: `0x419F32`, in `config_baudrate()` (`hal_ambiq mcu/apollo510/hal/mcu/am_hal_uart.c:983`)
  — GCC 14.3 emits LSLL for the 64-bit baud-rate arithmetic whenever the tree is compiled
  with MVE available.
* Symptom: execution enters `z_arm_bus_fault` (execution trace: PC 0x419F32 →
  z_arm_bus_fault → z_arm_fault → k_sys_fatal_error_handler). No UART output — the fault
  happens during console init.
* Consequence: **any** `CONFIG_FPU=y` Zephyr image for apollo510_evb (which enables MVE
  codegen tree-wide via `ARMV8_1_M_MVEI/MVEF`) dies in the boot path under Renode 1.16.1.
* Workaround used for characterization: build the tree with `-mcpu=cortex-m55+nomve`
  (keeps FPv5) and restore full MVE only for the file under test — see
  `tests/renode-mve-smoke/CMakeLists.txt` (`SMOKE_TREE_NOMVE=y`).

### B1.2 — MVE vector instructions inside DLS/LE loops execute only the first beat (silent data corruption)

* Minimal reproducer (deterministic): a 4-iteration `vld1q_s32/vaddq_s32/vst1q_s32` loop
  compiled at `-Os` so GCC keeps a `dls lr, lr` / `le lr, <loop>` low-overhead loop with
  `VLDRW.U32` / `VADD.I32` / `VSTRW.32` in the body (apollo510_evb build, main loop at
  `main+0x40..0x62`; encodings `ed92 7f00`, `ed92 5f00`, `ef26 6844`, `ed82 7f00`).
* Expected dst: `101 202 303 ... 1616`; observed:
  `101 0 0 0 505 0 0 0 909 0 0 0 33 0 0 0` — only lane 0 of each iteration is stored
  (and the final iteration's lane 0 is additionally truncated: 33 = 1313 & 0xFF).
* Control experiments (all PASS on the same Renode build): identical vld1q/vaddq/vst1q in
  straight-line (unrolled) code; VDUP+VSTRW all-lane store; VLDRW all-lane load;
  `vctp32q`-predicated load/add/store in straight-line code; explicit `dls/le` loop with
  scalar-only body. The defect is specifically the MVE-in-LOB-loop combination — consistent
  with (but more severe than) Antmicro's "LOB support is rudimentary" note for 1.16.1:
  it is silent corruption, not a fault.
* Since real MVE workloads (zscilib at -O2) are dominated by auto-generated DLS/LETP loops,
  Renode 1.16.1 cannot run them faithfully. → QEMU-only for T5b.
* Upstream-report material: both reproducers are tiny Zephyr apps; B1.1 additionally
  reproduces with nothing but a `CONFIG_FPU=y` hello_world on apollo510_evb.

## B2 — zscilib MVE patch unavailable (handoff §0 never filled in)

The handoff's zscilib MVE RFC link and patch file fields are placeholders; no patch file
exists in this environment. T5b therefore runs the **unpatched** upstream zscilib test suite
(with `CONFIG_ZSL_SINGLE_PRECISION=y`) as the QEMU↔(Renode) parity baseline. The
MVE-patched rerun remains blocked until the actual patch is provided. Symptom: n/a (missing
input, not a failure). What was NOT verified: zscilib's MVE-accelerated code paths — only
its portable C paths.

## B3 — `ambiq-stimer-fixes` branch does not exist on the fork remotes

Handoff §T2 asks to validate against branch `ambiq-stimer-fixes` (F-15/16/17);
`git ls-remote` shows no such branch on RichardSWheatley/zephyr or either hal_ambiq remote.
Validation was performed against this branch's in-tree `drivers/timer/ambiq_stimer.c`,
which already contains the 32-bit wrap-extension logic (`update_tick_counter()`); the wrap
path was explicitly exercised via counter preload (see PROGRESS.md T2). What was NOT
verified: any F-15/F-17 changes that exist only on the missing branch.
