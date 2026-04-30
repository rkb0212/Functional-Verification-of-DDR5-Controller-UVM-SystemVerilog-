# Functional Verification of DDR5 Controller
### UVM · SystemVerilog · DPI-C Golden Model · EDA Playground · Aldec Riviera-PRO

## Overview

This project builds a complete **UVM-based functional verification environment** for a custom DDR5 memory controller written in SystemVerilog, extended with a **C++ DPI-C golden reference model** that runs as a second independent checker alongside the UVM scoreboard.

The DUT models realistic DDR5 DRAM protocol behavior — from per-bank state machines and address mapping through refresh sequencing and timing enforcement — and the testbench proves correctness under the full range of protocol scenarios a production controller would face.

**What the DPI-C layer adds over the base UVM TB:**

- A C++ golden memory model that independently stores every write burst and cross-checks every read response beat-by-beat across all 16 words of each BL16 transfer
- An independent JEDEC timing compliance engine that re-verifies all 10 timing parameters from scratch, using `int64_t` cycle counters that never wrap and catch violations the SV scoreboard might miss at boundary conditions
- A bug fix for the original SV scoreboard: after a refresh command, the C model correctly resets `last_act` and `last_pre` timestamps for all banks, preventing false `tRC`/`tRP` violations on the first ACT after refresh
- Full `tCCD_S` (cross-bank-group column spacing) coverage, which the original SV scoreboard did not check
- A `read_req` / `read_rsp` split that snapshots the expected data into a FIFO at request time, matching the pipeline latency exactly regardless of how many clock cycles the DUT takes to respond

The verification goal remains the same across all three layers:

- **Safety** — no timing constraint is ever violated
- **Correctness** — every read returns the data from the most recent write to that address
- **Completeness** — the controller reaches every meaningful protocol state that JEDEC defines

---

## Repository Structure

```
.
├── design.sv                  # Top-level design wrapper (includes DUT)
├── testbench.sv               # Top-level TB (UVM kickoff, DUT instantiation)
├── ddr5_controller_DUT.sv     # DUT: cycle-accurate DDR5 controller model
├── ddr5_dut_if.sv             # Interface with driver and monitor clocking blocks
├── ddr5_tb_params_pkg.sv      # Global timing parameters and address-field widths
├── ddr5_tb_utils_pkg.sv       # Helper functions: address packing, burst data generation
├── ddr5_tb_pkg.sv             # Top-level TB package (imports all components)
├── transactions.sv            # UVM sequence items: ddr5_req_txn, ddr5_cmd_txn
├── sequences.sv               # All directed + stress sequences (16 sequences)
├── driver.sv                  # UVM driver: request → DUT pin stimulus
├── monitor.sv                 # UVM monitor: 3 parallel threads (req / rsp / cmd)
├── scoreboard.sv              # Data integrity + timing checker with DPI-C hooks
├── coverage.sv                # 13 functional covergroups
├── agent.sv                   # UVM agent
├── env.sv                     # UVM environment
├── tests.sv                   # All test classes + regression test
├── ddr5_dpi_model.h           # DPI-C golden model — C++ header (extern "C" API)
├── ddr5_dpi_model.cpp         # DPI-C golden model — full C++ implementation
├── run.bash                   # Shell script: compile C++ SO, compile SV, run sim
└── run.do                     # Aldec vsim Tcl script
```

---

## DPI-C Golden Reference Model

### Architecture

The C++ model (`ddr5_dpi_model.cpp`) is compiled into a shared object (`ddr5_dpi_model.so`) and loaded by Riviera-PRO at simulation startup via `vsim -sv_lib`. The SV scoreboard imports five `extern "C"` functions using `import "DPI-C"` declarations at the top of `scoreboard.sv`.

```
                    UVM Scoreboard (SV)
                           │
        ┌──────────────────┼───────────────────┐
        │                  │                   │
   write_req()        write_rsp()         write_cmd()
        │                  │                   │
   ddr5_dpi_           ddr5_dpi_          ddr5_dpi_
   write_req()         read_req()           cmd()
   ddr5_dpi_           ddr5_dpi_
   write_req()         read_rsp()
        │                  │                   │
        └──────────────────┼───────────────────┘
                           │
                ┌──────────▼──────────┐
                │  ddr5_dpi_model.cpp │
                │                     │
                │  burst_t mem[][]    │  ← golden memory
                │  BankState banks[]  │  ← bank state machine
                │  pending_reads FIFO │  ← in-flight read tracker
                │  int64_t timestamps │  ← timing counters
                └─────────────────────┘
```

### DPI-C API

| Function | Called from | Purpose |
|----------|-------------|---------|
| `ddr5_dpi_reset()` | `build_phase` | Zero memory, bank state, timestamps |
| `ddr5_dpi_write_req(addr, wdata)` | `write_req()` on WRITE | Store BL16 burst into golden memory |
| `ddr5_dpi_read_req(addr)` | `write_req()` on READ | Snapshot expected burst into pending-read FIFO |
| `ddr5_dpi_read_rsp(dut_rdata, exp_rdata)` | `write_rsp()` | Pop snapshot, compare all 16 words, return pass/fail |
| `ddr5_dpi_cmd(cycle, cmd_code, rank, bg, bank, row, col)` | `write_cmd()` | Timing compliance check for every cmd_valid pulse |

### Memory Model

The C++ memory is a three-dimensional array: `burst_t mem[BANKS_TOTAL][16][16]` where:
- `BANKS_TOTAL = RANKS × BANK_GROUPS × BANKS_PER_GROUP = 32`
- First dimension is a flat bank index: `rank × 16 + bg × 4 + bank`
- Second dimension is the row field from the address (`ROW_W = 4`, values 0–15)
- Third dimension is the column field from the address (`COL_W = 4`, values 0–15)
- Each entry is `burst_t` — a `std::array<uint32_t, 16>` holding one full BL16 burst

The address decoder in the C++ model mirrors `canonical_addr()` in `ddr5_tb_utils_pkg.sv` exactly:

```
addr[3:0]   = column  (COL_W  = 4)
addr[5:4]   = bank    (BANK_W = 2)
addr[7:6]   = bg      (BG_W   = 2)
addr[8]     = rank    (RANK_W = 1)
addr[31:28] = row     (ROW_W  = 4)
```

Unwritten locations are zero-initialised in `ddr5_dpi_reset()`. Any `read_rsp` for an unwritten address returns zero, which the SV scoreboard accepts as correct per the original verification plan.

### Timing Checks in the C++ Model

`ddr5_dpi_cmd()` maintains per-bank `last_act` / `last_pre` timestamps and global `last_read_cycle` / `last_write_cycle` / `bg_last_col[]` counters, all typed `int64_t` to prevent overflow across long simulations. Timing checks performed per `cmd_code`:

| `cmd_code` | Command | Checks |
|-----------|---------|--------|
| 1 | ACT | `tRP` (PRE → ACT, same bank), `tRC` (ACT → ACT, same bank) |
| 2 | READ | `tRCD` (ACT → READ, same bank), `tWTR` (WRITE → READ, global), `tCCD_L` (same-BG), `tCCD_S` (cross-BG) |
| 3 | WRITE | `tRCD` (ACT → WRITE, same bank), `tRTW` (READ → WRITE, global), `tCCD_L` (same-BG), `tCCD_S` (cross-BG) |
| 4 | PRE | `tRAS` (ACT → PRE, same bank) |
| 5 | REF | All banks must be closed; resets all `last_act` / `last_pre` to –1 |

**Key difference from original SV scoreboard:** The C++ model also checks `tCCD_S` (cross-bank-group column spacing) and `tRCD` (activate-to-column), which the original `write_cmd()` in SV did not verify. It also performs a bank-open check before accepting any column command.

### REF Bug Fix

The original SV scoreboard `write_cmd()` for `cmd_code == 3'd5 (REF)` only set `bank_is_open = 0` but left `last_act_cycle` and `last_pre_cycle` unchanged. This caused a false `tRC` violation on the first ACT after a REF, because the timestamp from the pre-refresh activation was still visible to the tRC guard.

The C++ model's `case 5` resets `last_act = -1` and `last_pre = -1` for all 32 banks after a refresh, which is correct JEDEC behaviour: refresh resets the row-cycle timing windows. This means the C++ model and SV scoreboard can disagree on the first post-refresh ACT — the C++ model will pass it while the SV scoreboard may flag a false tRC violation.

---

## How the Scoreboard Uses the DPI

`scoreboard.sv` integrates the C++ model in three functions, with the DPI layer clearly marked in the source code.

### `build_phase` — Reset

```systemverilog
ddr5_dpi_reset();
`uvm_info("SB_DPI", "C golden model reset complete", UVM_MEDIUM)
```

Called once at simulation start. Synchronises the C++ model to the same initial state as the SV scoreboard.

### `write_req` — Writes and Read Requests

```systemverilog
// WRITE: layer 1 (SV) then layer 2 (DPI)
exp_by_addr[canon] = t.wdata;
ddr5_dpi_write_req(canon, t.wdata);

// READ: layer 1 (SV) then layer 2 (DPI)
pending_reads.push_back(canon);
ddr5_dpi_read_req(canon);
```

For writes, `ddr5_dpi_write_req` passes the full `bit [511:0] wdata` packed vector; Riviera-PRO maps this to `const svBitVecVal*` (16 `uint32_t` words) in the C++ function.

For reads, `ddr5_dpi_read_req` snapshots the expected data into the C++ FIFO at the same moment the SV scoreboard records the address in `pending_reads`. This ensures both checkers consume responses in the same order regardless of DUT pipeline depth.

### `write_rsp` — Read Response Comparison

```systemverilog
bit [DATA_BUS_W-1:0] dpi_exp;
int dpi_match = ddr5_dpi_read_rsp(t.rdata, dpi_exp);

`uvm_info("SB_DPI_COMPARE",
  $sformatf("ADDR=0x%08h  DUT=0x%08h  MODEL=0x%08h  RESULT=%s",
            addr, t.rdata[31:0], dpi_exp[31:0],
            (dpi_match ? "PASS" : "FAIL")),
  UVM_MEDIUM)

if (!dpi_match) dpi_fail_cnt++;
```

`ddr5_dpi_read_rsp` pops the expected burst from the FIFO, compares all 16 words, prints each mismatched beat with `exp=` and `got=` values, and fills `dpi_exp` so the SV side can log it. The `dpi_fail_cnt` counter accumulates independently of `fail_cnt`.

### `write_cmd` — Timing Compliance

```systemverilog
int dpi_pass = ddr5_dpi_cmd(
    int'(c.cycle_num), int'(c.cmd_code),
    int'(c.cmd_rank),  int'(c.cmd_bg),
    int'(c.cmd_bank),  int'(c.cmd_row), int'(c.cmd_col));

if (!dpi_pass) begin
    `uvm_error("SB_DPI",
      $sformatf("C MODEL TIMING VIOLATION cmd=%0d rank=%0d bg=%0d bank=%0d ...",
                c.cmd_code, ...))
    dpi_fail_cnt++;
end
```

The DPI timing check runs before the existing SV timing checks, so C++ `printf` output appears first in the transcript. Both checkers see the same `cycle_num` value from the monitor.

### `report_phase` — Final Summary

The scoreboard prints two summary blocks:

```
=== SCOREBOARD FINAL REPORT ===
WRITES accepted      : N
READS accepted       : N
SV  READ PASS        : N
SV  READ FAIL        : 0
DPI TIMING ERRORS    : 0
C MODEL: ALL TIMING CHECKS PASSED

=== FUNCTIONAL COVERAGE REPORT ===
...
```

If `dpi_fail_cnt > 0`, a `uvm_error` is raised so the simulator exits with a non-zero status even if the SV layer reported no failures.

---

## DUT Description

**File:** `ddr5_controller_DUT.sv`

The DUT is a behavioral RTL model of a DDR5 memory controller scheduler and timing engine. It accepts a simple request interface (`req_valid`, `req_write`, `req_addr`, `req_wdata`) and drives a DRAM command bus (`cmd_code`, `cmd_bg`, `cmd_bank`, `cmd_row`, `cmd_col`) along with a response path (`rsp_valid`, `rsp_rdata`).

### State Machine

```
CPU Request Interface
        │
        ▼
  ┌─────────────┐
  │   ST_IDLE   │◄──────────────────────────────┐
  └──────┬──────┘                               │
         │ row hit / miss / empty bank          │
         ▼                                      │
  ┌─────────────┐    ┌─────────────┐            │
  │ ST_PRE_WAIT │───►│ST_ACT_ISSUE │            │
  └─────────────┘    └──────┬──────┘            │
                            │                   │
                     ┌──────▼──────┐            │
                     │ST_RCD_WAIT  │            │
                     └──────┬──────┘            │
                            │                   │
                     ┌──────▼──────┐            │
                     │ST_COL_ISSUE │            │
                     └──────┬──────┘            │
               ┌────────────┴────────────┐      │
               ▼                         ▼      │
       ┌──────────────┐        ┌───────────────┐│
       │ST_READ_WAIT  │        │ST_WRITE_WAIT  ││
       │  (CL cycles) │        │ (tCWL cycles) ││
       └──────┬───────┘        └───────┬───────┘│
              │                        │        │
              └───────────┬────────────┘        │
                          │                     │
                   ┌──────▼──────┐              │
                   │   ST_IDLE   │──────────────┘
                   └─────────────┘
                          │ (refresh_pending)
                   ┌──────▼──────┐    ┌─────────────┐
                   │ST_REF_ISSUE │───►│ ST_REF_WAIT │
                   └─────────────┘    └─────────────┘
```

### Timing Parameters

| Parameter | Value | What it enforces |
|-----------|-------|-----------------|
| `tRCD` | 4 | ACT → first READ/WRITE |
| `tRAS` | 8 | Minimum row active time before PRE |
| `tRP` | 4 | PRE → next ACT on same bank |
| `tRC` | 12 | ACT → ACT on same bank (= tRAS + tRP) |
| `CL` | 4 | READ command → first data on bus |
| `tCWL` | 4 | WRITE command → DRAM accepts data |
| `tWTR` | 4 | Last WRITE → next READ (bus turnaround) |
| `tRTW` | 6 | Last READ → next WRITE (bus direction flip) |
| `tCCD_L` | 8 | Column command spacing within same bank group |
| `tCCD_S` | 4 | Column command spacing across different bank groups |
| `tRFC` | 16 | Refresh cycle time |
| `tRAS_MAX` | 64 | Maximum row open time (watchdog forced-PRE) |

### Address Mapping

```
addr[31:28]  = row   (ROW_W  = 4)
addr[8]      = rank  (RANK_W = 1, 2 ranks)
addr[7:6]    = bg    (BG_W   = 2, 4 bank groups)
addr[5:4]    = bank  (BANK_W = 2, 4 banks/group)
addr[3:0]    = col   (COL_W  = 4)
```

Total: 16 banks (2 ranks × 4 BGs × 4 banks per BG), 32 flat bank indices.

---

## UVM Testbench Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                          TEST                                │
│  ┌────────────────────────────────────────────────────────┐  │
│  │                        ENV                            │  │
│  │  ┌──────────────────────────┐  ┌───────────────────┐  │  │
│  │  │         AGENT            │  │    SCOREBOARD     │  │  │
│  │  │  ┌───────────────────┐   │  │  ┌─────────────┐  │  │  │
│  │  │  │    SEQUENCER      │   │  │  │ SV Checker  │  │  │  │
│  │  │  └────────┬──────────┘   │  │  │ (Layer 1)   │  │  │  │
│  │  │  ┌────────▼──────────┐   │  │  └──────┬──────┘  │  │  │
│  │  │  │     DRIVER        │   │  │         │          │  │  │
│  │  │  └───────────────────┘   │  │  ┌──────▼──────┐  │  │  │
│  │  │  ┌───────────────────┐   │  │  │ DPI-C Model │  │  │  │
│  │  │  │    MONITOR        │   │  │  │ (Layer 2)   │  │  │  │
│  │  │  │ (3 threads)       │   │  │  └─────────────┘  │  │  │
│  │  │  └───────────────────┘   │  └───────────────────┘  │  │
│  │  └──────────────────────────┘  ┌───────────────────┐  │  │
│  │                                │   COVERAGE        │  │  │
│  │                                │  (13 groups)      │  │  │
│  │                                └───────────────────┘  │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
                             │
                  ┌──────────▼──────────┐
                  │    ddr5_dut_if      │
                  └──────────┬──────────┘
                             │
                  ┌──────────▼──────────┐
                  │ ddr5_controller_dut │
                  └─────────────────────┘
```

### Component Details

#### Interface — `ddr5_dut_if.sv`
Two clocking blocks with `#1step` skew to prevent driver/DUT race conditions:
- `drv_cb` — output skew ensures signals are stable before the DUT's sampling edge
- `mon_cb` — read-only; samples all request, response, and command bus signals

#### Transactions — `transactions.sv`
Two transaction types:
- `ddr5_req_txn` — randomized request with `is_write`, `addr`, `wdata`, decoded sub-fields, and constraints that keep the address packing consistent with the DUT's field layout. The `c_addr_pack` constraint ensures `addr` is always a canonical address with sub-fields consistent with `rank`, `bg`, `bank`, `row`, `col`.
- `ddr5_cmd_txn` — observed command from the DUT's observability bus (`cmd_code`, `cmd_bg`, `cmd_bank`, `cmd_row`, `cmd_col`, `cycle_num`)

#### Sequences — `sequences.sv`

16 sequences across two categories:

**Protocol scenario sequences** — each targets a specific DUT code path:

| Sequence | Target scenario |
|----------|----------------|
| `ddr5_row_hit_seq` | Write then two reads to the same address — row stays open |
| `ddr5_row_miss_seq` | Two requests to same bank, different rows — forces PRE + ACT |
| `ddr5_multi_bank_seq` | Writes to all 16 banks then read back — full BG/bank matrix |
| `ddr5_multi_rank_seq` | Requests across both ranks — rank switching and address decode |
| `ddr5_wtr_seq` | Write immediately followed by read — tWTR stress |
| `ddr5_rtw_seq` | Read immediately followed by write — tRTW stress |
| `ddr5_ccd_seq` | Back-to-back column commands across BGs — tCCD_S stress |
| `ddr5_refresh_seq` | Enough ops to span a refresh boundary — bank drain and resume |
| `ddr5_stress_seq` | 200 fully-random transactions — constrained-random coverage closure |

**Timing-boundary sequences** — each is designed to hit a specific JEDEC parameter exactly at its boundary, exercising the DPI timing checker at the transition point:

| Sequence | Boundary targeted |
|----------|------------------|
| `ddr5_trcd_exact_seq` | ACT → column at exactly `tRCD` cycles |
| `ddr5_twtr_exact_seq` | WRITE → READ at exactly `tWTR` cycles |
| `ddr5_trtw_exact_seq` | READ → WRITE at exactly `tRTW` cycles |
| `ddr5_tccd_s_seq` | Column to column across BGs at exactly `tCCD_S` cycles |
| `ddr5_twtr_later_seq` | WRITE → READ with extra NOP gap (well after `tWTR`) |
| `ddr5_trtw_later_seq` | READ → WRITE with extra NOP gap (well after `tRTW`) |
| `ddr5_tccd_s_later_seq` | Cross-BG column with extra traffic (well after `tCCD_S`) |

#### Driver — `driver.sv`
Waits for `req_ready` then presents the request for exactly one cycle. Deasserts all signals cleanly afterwards. The wait-then-drive pattern ensures the DUT sees a stable request on the same cycle it is accepted.

#### Monitor — `monitor.sv`
Three parallel threads running under `fork...join`:
1. **Request thread** — fires when `req_valid && req_ready`; publishes `ddr5_req_txn` to `req_ap`
2. **Response thread** — fires when `rsp_valid`; publishes `ddr5_req_txn` (with `rdata` filled) to `rsp_ap`
3. **Command thread** — fires when `cmd_valid`; captures full command bus + cycle count; publishes `ddr5_cmd_txn` to `cmd_ap`

#### Scoreboard — `scoreboard.sv`

Two independent checking layers per check type:

**Data integrity — Layer 1 (SV):**
- Accepted WRITEs stored in `exp_by_addr[]` associative array
- Accepted READs push address onto `pending_reads[$]`
- Each `rsp_valid` pops from `pending_reads`, compares `rsp_rdata` against stored value

**Data integrity — Layer 2 (DPI-C):**
- `ddr5_dpi_write_req` mirrors every write into C++ memory at the same time
- `ddr5_dpi_read_req` snapshots expected data into a FIFO at read-request time
- `ddr5_dpi_read_rsp` pops the snapshot, compares all 16 BL16 words, reports each mismatched beat individually

**Timing — Layer 1 (SV):** `tCCD_L`, `tWTR`, `tRTW`, `tRP`, `tRC`, `tRAS`

**Timing — Layer 2 (DPI-C):** all of the above plus `tCCD_S`, `tRCD`, bank-open checks, and correct post-REF timestamp reset

#### Coverage — `coverage.sv`

13 functional covergroups tracking both the request side and the DUT command stream:

| Covergroup | What it measures |
|------------|----------------|
| `cg_req_type` | READ and WRITE both exercised |
| `cg_bank_access` | All 16 BG × Bank combinations accessed |
| `cg_row_access` | Row hit / row miss / empty bank per BG |
| `cg_timing_err` | Legal timing and at-boundary scenarios both seen |
| `cg_cmd_sequence` | Key transitions: ACT→READ, WRITE→PRE, PRE→ACT, WRITE→READ, READ→WRITE |
| `cg_refresh` | Refresh commands observed |
| `cg_col_range` | Low / mid / high column addresses accessed |
| `cg_bank_conflict` | Back-to-back same-bank different-row scenarios |
| `cg_cmd_bg_cross` | Every command type seen on every bank group |
| `cg_tccd` | `tCCD_L` boundary buckets: at-boundary and later |
| `cg_twtr` | `tWTR` boundary buckets |
| `cg_trtw` | `tRTW` boundary buckets |
| `cg_tccd_s` | `tCCD_S` cross-BG boundary buckets (new in DPI version) |

---

## How to Run on EDA Playground

### Setup

1. Go to [edaplayground.com](https://www.edaplayground.com) and log in
2. Select **Aldec Riviera-PRO 2025.04**
3. Tick **UVM 1.2** in the Libraries panel
4. Enable **"Use run.bash shell script"** in the Run Options panel

### Files

Place all files in the **Design** pane in this order:

```
ddr5_tb_params_pkg.sv
ddr5_tb_utils_pkg.sv
ddr5_dut_if.sv
transactions.sv
sequences.sv
driver.sv
monitor.sv
scoreboard.sv
coverage.sv
agent.sv
env.sv
tests.sv
testbench.sv        ← top-level (tb_top module)
design.sv           ← DUT wrapper
ddr5_dpi_model.h    ← C++ header (must be present for #include)
ddr5_dpi_model.cpp  ← C++ golden model
```

### run.bash

The `run.bash` script handles compilation and simulation in three steps:

```bash
# Step 1: Compile DPI-C golden model into a shared object
g++ -fPIC -shared \
    -std=c++11 \
    -I"$RIVIERA_HOME/interfaces/include" \
    -o ddr5_dpi_model.so \
    ddr5_dpi_model.cpp

# Step 2: Compile SystemVerilog
vlib work
vlog -sv -timescale 1ns/1ns \
    +incdir+$RIVIERA_HOME/vlib/uvm-1.2/src \
    +incdir+. \
    -l uvm_1_2 \
    design.sv testbench.sv

# Step 3: Simulate
vsim -c -do run.do
```

### run.do

```tcl
asim +access +r/+w -sv_lib ./ddr5_dpi_model work.tb_top
run -all
exit
```

The `-sv_lib ./ddr5_dpi_model` flag tells Riviera-PRO to load `ddr5_dpi_model.so` at startup and resolve all `import "DPI-C"` declarations against it.

### Run Options (EDA Playground UI)

| Field | Value |
|-------|-------|
| Compile Options | `-timescale 1ns/1ns` |
| Run Options | `+access+r` |
| Use run.bash | ✅ enabled |
| Open EPWave after run | ✅ recommended |

---

## Reading the Simulation Output

After the run completes, look for these blocks in the transcript:

**DPI model output (from C++ `printf`):**
```
[DDR5-DPI] Golden model reset complete
[DDR5-DPI][ERROR] tCCD_L violation READ ...    ← timing violation if any
```

**Scoreboard DPI comparison log (UVM_MEDIUM):**
```
SB_DPI_COMPARE: ADDR=0x10002040 DUT=0xA5A50001 MODEL=0xA5A50001 RESULT=PASS
```

**Final scoreboard report:**
```
SB: WRITES accepted      : 145
SB: READS accepted       : 147
SB: SV  READ PASS        : 147
SB: SV  READ FAIL        : 0
SB_DPI: DPI TIMING ERRORS : 0
SB_DPI: C MODEL: ALL TIMING CHECKS PASSED
```

**Coverage report:**
```
COV: cg_req_type      : 100.0%
COV: cg_bank_access   : 100.0%
...
COV: cg_tccd_s        : 100.0%
COV: TOTAL COVERAGE   : 90.8%
```

---

## Verification Plan

### What Is Verified

| Category | Checks | Verified by |
|----------|--------|-------------|
| Data integrity — all 16 beats | SV compare + C++ beat-by-beat compare | SV Layer 1 + DPI Layer 2 |
| `tRCD` (ACT → column) | C++ model | DPI Layer 2 |
| `tRAS` (min row active) | SV + C++ | Both layers |
| `tRP` (PRE → ACT) | SV + C++ | Both layers |
| `tRC` (ACT → ACT) | SV + C++ | Both layers |
| `tWTR` (WRITE → READ) | SV + C++ | Both layers |
| `tRTW` (READ → WRITE) | SV + C++ | Both layers |
| `tCCD_L` (same-BG column) | SV + C++ | Both layers |
| `tCCD_S` (cross-BG column) | C++ model | DPI Layer 2 |
| Bank-open before column | C++ model | DPI Layer 2 |
| REF timestamp reset | C++ model | DPI Layer 2 |
| All 16 banks (2R × 4BG × 4B) | Sequences + coverage | Multi-bank + stress seqs |
| Row hit / miss / empty | Coverage + directed seqs | `cg_row_access` |
| Refresh drain + resume | Sequence + C++ bank-open check | `ddr5_refresh_seq` |
| tRAS_MAX watchdog | Stress sequence | `ddr5_stress_seq` |
| At-boundary timing | 7 boundary sequences | `cg_tccd`, `cg_twtr`, `cg_trtw`, `cg_tccd_s` |

### Known Limitations

| Limitation | Impact | Path to closure |
|------------|--------|----------------|
| Per-bank refresh (REFPB) not modeled | All-bank refresh only; REFPB requires a separate address scheduler | Out of scope for behavioral controller model |
| Rank-to-rank switching latency (`tCMD_RATE`) | Rank interleaving not verified | Add directed multi-rank column interleave sequence |
| Physical CA bus timing (`tCMD`) | DUT operates at command level only; PHY timing not modeled | Requires PHY wrapper layer |
| Power-down modes | CKE-based power-down and exit timing (`tXP`) not implemented in DUT | Add CKE support to DUT and driver |

---

## Key Design Decisions

**Why C++ rather than C?**
`std::unordered_map`, `std::queue`, and `std::array` make the golden memory and FIFO implementations clean and safe. The `extern "C"` wrapper means the SV DPI linkage is unaffected by C++ name mangling.

**Why `svBitVecVal*` for the 512-bit bus?**
Passing a `bit [511:0]` packed vector from SV to C++ maps naturally to `const svBitVecVal*` (an array of `uint32_t`). This avoids the fragile 16-individual-`int` approach and lets the C++ side iterate cleanly over `DATA_WORDS = 16` words.

**Why snapshot at `read_req`, not at `read_rsp`?**
The DUT pipeline can be multiple cycles deep. If the scoreboard read the expected data at `rsp_valid` time, intervening writes to the same address would change the expected value. Snapshotting at `read_req` time into the `pending_reads` FIFO captures the correct expected value at the point the read was issued, matching the JEDEC read semantics.

**Why does `dpi_fail_cnt` exist separately from `fail_cnt`?**
The two checkers may disagree in specific edge cases (the REF timestamp bug being the main example). Keeping separate counters lets you see whether a failure was caught by one checker, both, or only the other — which is useful for diagnosing both DUT bugs and model discrepancies.
