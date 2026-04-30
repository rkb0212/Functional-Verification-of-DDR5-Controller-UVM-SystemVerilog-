// DDR5 DPI-C Golden Reference Model — Implementation

#include "ddr5_dpi_model.h"
#include <unordered_map>
#include <queue>
#include <array>
#include <cstdio>
#include <cstdint>
#include <cstring>

// =============================================================================
// Compile-time constants — mirror ddr5_tb_params_pkg.sv exactly
// =============================================================================
static const int RANKS          = 2;
static const int BANK_GROUPS    = 4;
static const int BANKS_PER_GROUP= 4;
static const int BANKS_TOTAL    = RANKS * BANK_GROUPS * BANKS_PER_GROUP; // 32

static const int DATA_WORDS     = 16;   // DATA_W(32) * BURST_LEN(16) / 32

// Timing parameters — must match ddr5_tb_params_pkg.sv
static const int T_RCD   = 4;
static const int T_RAS   = 8;
static const int T_RP    = 4;
static const int T_RC    = 12;
static const int T_WTR   = 4;
static const int T_RTW   = 6;
static const int T_CCD_L = 8;
static const int T_CCD_S = 4;   // tracked per-BG but checked against same-BG commands

// =============================================================================
// Internal types
// =============================================================================

// 512-bit burst = 16 x uint32_t
using burst_t = std::array<uint32_t, 16>;

// Per-bank state — mirrors bank_state_t in ddr5_controller_DUT.sv
struct BankState {
    bool    open     = false;
    int     open_row = 0;
    int64_t last_act = -1;   // -1 = never; -1 guard used for all checks
    int64_t last_pre = -1;
};

// =============================================================================
// Static model state
// =============================================================================

// 32 banks indexed by flat_bank()
static BankState banks[BANKS_TOTAL];

// Per-BG last column command cycle (-1 = never)
// Used for tCCD_L (same BG) and tCCD_S (different BG) checks
static int64_t bg_last_col[BANK_GROUPS];

// Global bus turnaround timestamps
static int64_t last_read_cycle;    // for tRTW: READ→WRITE check
static int64_t last_write_cycle;   // for tWTR:  WRITE→READ check

// Memory: [bank_flat][row][col] = one BL16 burst
// 32 banks × 16 rows × 16 cols × 16 words × 4 bytes = 524,288 bytes
static burst_t mem[BANKS_TOTAL][16][16];

// Pending-read FIFO: expected burst data snapshotted at req time
static std::queue<burst_t> pending_reads;

// =============================================================================
// Internal helpers (not exported)
// =============================================================================

// Flat bank index — identical to DUT flatten_bank() (DUT line 232)
static inline int flat_bank(unsigned int rank, unsigned int bg, unsigned int bank) {
    return (int)(rank * BANK_GROUPS * BANKS_PER_GROUP
               + bg   * BANKS_PER_GROUP
               + bank);
}

// Decode address fields — matches ddr5_tb_utils_pkg.sv canonical_addr()
// col  = bits [3:0]
// bank = bits [5:4]
// bg   = bits [7:6]
// rank = bit  [8]
// row  = bits [31:28]
static inline unsigned int addr_col (uint32_t a) { return  a        & 0xFu; }
static inline unsigned int addr_bank(uint32_t a) { return (a >>  4) & 0x3u; }
static inline unsigned int addr_bg  (uint32_t a) { return (a >>  6) & 0x3u; }
static inline unsigned int addr_rank(uint32_t a) { return (a >>  8) & 0x1u; }
static inline unsigned int addr_row (uint32_t a) { return (a >> 28) & 0xFu; }

// Copy svBitVecVal[16] → burst_t
// svBitVecVal is uint32_t. Element [i] holds bits [(i+1)*32-1 : i*32].
static void sv_to_burst(const svBitVecVal *sv, burst_t &out) {
    for (int i = 0; i < DATA_WORDS; i++)
        out[i] = (uint32_t)sv[i];
}

// Copy burst_t → svBitVecVal[16]
static void burst_to_sv(const burst_t &in, svBitVecVal *sv) {
    for (int i = 0; i < DATA_WORDS; i++)
        sv[i] = (svBitVecVal)in[i];
}

// =============================================================================
// Exported DPI-C functions
// =============================================================================

extern "C" {

// -----------------------------------------------------------------------------
// ddr5_dpi_reset
// Clears all model state to power-on defaults.
// Called from scoreboard build_phase and on rst_n assertion.
// -----------------------------------------------------------------------------
void ddr5_dpi_reset(void) {
    for (int b = 0; b < BANKS_TOTAL; b++)
        banks[b] = BankState();              // open=false, rows=0, timestamps=-1

    for (int g = 0; g < BANK_GROUPS; g++)
        bg_last_col[g] = -1;

    last_read_cycle  = -1;
    last_write_cycle = -1;

    // Zero all model memory
    memset(mem, 0, sizeof(mem));

    // Drain any leftover pending reads from a previous run
    while (!pending_reads.empty())
        pending_reads.pop();

    std::printf("[DDR5-DPI] Golden model reset complete\n");
}

// -----------------------------------------------------------------------------
// ddr5_dpi_write_req
// Store write burst into model memory at request-acceptance time.
// -----------------------------------------------------------------------------
void ddr5_dpi_write_req(unsigned int addr, const svBitVecVal *wdata) {
    unsigned int r   = addr_rank(addr);
    unsigned int bg  = addr_bg  (addr);
    unsigned int bk  = addr_bank(addr);
    unsigned int row = addr_row (addr);
    unsigned int col = addr_col (addr);
    int b = flat_bank(r, bg, bk);

    sv_to_burst(wdata, mem[b][row][col]);
}

// -----------------------------------------------------------------------------
// ddr5_dpi_read_req
// -----------------------------------------------------------------------------
void ddr5_dpi_read_req(unsigned int addr) {
    unsigned int r   = addr_rank(addr);
    unsigned int bg  = addr_bg  (addr);
    unsigned int bk  = addr_bank(addr);
    unsigned int row = addr_row (addr);
    unsigned int col = addr_col (addr);
    int b = flat_bank(r, bg, bk);

    // mem[][] is zero-initialised in reset, so unwritten addresses return 0
    pending_reads.push(mem[b][row][col]);
}

// -----------------------------------------------------------------------------
// ddr5_dpi_read_rsp
// Compare DUT rsp_rdata against the expected burst popped from the FIFO.
// Returns 1 (PASS) if all 16 words match, 0 (FAIL) otherwise.
// -----------------------------------------------------------------------------
int ddr5_dpi_read_rsp(const svBitVecVal *dut_rdata, svBitVecVal *exp_rdata) {
    if (pending_reads.empty()) {
        std::printf("[DDR5-DPI][ERROR] rsp_valid seen with no pending DPI read\n");
        // Fill exp_rdata with zeros to avoid leaving it uninitialised
        for (int i = 0; i < DATA_WORDS; i++)
            exp_rdata[i] = 0;
        return 0;
    }

    burst_t exp = pending_reads.front();
    pending_reads.pop();

    burst_to_sv(exp, exp_rdata);

    int pass = 1;
    for (int i = 0; i < DATA_WORDS; i++) {
        if ((uint32_t)dut_rdata[i] != exp[i]) {
            std::printf(
                "[DDR5-DPI][ERROR] READ mismatch beat=%d "
                "exp=0x%08x got=0x%08x\n",
                i, exp[i], (uint32_t)dut_rdata[i]);
            pass = 0;
        }
    }
    return pass;
}

// -----------------------------------------------------------------------------
// ddr5_dpi_cmd
// Timing compliance checker for every cmd_valid pulse.
// Returns 1 = PASS, 0 = at least one JEDEC timing violation detected.
//
// Design notes on each case:
//
// ACT (cmd_code=1):
//   tRP:  delta since last PRE on THIS bank must be >= T_RP.
//         Guard: last_pre >= 0 (skip check if bank never precharged).
//   tRC:  delta since last ACT on THIS bank must be >= T_RC.
//         Guard: last_act >= 0 (skip check if bank never activated).
//   Update: open=true, open_row=row, last_act=cycle.
//
// READ (cmd_code=2):
//   tRCD: delta since last ACT on THIS bank must be >= T_RCD.
//   tWTR: delta since last WRITE (any bank) must be >= T_WTR.
//   tCCD_L: delta since last col cmd in SAME BG must be >= T_CCD_L.
//   tCCD_S: delta since last col cmd in OTHER BGs must be >= T_CCD_S.
//   Bank-open check: bank must be open (last_act >= 0 and open==true).
//   Update: last_read_cycle=cycle, bg_last_col[bg]=cycle.
//
// WRITE (cmd_code=3):
//   tRCD: delta since last ACT on THIS bank must be >= T_RCD.
//   tRTW: delta since last READ (any bank) must be >= T_RTW.
//   tCCD_L / tCCD_S: same as READ.
//   Bank-open check: same as READ.
//   Update: last_write_cycle=cycle, bg_last_col[bg]=cycle.
//
// PRE (cmd_code=4):
//   tRAS: delta since last ACT on THIS bank must be >= T_RAS.
//   Guard: last_act >= 0.
//   Update: open=false, last_pre=cycle.
//
// REF (cmd_code=5):
//   All banks must be closed. Prints error for each open bank found.
//   FIX vs original code: resets last_act AND last_pre to -1 for all banks.
//   Per JEDEC, refresh resets the tRC/tRP timing windows — without this
//   reset, the first ACT after REF would be wrongly blocked by the pre-REF
//   timestamps.
// -----------------------------------------------------------------------------
int ddr5_dpi_cmd(int cycle, int cmd_code,
                 unsigned int rank, unsigned int bg,
                 unsigned int bank, unsigned int row, unsigned int col) {
    int64_t cyc = (int64_t)cycle;
    int b       = flat_bank(rank, bg, bank);
    int pass    = 1;

    switch (cmd_code) {

        // -----------------------------------------------------------------
        case 1: { // ACT
            // tRP: must wait T_RP cycles after PRE before next ACT
            if (banks[b].last_pre >= 0) {
                int64_t d = cyc - banks[b].last_pre;
                if (d < (int64_t)T_RP) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tRP violation ACT "
                        "rank=%u bg=%u bank=%u cycle=%d delta=%lld < tRP=%d\n",
                        rank, bg, bank, cycle, (long long)d, T_RP);
                    pass = 0;
                }
            }
            // tRC: must wait T_RC cycles between consecutive ACTs on same bank
            if (banks[b].last_act >= 0) {
                int64_t d = cyc - banks[b].last_act;
                if (d < (int64_t)T_RC) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tRC violation ACT "
                        "rank=%u bg=%u bank=%u cycle=%d delta=%lld < tRC=%d\n",
                        rank, bg, bank, cycle, (long long)d, T_RC);
                    pass = 0;
                }
            }
            // Update state
            banks[b].open     = true;
            banks[b].open_row = (int)row;
            banks[b].last_act = cyc;
            break;
        }

        // -----------------------------------------------------------------
        case 2: { // READ
            // Bank must be open
            if (!banks[b].open) {
                std::printf(
                    "[DDR5-DPI][ERROR] READ to closed bank "
                    "rank=%u bg=%u bank=%u cycle=%d\n",
                    rank, bg, bank, cycle);
                pass = 0;
            }
            // tRCD: ACT → READ
            if (banks[b].last_act >= 0) {
                int64_t d = cyc - banks[b].last_act;
                if (d < (int64_t)T_RCD) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tRCD violation READ "
                        "rank=%u bg=%u bank=%u cycle=%d delta=%lld < tRCD=%d\n",
                        rank, bg, bank, cycle, (long long)d, T_RCD);
                    pass = 0;
                }
            }
            // tWTR: WRITE → READ bus turnaround
            if (last_write_cycle >= 0) {
                int64_t d = cyc - last_write_cycle;
                if (d < (int64_t)T_WTR) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tWTR violation READ "
                        "cycle=%d delta=%lld < tWTR=%d\n",
                        cycle, (long long)d, T_WTR);
                    pass = 0;
                }
            }
            // tCCD_L: same-BG column spacing
            if (bg_last_col[bg] >= 0) {
                int64_t d = cyc - bg_last_col[bg];
                if (d < (int64_t)T_CCD_L) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tCCD_L violation READ "
                        "bg=%u cycle=%d delta=%lld < tCCD_L=%d\n",
                        bg, cycle, (long long)d, T_CCD_L);
                    pass = 0;
                }
            }
            // tCCD_S: different-BG column spacing
            for (int g = 0; g < BANK_GROUPS; g++) {
                if (g == (int)bg) continue;
                if (bg_last_col[g] >= 0) {
                    int64_t d = cyc - bg_last_col[g];
                    if (d < (int64_t)T_CCD_S) {
                        std::printf(
                            "[DDR5-DPI][ERROR] tCCD_S violation READ "
                            "from_bg=%d to_bg=%u cycle=%d delta=%lld < tCCD_S=%d\n",
                            g, bg, cycle, (long long)d, T_CCD_S);
                        pass = 0;
                    }
                }
            }
            // Update state
            last_read_cycle  = cyc;
            bg_last_col[bg]  = cyc;
            (void)col;
            break;
        }

        // -----------------------------------------------------------------
        case 3: { // WRITE
            // Bank must be open
            if (!banks[b].open) {
                std::printf(
                    "[DDR5-DPI][ERROR] WRITE to closed bank "
                    "rank=%u bg=%u bank=%u cycle=%d\n",
                    rank, bg, bank, cycle);
                pass = 0;
            }
            // tRCD: ACT → WRITE
            if (banks[b].last_act >= 0) {
                int64_t d = cyc - banks[b].last_act;
                if (d < (int64_t)T_RCD) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tRCD violation WRITE "
                        "rank=%u bg=%u bank=%u cycle=%d delta=%lld < tRCD=%d\n",
                        rank, bg, bank, cycle, (long long)d, T_RCD);
                    pass = 0;
                }
            }
            // tRTW: READ → WRITE bus turnaround
            if (last_read_cycle >= 0) {
                int64_t d = cyc - last_read_cycle;
                if (d < (int64_t)T_RTW) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tRTW violation WRITE "
                        "cycle=%d delta=%lld < tRTW=%d\n",
                        cycle, (long long)d, T_RTW);
                    pass = 0;
                }
            }
            // tCCD_L: same-BG column spacing
            if (bg_last_col[bg] >= 0) {
                int64_t d = cyc - bg_last_col[bg];
                if (d < (int64_t)T_CCD_L) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tCCD_L violation WRITE "
                        "bg=%u cycle=%d delta=%lld < tCCD_L=%d\n",
                        bg, cycle, (long long)d, T_CCD_L);
                    pass = 0;
                }
            }
            // tCCD_S: different-BG column spacing
            for (int g = 0; g < BANK_GROUPS; g++) {
                if (g == (int)bg) continue;
                if (bg_last_col[g] >= 0) {
                    int64_t d = cyc - bg_last_col[g];
                    if (d < (int64_t)T_CCD_S) {
                        std::printf(
                            "[DDR5-DPI][ERROR] tCCD_S violation WRITE "
                            "from_bg=%d to_bg=%u cycle=%d delta=%lld < tCCD_S=%d\n",
                            g, bg, cycle, (long long)d, T_CCD_S);
                        pass = 0;
                    }
                }
            }
            // Update state
            last_write_cycle = cyc;
            bg_last_col[bg]  = cyc;
            (void)col;
            break;
        }

        // -----------------------------------------------------------------
        case 4: { // PRE
            // tRAS: ACT → PRE minimum active time
            if (banks[b].last_act >= 0) {
                int64_t d = cyc - banks[b].last_act;
                if (d < (int64_t)T_RAS) {
                    std::printf(
                        "[DDR5-DPI][ERROR] tRAS violation PRE "
                        "rank=%u bg=%u bank=%u cycle=%d delta=%lld < tRAS=%d\n",
                        rank, bg, bank, cycle, (long long)d, T_RAS);
                    pass = 0;
                }
            }
            // Update state
            banks[b].open     = false;
            banks[b].last_pre = cyc;
            break;
        }

        // -----------------------------------------------------------------
        case 5: { // REF
            // All banks must be precharged before REF
            // (DUT drains all open banks in ST_REF_ISSUE before issuing REF)
            for (int k = 0; k < BANKS_TOTAL; k++) {
                if (banks[k].open) {
                    std::printf(
                        "[DDR5-DPI][ERROR] REF issued with open bank "
                        "flat_idx=%d cycle=%d\n", k, cycle);
                    pass = 0;
                }
            }
            // FIX: reset tRC and tRP timing windows for all banks.
            // Per JEDEC DDR5, refresh resets the row-cycle timing so the
            // next ACT after REF is not gated by the pre-REF timestamps.
            // The original code only set open=false but left last_act/last_pre
            // unchanged, causing false tRC violations on the first post-REF ACT.
            for (int k = 0; k < BANKS_TOTAL; k++) {
                banks[k].open     = false;
                banks[k].last_act = -1;   // ← FIX: was missing in original code
                banks[k].last_pre = -1;   // ← FIX: was missing in original code
            }
            break;
        }

        default: // NOP (cmd_code=0) — no check, no state update
            break;
    }

    return pass;
}

} 