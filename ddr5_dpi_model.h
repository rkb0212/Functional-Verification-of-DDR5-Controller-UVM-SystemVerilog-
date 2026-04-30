#ifndef DDR5_DPI_MODEL_H
#define DDR5_DPI_MODEL_H

#include <cstdint>   // uint32_t, int64_t — explicit, never rely on transitive includes
#include "svdpi.h"   // svBitVecVal — simulator-provided DPI header

#ifdef __cplusplus
extern "C" {
#endif

void ddr5_dpi_reset(void);

// -----------------------------------------------------------------------------
// ddr5_dpi_write_req
// -----------------------------------------------------------------------------
void ddr5_dpi_write_req(
    unsigned int        addr,
    const svBitVecVal  *wdata
);

// -----------------------------------------------------------------------------
// ddr5_dpi_read_req
// -----------------------------------------------------------------------------
void ddr5_dpi_read_req(
    unsigned int addr
);

// -----------------------------------------------------------------------------
// ddr5_dpi_read_rsp
// -----------------------------------------------------------------------------
int ddr5_dpi_read_rsp(
    const svBitVecVal  *dut_rdata,
    svBitVecVal        *exp_rdata
);

// -----------------------------------------------------------------------------
// ddr5_dpi_cmd
// Check a command observed on the cmd_valid bus for JEDEC timing compliance.
// Updates internal model state (bank open/closed, timestamps).
// Called by scoreboard write_cmd() for every cmd_valid pulse.
//
// cycle    : monitor's cycle_num at the time cmd_valid was sampled
//            (type 'int' matches monitor.sv's 'int cycle_num')
// cmd_code : 0=NOP 1=ACT 2=READ 3=WRITE 4=PRE 5=REF
//            (matches DUT's cmd_t enum in ddr5_controller_DUT.sv)
// rank, bg, bank, row, col: from cmd_rank/bg/bank/row/col ports
//            NOTE: row is only valid for ACT (cmd_code=1). For READ/WRITE,
//            the DUT does not update cmd_row (set only in ST_ACT_ISSUE);
//            the model uses its stored open_row for bank-state checks.
// Returns  : 1 = no violation (PASS), 0 = timing or state violation (FAIL)
//
// Timing checks performed per cmd_code:
//   ACT  (1): tRP  (PRE→ACT same bank), tRC (ACT→ACT same bank)
//   READ (2): tRCD (ACT→READ), tWTR (WRITE→READ), tCCD_L (same-BG col spacing)
//   WRITE(3): tRCD (ACT→WRITE), tRTW (READ→WRITE), tCCD_L (same-BG col spacing)
//   PRE  (4): tRAS (ACT→PRE same bank)
//   REF  (5): all banks must be closed; resets tRC/tRP timing windows
// -----------------------------------------------------------------------------
int ddr5_dpi_cmd(
    int          cycle,
    int          cmd_code,
    unsigned int rank,
    unsigned int bg,
    unsigned int bank,
    unsigned int row,
    unsigned int col
);

#ifdef __cplusplus
}
#endif

#endif 