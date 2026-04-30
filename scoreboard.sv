// =============================================================================
// SCOREBOARD — DPI-C Golden Model Integration
// =============================================================================

import "DPI-C" function void ddr5_dpi_reset();

import "DPI-C" function void ddr5_dpi_write_req(
    input int unsigned  addr,
    input bit [511:0]   wdata
);

import "DPI-C" function void ddr5_dpi_read_req(
    input int unsigned addr
);

import "DPI-C" function int ddr5_dpi_read_rsp(
    input  bit [511:0] dut_rdata,
    output bit [511:0] exp_rdata
);

import "DPI-C" function int ddr5_dpi_cmd(
    input int          cycle,
    input int          cmd_code,
    input int unsigned rank,
    input int unsigned bg,
    input int unsigned bank,
    input int unsigned row,
    input int unsigned col
);

// -----------------------------------------------------------------------------
// analysis imp tag declarations 
// -----------------------------------------------------------------------------
`uvm_analysis_imp_decl(_req)
`uvm_analysis_imp_decl(_rsp)
`uvm_analysis_imp_decl(_cmd)

// =============================================================================
// Scoreboard class
// =============================================================================
class ddr5_scoreboard extends uvm_component;
  `uvm_component_utils(ddr5_scoreboard)

  // original data structures 
  bit [DATA_BUS_W-1:0] exp_by_addr [bit[ADDR_W-1:0]];
  bit [ADDR_W-1:0]     pending_reads[$];

  uvm_analysis_imp_req #(ddr5_req_txn, ddr5_scoreboard) req_imp;
  uvm_analysis_imp_rsp #(ddr5_req_txn, ddr5_scoreboard) rsp_imp;
  uvm_analysis_imp_cmd #(ddr5_cmd_txn, ddr5_scoreboard) cmd_imp;

  int unsigned pass_cnt;
  int unsigned fail_cnt;
  int unsigned write_cnt;
  int unsigned read_cnt;

  // DPI timing error counter
  int unsigned dpi_fail_cnt;

  longint last_col_cycle_bg[BANK_GROUPS];
  longint last_read_cycle;
  longint last_write_cycle;
  longint last_act_cycle   [RANKS][BANK_GROUPS][BANKS_PER_GROUP];
  longint last_pre_cycle   [RANKS][BANK_GROUPS][BANKS_PER_GROUP];
  bit     bank_is_open     [RANKS][BANK_GROUPS][BANKS_PER_GROUP];
  bit [ROW_W-1:0] open_row [RANKS][BANK_GROUPS][BANKS_PER_GROUP];

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  // --------------------------------------------------------------------------
  // build_phase
  // --------------------------------------------------------------------------
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    req_imp = new("req_imp", this);
    rsp_imp = new("rsp_imp", this);
    cmd_imp = new("cmd_imp", this);

    pass_cnt     = 0;
    fail_cnt     = 0;
    write_cnt    = 0;
    read_cnt     = 0;
    dpi_fail_cnt = 0;

    // SV tracker init 
    last_read_cycle  = -999;
    last_write_cycle = -999;
    for (int i = 0; i < BANK_GROUPS; i++) last_col_cycle_bg[i] = -999;
    foreach (last_act_cycle[r,bg,bk]) begin
      last_act_cycle[r][bg][bk] = -999;
      last_pre_cycle[r][bg][bk] = -999;
      bank_is_open[r][bg][bk]   = 0;
      open_row[r][bg][bk]       = '0;
    end

    // reset C golden model 
    ddr5_dpi_reset();
    `uvm_info("SB_DPI", "C golden model reset complete", UVM_MEDIUM)
  endfunction

  // --------------------------------------------------------------------------
  // write_req
  // --------------------------------------------------------------------------
  function void write_req(ddr5_req_txn t);
    bit [ADDR_W-1:0] canon;
    canon = canonical_addr(t.addr);

    if (t.is_write) begin
      //  Layer 1: SV data store 
      exp_by_addr[canon] = t.wdata;
      write_cnt++;
      `uvm_info("SB",
        $sformatf("WRITE accepted addr=0x%08h wdata[31:0]=0x%08h",
                  canon, t.wdata[31:0]),
        UVM_LOW)

      //  Layer 2: DPI write — store burst into C model memory
      // wdata is bit[511:0] which maps to svBitVecVal*[16] in the C function.
      // Riviera-PRO passes packed SV arrays by pointer automatically.
      ddr5_dpi_write_req(canon, t.wdata);
    end
    else begin
      // Layer 1: enqueue address for SV compare 
      pending_reads.push_back(canon);
      read_cnt++;

      // Layer 2: DPI read — snapshot expected data into C model FIFO 
      ddr5_dpi_read_req(canon);
    end
  endfunction

  // --------------------------------------------------------------------------
  // write_rsp
  // --------------------------------------------------------------------------
  function void write_rsp(ddr5_req_txn t);
    bit [ADDR_W-1:0]     addr;
    bit [DATA_BUS_W-1:0] exp;
    bit [DATA_BUS_W-1:0] dpi_exp;   // NEW: expected data from C model
    int                  dpi_match; // NEW: 1=pass, 0=mismatch

    if (pending_reads.size() == 0) begin
      `uvm_error("SB", "rsp_valid seen but no pending read exists")
      fail_cnt++;
      return;
    end

    addr = pending_reads.pop_front();

    // Layer 2: DPI read response check
    // ddr5_dpi_read_rsp pops the expected burst snapshot, compares with DUT
    // rsp_rdata, and returns 1 (match) or 0 (mismatch).
    // dpi_exp is filled with the model's expected value for logging.
    dpi_match = ddr5_dpi_read_rsp(t.rdata, dpi_exp);

    `uvm_info("SB_DPI_COMPARE",
      $sformatf("\n-------------------------------\nADDR  = 0x%08h\nDUT   = 0x%08h\nMODEL = 0x%08h\nRESULT= %s\n-------------------------------",
                addr,
                t.rdata[31:0],
                dpi_exp[31:0],
                (dpi_match ? "PASS" : "FAIL")),
      UVM_MEDIUM)

    if (!dpi_match) begin
      dpi_fail_cnt++;
    end

    //  Layer 1: original SV compare 
    if (exp_by_addr.exists(addr)) begin
      exp = exp_by_addr[addr];
      if (t.rdata === exp) begin
        pass_cnt++;
      end
      else begin
        `uvm_error("SB",
          $sformatf("READ mismatch addr=0x%08h exp[31:0]=0x%08h got[31:0]=0x%08h",
                    addr, exp[31:0], t.rdata[31:0]))
        fail_cnt++;
      end
    end
    else begin
      if (t.rdata === '0) begin
        pass_cnt++;
        `uvm_info("SB",
          $sformatf("READ of unwritten addr=0x%08h got zero as expected", addr),
          UVM_LOW)
      end
      else begin
        `uvm_warning("SB",
          $sformatf("READ of unwritten addr=0x%08h got nonzero[31:0]=0x%08h",
                    addr, t.rdata[31:0]))
        pass_cnt++;
      end
    end
  endfunction

  // --------------------------------------------------------------------------
  // write_cmd
  // Called when monitor sees cmd_valid.
  // Layer 2 (DPI) runs first, then Layer 1
  // --------------------------------------------------------------------------
  function void write_cmd(ddr5_cmd_txn c);
    int dpi_pass;

    //  Layer 2: DPI timing check 
  dpi_pass = ddr5_dpi_cmd(
      int'(c.cycle_num),
      int'(c.cmd_code),
      int'(c.cmd_rank),
      int'(c.cmd_bg),
      int'(c.cmd_bank),
      int'(c.cmd_row),
      int'(c.cmd_col)
  );

    if (!dpi_pass) begin
`uvm_error("SB_DPI",
  $sformatf("C MODEL TIMING VIOLATION cmd=%0d rank=%0d bg=%0d bank=%0d row=%0d col=%0d cycle=%0d",
            c.cmd_code, c.cmd_rank, c.cmd_bg, c.cmd_bank,
            c.cmd_row, c.cmd_col, c.cycle_num))
      dpi_fail_cnt++;
    end
    else begin
      `uvm_info("SB_DPI",
        $sformatf("C MODEL OK cmd=%0d rank=%0d bg=%0d bank=%0d cycle=%0d",
                  c.cmd_code, c.cmd_rank, c.cmd_bg, c.cmd_bank, c.cycle_num),
        UVM_HIGH)
    end

    //  Layer 1: original SV timing checks 
    if ((c.cmd_code == 3'd2) || (c.cmd_code == 3'd3)) begin
      // Same-BG tCCD_L
      if ((last_col_cycle_bg[c.cmd_bg] >= 0) &&
          ((c.cycle_num - last_col_cycle_bg[c.cmd_bg]) < tCCD_L)) begin
        `uvm_error("SB",
          $sformatf("same-BG column spacing violation bg=%0d cycle=%0d delta=%0d < tCCD_L=%0d",
                    c.cmd_bg, c.cycle_num,
                    c.cycle_num - last_col_cycle_bg[c.cmd_bg], tCCD_L))
        fail_cnt++;
      end

      if ((c.cmd_code == 3'd2) &&
          (last_write_cycle >= 0) &&
          ((c.cycle_num - last_write_cycle) < tWTR)) begin
        `uvm_error("SB",
          $sformatf("READ too soon after WRITE at cycle %0d delta=%0d < tWTR=%0d",
                    c.cycle_num, c.cycle_num - last_write_cycle, tWTR))
        fail_cnt++;
      end

      if ((c.cmd_code == 3'd3) &&
          (last_read_cycle >= 0) &&
          ((c.cycle_num - last_read_cycle) < tRTW)) begin
        `uvm_error("SB",
          $sformatf("WRITE too soon after READ at cycle %0d delta=%0d < tRTW=%0d",
                    c.cycle_num, c.cycle_num - last_read_cycle, tRTW))
        fail_cnt++;
      end

      last_col_cycle_bg[c.cmd_bg] = c.cycle_num;
      if (c.cmd_code == 3'd2) last_read_cycle  = c.cycle_num;
      if (c.cmd_code == 3'd3) last_write_cycle = c.cycle_num;
    end

    case (c.cmd_code)
      3'd1: begin // ACT
        if ((last_pre_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank] >= 0) &&
            ((c.cycle_num - last_pre_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank]) < tRP)) begin
          `uvm_error("SB",
            $sformatf("tRP violation rank=%0d bg=%0d bank=%0d delta=%0d < tRP=%0d",
                      c.cmd_rank, c.cmd_bg, c.cmd_bank,
                      c.cycle_num - last_pre_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank], tRP))
          fail_cnt++;
        end
        if ((last_act_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank] >= 0) &&
            ((c.cycle_num - last_act_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank]) < tRC)) begin
          `uvm_error("SB",
            $sformatf("tRC violation rank=%0d bg=%0d bank=%0d delta=%0d < tRC=%0d",
                      c.cmd_rank, c.cmd_bg, c.cmd_bank,
                      c.cycle_num - last_act_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank], tRC))
          fail_cnt++;
        end
        last_act_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank] = c.cycle_num;
        bank_is_open[c.cmd_rank][c.cmd_bg][c.cmd_bank]   = 1;
        open_row[c.cmd_rank][c.cmd_bg][c.cmd_bank]       = c.cmd_row;
      end

      3'd4: begin // PRE
        if ((last_act_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank] >= 0) &&
            ((c.cycle_num - last_act_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank]) < tRAS)) begin
          `uvm_error("SB",
            $sformatf("tRAS violation rank=%0d bg=%0d bank=%0d delta=%0d < tRAS=%0d",
                      c.cmd_rank, c.cmd_bg, c.cmd_bank,
                      c.cycle_num - last_act_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank], tRAS))
          fail_cnt++;
        end
        last_pre_cycle[c.cmd_rank][c.cmd_bg][c.cmd_bank] = c.cycle_num;
        bank_is_open[c.cmd_rank][c.cmd_bg][c.cmd_bank]   = 0;
      end

      3'd5: begin // REF
        foreach (bank_is_open[r,bg,bk])
          bank_is_open[r][bg][bk] = 0;
      end

      default: begin end
    endcase

  endfunction

  // --------------------------------------------------------------------------
  // report_phase
  // --------------------------------------------------------------------------
  function void report_phase(uvm_phase phase);
    `uvm_info("SB",  "========================================", UVM_MEDIUM)
    `uvm_info("SB",  "       SCOREBOARD FINAL REPORT",          UVM_MEDIUM)
    `uvm_info("SB",  "========================================", UVM_MEDIUM)
    `uvm_info("SB",  $sformatf("WRITES accepted      : %0d", write_cnt),    UVM_MEDIUM)
    `uvm_info("SB",  $sformatf("READS accepted       : %0d", read_cnt),     UVM_MEDIUM)
    `uvm_info("SB",  $sformatf("SV  READ PASS        : %0d", pass_cnt),     UVM_MEDIUM)
    `uvm_info("SB",  $sformatf("SV  READ FAIL        : %0d", fail_cnt),     UVM_MEDIUM)
    `uvm_info("SB_DPI", $sformatf("DPI TIMING ERRORS : %0d", dpi_fail_cnt),UVM_MEDIUM)
    if (dpi_fail_cnt == 0)
      `uvm_info("SB_DPI", "C MODEL: ALL TIMING CHECKS PASSED", UVM_MEDIUM)
    else
      `uvm_error("SB_DPI",
        $sformatf("C MODEL: %0d TIMING VIOLATIONS DETECTED", dpi_fail_cnt))
    if (pending_reads.size() > 0)
      `uvm_warning("SB",
        $sformatf("%0d reads never got a response", pending_reads.size()))
  endfunction

endclass
