// =============================================================================
// Testbench   : tb_ahb
// Description : Self-checking testbench for the AHB-Lite system (ahb_top).
//
//   Structure
//   ---------
//   1. Clock / reset
//   2. Reference model     : byte array that mirrors the two slave memories
//   3. Helper functions    : independent copies of the burst / address rules
//   4. Command task        : run_command() issues one command and checks it
//   5. Protocol checkers   : watch the AHB bus every clock
//                              - address/control stable during wait states
//                              - write data stable during wait states
//                              - ERROR is exactly a two-cycle response
//                              - burst addresses follow INCR / WRAP rules
//   6. Coverage counters
//   7. Test sequence       : directed tests + random tests
//
//   Stimulus is driven on the NEGATIVE clock edge; checks run on the
//   POSITIVE edge, so the testbench never races with the design.
// =============================================================================
`timescale 1ns / 1ps
`include "ahb_defines.vh"

module tb_ahb;

    // -------------------------------------------------------------------------
    // DUT connections
    // -------------------------------------------------------------------------
    reg         HCLK;
    reg         HRESETn;

    reg         start;
    reg         cmd_write;
    reg  [31:0] cmd_addr;
    reg  [2:0]  cmd_size;
    reg  [2:0]  cmd_burst;
    reg  [4:0]  cmd_len;
    wire        busy, done, error;

    reg         buf_we;
    reg  [3:0]  buf_idx;
    reg  [31:0] buf_wdata;
    wire [31:0] buf_rdata;

    ahb_top dut (
        .HCLK      (HCLK),
        .HRESETn   (HRESETn),
        .start     (start),
        .cmd_write (cmd_write),
        .cmd_addr  (cmd_addr),
        .cmd_size  (cmd_size),
        .cmd_burst (cmd_burst),
        .cmd_len   (cmd_len),
        .busy      (busy),
        .done      (done),
        .error     (error),
        .buf_we    (buf_we),
        .buf_idx   (buf_idx),
        .buf_wdata (buf_wdata),
        .buf_rdata (buf_rdata)
    );

    // internal AHB bus, used by the protocol checkers
    wire [31:0] HADDR  = dut.HADDR;
    wire [1:0]  HTRANS = dut.HTRANS;
    wire        HWRITE = dut.HWRITE;
    wire [2:0]  HSIZE  = dut.HSIZE;
    wire [2:0]  HBURST = dut.HBURST;
    wire [31:0] HWDATA = dut.HWDATA;
    wire        HREADY = dut.HREADY;
    wire        HRESP  = dut.HRESP;

    // -------------------------------------------------------------------------
    // 1. Clock (100 MHz)
    // -------------------------------------------------------------------------
    initial HCLK = 1'b0;
    always #5 HCLK = ~HCLK;

    // -------------------------------------------------------------------------
    // 2. Reference model: 8 KB byte array = slave 0 (0x0000) + slave 1 (0x1000)
    // -------------------------------------------------------------------------
    reg [7:0]  ref_mem [0:8191];
    reg [31:0] wdata_used [0:15];       // write data loaded for the current command

    integer data_errors     = 0;
    integer cmd_errors      = 0;
    integer protocol_errors = 0;
    integer commands_run    = 0;

    // coverage counters (see section 6)
    integer cov_burst [0:7];            // commands per burst type
    integer cov_size  [0:2];            // commands per transfer size
    integer cov_wait_cycles = 0;
    integer cov_error_resp  = 0;
    integer cov_wraps       = 0;
    integer n;

    // state remembered by the protocol checkers (see section 5)
    reg        prev_wait       = 1'b0;  // last cycle was a wait state
    reg        prev_err_first  = 1'b0;  // last cycle was ERROR cycle 1
    reg        prev_wdata_wait = 1'b0;  // last cycle was a stalled write data phase
    reg        prev_ready      = 1'b1;
    reg        dphase_write    = 1'b0;  // current data phase is a write
    reg [31:0] prev_addr, prev_wdata;
    reg [1:0]  prev_trans;
    reg        prev_write;
    reg [2:0]  prev_size, prev_burst;

    // -------------------------------------------------------------------------
    // 3. Helper functions (written independently from the RTL)
    // -------------------------------------------------------------------------
    function integer beats_of;
        input [2:0] burst;
        input [4:0] len;
        begin
            case (burst)
                `HBURST_SINGLE                 : beats_of = 1;
                `HBURST_WRAP4,  `HBURST_INCR4  : beats_of = 4;
                `HBURST_WRAP8,  `HBURST_INCR8  : beats_of = 8;
                `HBURST_WRAP16, `HBURST_INCR16 : beats_of = 16;
                default                        : beats_of = (len == 0) ? 1 : len;
            endcase
        end
    endfunction

    function [31:0] addr_after;
        input [31:0] addr;
        input [2:0]  size;
        input [2:0]  burst;
        integer bytes, beats, window;
        begin
            bytes = 1 << size;
            beats = beats_of(burst, 5'd1);
            if (burst == `HBURST_WRAP4 || burst == `HBURST_WRAP8 || burst == `HBURST_WRAP16) begin
                window     = bytes * beats;
                addr_after = (addr / window) * window + ((addr + bytes) % window);
            end else begin
                addr_after = addr + bytes;
            end
        end
    endfunction

    function [3:0] byte_enable;
        input [2:0] size;
        input [1:0] low_addr;
        begin
            case (size)
                `HSIZE_BYTE : byte_enable = 4'b0001 << low_addr;
                `HSIZE_HALF : byte_enable = 4'b0011 << low_addr;
                default     : byte_enable = 4'b1111;
            endcase
        end
    endfunction

    function is_mapped;
        input [31:0] addr;
        begin
            is_mapped = (addr < `SLAVE1_BASE + `SLAVE_SIZE);
        end
    endfunction

    function is_incr_burst;
        input [2:0] burst;
        begin
            is_incr_burst = (burst == `HBURST_INCR)  || (burst == `HBURST_INCR4) ||
                            (burst == `HBURST_INCR8) || (burst == `HBURST_INCR16);
        end
    endfunction

    // -------------------------------------------------------------------------
    // 4. Command task
    // -------------------------------------------------------------------------

    // number of address phases accepted on the bus (for burst-cancel check)
    integer bus_beats = 0;
    always @(posedge HCLK)
        if (HRESETn && HREADY && HTRANS[1])
            bus_beats = bus_beats + 1;

    // load random write data into the master's write buffer
    task load_write_data;
        input integer beats;
        integer k;
        begin
            for (k = 0; k < beats; k = k + 1) begin
                @(negedge HCLK);
                buf_we        = 1'b1;
                buf_idx       = k;
                buf_wdata     = $random;
                wdata_used[k] = buf_wdata;
            end
            @(negedge HCLK) buf_we = 1'b0;
        end
    endtask

    // issue one command, wait for it to finish, then check everything
    task run_command;
        input        write;
        input [31:0] addr;
        input [2:0]  size;
        input [2:0]  burst;
        input [4:0]  len;
        integer beats, k, b, expected_bus_beats;
        reg     expect_error;
        reg [31:0] a, rword;
        reg [3:0]  be;
        begin
            beats        = beats_of(burst, len);
            expect_error = !is_mapped(addr);

            if (write) load_write_data(beats);

            // start the command
            @(negedge HCLK);
            cmd_write = write;
            cmd_addr  = addr;
            cmd_size  = size;
            cmd_burst = burst;
            cmd_len   = len;
            bus_beats = 0;
            start     = 1'b1;
            @(negedge HCLK) start = 1'b0;

            // wait for 'done'
            @(posedge HCLK);
            while (!done) @(posedge HCLK);
            commands_run = commands_run + 1;

            // (a) error flag must match the address map
            if (error !== expect_error) begin
                $display("[%0t] ERROR: %s 0x%08h error flag = %b, expected %b",
                         $time, write ? "write" : "read", addr, error, expect_error);
                cmd_errors = cmd_errors + 1;
            end

            // (b) number of beats on the bus (ERROR must cancel the burst)
            expected_bus_beats = expect_error ? 1 : beats;
            if (bus_beats != expected_bus_beats) begin
                $display("[%0t] ERROR: %0d beats on bus, expected %0d (addr 0x%08h burst %0d)",
                         $time, bus_beats, expected_bus_beats, addr, burst);
                cmd_errors = cmd_errors + 1;
            end

            // (c) update the reference model (write) or check data (read)
            if (!expect_error) begin
                a = addr;
                for (k = 0; k < beats; k = k + 1) begin
                    be = byte_enable(size, a[1:0]);
                    if (!write) begin
                        buf_idx = k;
                        #1 rword = buf_rdata;
                    end
                    for (b = 0; b < 4; b = b + 1) begin
                        if (be[b]) begin
                            if (write)
                                ref_mem[{a[12:2], 2'b00} + b] = wdata_used[k][8*b +: 8];
                            else if (rword[8*b +: 8] !== ref_mem[{a[12:2], 2'b00} + b]) begin
                                $display("[%0t] ERROR: read 0x%08h byte %0d = 0x%02h, expected 0x%02h",
                                         $time, a, b, rword[8*b +: 8], ref_mem[{a[12:2], 2'b00} + b]);
                                data_errors = data_errors + 1;
                            end
                        end
                    end
                    a = addr_after(a, size, burst);
                end
            end

            cov_burst[burst] = cov_burst[burst] + 1;
            cov_size[size]   = cov_size[size] + 1;
        end
    endtask

    // write followed by read-back of the same locations
    task write_and_read;
        input [31:0] addr;
        input [2:0]  size;
        input [2:0]  burst;
        input [4:0]  len;
        begin
            run_command(1'b1, addr, size, burst, len);
            run_command(1'b0, addr, size, burst, len);
        end
    endtask

    // random legal command (aligned, INCR bursts never cross 1 KB)
    task random_command;
        integer region, bytes, beats, offset, block;
        reg [31:0] base;
        reg [2:0]  size, burst;
        reg [4:0]  len;
        begin
            region = {$random} % 10;                 // 0-4: S0, 5-8: S1, 9: unmapped
            base   = (region < 5) ? `SLAVE0_BASE :
                     (region < 9) ? `SLAVE1_BASE : 32'h8000_0000;
            size   = {$random} % 3;
            burst  = {$random} % 8;
            len    = ({$random} % 16) + 1;
            bytes  = 1 << size;
            beats  = beats_of(burst, len);
            block  = {$random} % 4;                  // which 1 KB block
            offset = ({$random} % 1024) & ~(bytes - 1);

            if (is_incr_burst(burst) && (offset + beats * bytes > 1024))
                offset = 1024 - beats * bytes;

            run_command($random, base + block * 1024 + offset, size, burst, len);
        end
    endtask

    // -------------------------------------------------------------------------
    // 5. Protocol checkers
    // -------------------------------------------------------------------------
    task protocol_error;
        input [8*60:1] msg;
        begin
            $display("[%0t] PROTOCOL ERROR: %0s", $time, msg);
            protocol_errors = protocol_errors + 1;
        end
    endtask

    always @(posedge HCLK) begin
        if (HRESETn) begin
            // address and control must not change during a wait state
            if (prev_wait && (HADDR !== prev_addr || HTRANS !== prev_trans ||
                              HWRITE !== prev_write || HSIZE !== prev_size ||
                              HBURST !== prev_burst))
                protocol_error("address/control changed during wait state");

            // write data must not change while the data phase is stalled
            if (prev_wdata_wait && HWDATA !== prev_wdata)
                protocol_error("HWDATA changed during wait state");

            // ERROR must be two cycles: (HREADY=0,ERROR) then (HREADY=1,ERROR)
            if (prev_err_first && !(HRESP && HREADY))
                protocol_error("ERROR first cycle not followed by second cycle");
            if (HRESP && HREADY && !prev_err_first)
                protocol_error("ERROR ended without the first cycle");

            // SEQ address must follow the burst rule
            if (HTRANS == `HTRANS_SEQ && prev_ready && prev_trans[1] &&
                HADDR !== addr_after(prev_addr, HSIZE, HBURST))
                protocol_error("wrong burst address");

            // coverage
            if (HTRANS[1] && !HREADY && !HRESP)  cov_wait_cycles = cov_wait_cycles + 1;
            if (HRESP && !HREADY)                cov_error_resp  = cov_error_resp + 1;
            if (HTRANS == `HTRANS_SEQ && prev_ready && prev_trans[1] && HADDR < prev_addr)
                cov_wraps = cov_wraps + 1;
        end

        // remember this cycle for the next one
        prev_wait       = HRESETn && HTRANS[1] && !HREADY && !HRESP;
        prev_err_first  = HRESETn && HRESP && !HREADY;
        prev_wdata_wait = HRESETn && dphase_write && !HREADY;
        prev_ready      = HREADY;
        prev_addr       = HADDR;
        prev_trans      = HTRANS;
        prev_write      = HWRITE;
        prev_size       = HSIZE;
        prev_burst      = HBURST;
        prev_wdata      = HWDATA;
        if (HREADY) dphase_write = HTRANS[1] && HWRITE;
    end

    // -------------------------------------------------------------------------
    // 6. Coverage counters are declared near the top and printed in report()
    // 7. Test sequence
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_ahb.vcd");
        $dumpvars(0, tb_ahb);

        for (n = 0; n < 8192; n = n + 1) ref_mem[n] = 8'h00;
        for (n = 0; n < 8; n = n + 1)    cov_burst[n] = 0;
        for (n = 0; n < 3; n = n + 1)    cov_size[n] = 0;

        start = 0; cmd_write = 0; cmd_addr = 0; cmd_size = 0; cmd_burst = 0; cmd_len = 0;
        buf_we = 0; buf_idx = 0; buf_wdata = 0;

        HRESETn = 1'b0;
        repeat (5) @(posedge HCLK);
        @(negedge HCLK) HRESETn = 1'b1;

        // ---- Test 1: single word transfers (slave 0 and slave 1 with wait states)
        $display("[%0t] Test 1: single transfers", $time);
        write_and_read(32'h0000_0010, `HSIZE_WORD, `HBURST_SINGLE, 0);
        write_and_read(32'h0000_1010, `HSIZE_WORD, `HBURST_SINGLE, 0);

        // ---- Test 2: byte and half-word accesses
        $display("[%0t] Test 2: byte / half-word transfers", $time);
        run_command(1'b1, 32'h0000_0100, `HSIZE_WORD, `HBURST_SINGLE, 0);
        write_and_read(32'h0000_0101, `HSIZE_BYTE, `HBURST_SINGLE, 0);
        write_and_read(32'h0000_0102, `HSIZE_HALF, `HBURST_SINGLE, 0);
        run_command(1'b0, 32'h0000_0100, `HSIZE_WORD, `HBURST_SINGLE, 0);
        write_and_read(32'h0000_1203, `HSIZE_BYTE, `HBURST_INCR4, 0);

        // ---- Test 3: every burst type, start address chosen so WRAP bursts wrap
        $display("[%0t] Test 3: all burst types", $time);
        write_and_read(32'h0000_0238, `HSIZE_WORD, `HBURST_INCR,   5);
        write_and_read(32'h0000_0338, `HSIZE_WORD, `HBURST_INCR4,  0);
        write_and_read(32'h0000_0438, `HSIZE_WORD, `HBURST_WRAP4,  0);
        write_and_read(32'h0000_0538, `HSIZE_WORD, `HBURST_INCR8,  0);
        write_and_read(32'h0000_0638, `HSIZE_WORD, `HBURST_WRAP8,  0);
        write_and_read(32'h0000_0738, `HSIZE_WORD, `HBURST_INCR16, 0);
        write_and_read(32'h0000_0838, `HSIZE_WORD, `HBURST_WRAP16, 0);
        write_and_read(32'h0000_1438, `HSIZE_HALF, `HBURST_WRAP8,  0);   // slave 1
        write_and_read(32'h0000_1538, `HSIZE_WORD, `HBURST_INCR16, 0);   // slave 1

        // ---- Test 4: ERROR response from the default slave
        $display("[%0t] Test 4: error responses", $time);
        run_command(1'b0, 32'h8000_0000, `HSIZE_WORD, `HBURST_SINGLE, 0);
        run_command(1'b1, 32'h8000_0010, `HSIZE_WORD, `HBURST_INCR4,  0); // burst must be cancelled
        write_and_read(32'h0000_0020, `HSIZE_WORD, `HBURST_INCR4, 0);    // bus still works

        // ---- Test 5: random commands
        $display("[%0t] Test 5: random commands", $time);
        repeat (300) random_command;

        repeat (10) @(posedge HCLK);
        report;
        $finish;
    end

    // safety net in case the design gets stuck (e.g. 'done' never comes)
    reg timed_out = 1'b0;
    initial begin
        #2_000_000;
        $display("ERROR: simulation timeout - the design or the test got stuck");
        timed_out = 1'b1;
        report;
        $finish;
    end

    // -------------------------------------------------------------------------
    // Final report
    // -------------------------------------------------------------------------
    task report;
        integer i, coverage_ok;
        begin
            coverage_ok = (cov_wait_cycles > 0) && (cov_error_resp > 0) && (cov_wraps > 0);
            for (i = 0; i < 8; i = i + 1) if (cov_burst[i] == 0) coverage_ok = 0;
            for (i = 0; i < 3; i = i + 1) if (cov_size[i]  == 0) coverage_ok = 0;

            $display("");
            $display("==================== TEST SUMMARY ====================");
            $display(" Commands run               : %0d", commands_run);
            $display(" Data mismatches            : %0d", data_errors);
            $display(" Command check failures     : %0d", cmd_errors);
            $display(" Protocol check failures    : %0d", protocol_errors);
            $display("------------------------------------------------------");
            $display(" Coverage");
            $display("   SINGLE/INCR              : %0d / %0d", cov_burst[0], cov_burst[1]);
            $display("   WRAP4/INCR4              : %0d / %0d", cov_burst[2], cov_burst[3]);
            $display("   WRAP8/INCR8              : %0d / %0d", cov_burst[4], cov_burst[5]);
            $display("   WRAP16/INCR16            : %0d / %0d", cov_burst[6], cov_burst[7]);
            $display("   byte/half/word           : %0d / %0d / %0d", cov_size[0], cov_size[1], cov_size[2]);
            $display("   wait-state cycles        : %0d", cov_wait_cycles);
            $display("   ERROR responses          : %0d", cov_error_resp);
            $display("   address wrap-arounds     : %0d", cov_wraps);
            $display("------------------------------------------------------");
            if (data_errors == 0 && cmd_errors == 0 && protocol_errors == 0 &&
                !timed_out && coverage_ok)
                $display(" RESULT : *** TEST PASSED ***");
            else
                $display(" RESULT : *** TEST FAILED ***");
            $display("======================================================");
        end
    endtask

endmodule
