// =============================================================================
// Module      : ahb_master
// Description : AHB-Lite master with a simple command interface.
//
//   How to use it
//   -------------
//   1. (writes only) Load the write data for each beat into the 16-word write
//      buffer using buf_we / buf_idx / buf_wdata.
//   2. Put the command on cmd_* and pulse 'start' for one clock.
//   3. Wait for the 'done' pulse. 'error' tells if any beat got an ERROR.
//   4. (reads only) Read the received data from the read buffer using
//      buf_idx / buf_rdata.
//
//   Supported transfers
//   -------------------
//   SINGLE, INCR (1-16 beats), INCR4/8/16, WRAP4/8/16
//   Byte, half-word and word sizes (HSIZE)
//
//   AHB behaviour
//   -------------
//   - Pipelined: while beat N is in its data phase, beat N+1 is already in its
//     address phase.
//   - Wait states: when HREADY is low, the master holds address, control and
//     write data unchanged.
//   - ERROR: in the first ERROR cycle (HREADY=0, HRESP=1) the master cancels
//     the remaining beats of the burst by driving HTRANS = IDLE.
// =============================================================================
`include "ahb_defines.vh"

`timescale 1ns / 1ps

module ahb_master (
    input  wire        HCLK,
    input  wire        HRESETn,

    // ---------------- command interface ----------------
    input  wire        start,          // 1-cycle pulse to start a command
    input  wire        cmd_write,      // 1 = write, 0 = read
    input  wire [31:0] cmd_addr,       // start address (aligned to cmd_size)
    input  wire [2:0]  cmd_size,       // HSIZE
    input  wire [2:0]  cmd_burst,      // HBURST
    input  wire [4:0]  cmd_len,        // number of beats, only used for INCR
    output wire        busy,
    output reg         done,           // 1-cycle pulse when the command ends
    output reg         error,          // an ERROR response was received

    // ---------------- data buffers ----------------
    input  wire        buf_we,         // write into the write buffer
    input  wire [3:0]  buf_idx,        // beat index for buffer access
    input  wire [31:0] buf_wdata,
    output wire [31:0] buf_rdata,      // read buffer output

    // ---------------- AHB-Lite master interface ----------------
    output reg  [31:0] HADDR,
    output reg  [1:0]  HTRANS,
    output reg         HWRITE,
    output reg  [2:0]  HSIZE,
    output reg  [2:0]  HBURST,
    output reg  [31:0] HWDATA,
    input  wire [31:0] HRDATA,
    input  wire        HREADY,
    input  wire        HRESP
);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam S_IDLE = 1'b0;   // waiting for a command
    localparam S_BUSY = 1'b1;   // transfer in progress

    reg state;

    // -------------------------------------------------------------------------
    // Internal registers
    // -------------------------------------------------------------------------
    reg [31:0] wbuf [0:15];     // write data, one word per beat
    reg [31:0] rbuf [0:15];     // read data, one word per beat

    reg [4:0]  total_beats;     // beats in the current command
    reg [4:0]  addr_beat;       // beat number currently in the ADDRESS phase

    reg        data_valid;      // a beat is currently in the DATA phase
    reg        data_write;      // ... and it is a write
    reg [4:0]  data_beat;       // ... and this is its beat number
    reg        data_last;       // ... and it is the last beat of the command

    wire addr_active = HTRANS[1];                     // NONSEQ or SEQ
    wire addr_last   = (addr_beat == total_beats - 1'b1);

    assign busy      = (state == S_BUSY);
    assign buf_rdata = rbuf[buf_idx];

    // -------------------------------------------------------------------------
    // Helper functions
    // -------------------------------------------------------------------------

    // Number of beats for a burst type
    function [4:0] burst_beats;
        input [2:0] burst;
        input [4:0] len;
        begin
            case (burst)
                `HBURST_SINGLE                 : burst_beats = 5'd1;
                `HBURST_WRAP4,  `HBURST_INCR4  : burst_beats = 5'd4;
                `HBURST_WRAP8,  `HBURST_INCR8  : burst_beats = 5'd8;
                `HBURST_WRAP16, `HBURST_INCR16 : burst_beats = 5'd16;
                default : // INCR
                    burst_beats = (len == 0) ? 5'd1 : (len > 16) ? 5'd16 : len;
            endcase
        end
    endfunction

    // Address of the next beat.
    //   INCR : address + transfer size
    //   WRAP : address + transfer size, but wrap inside a window of
    //          (beats x size) bytes. Example WRAP4 word from 0x38:
    //          0x38 -> 0x3C -> 0x30 -> 0x34
    function [31:0] next_addr;
        input [31:0] addr;
        input [2:0]  size;
        input [2:0]  burst;
        reg   [31:0] bytes, window;
        begin
            bytes = 32'd1 << size;
            case (burst)
                `HBURST_WRAP4  : window = bytes << 2;
                `HBURST_WRAP8  : window = bytes << 3;
                `HBURST_WRAP16 : window = bytes << 4;
                default        : window = 32'd0;          // not wrapping
            endcase

            if (window == 0)
                next_addr = addr + bytes;
            else
                next_addr = (addr & ~(window - 1)) | ((addr + bytes) & (window - 1));
        end
    endfunction

    // -------------------------------------------------------------------------
    // Data buffers
    // -------------------------------------------------------------------------
    // write buffer: loaded by the user while the master is idle
    always @(posedge HCLK) begin
        if (buf_we && !busy)
            wbuf[buf_idx] <= buf_wdata;
    end

    // read buffer: store HRDATA when a read beat completes
    always @(posedge HCLK) begin
        if (busy && HREADY && data_valid && !data_write)
            rbuf[data_beat[3:0]] <= HRDATA;
    end

    // -------------------------------------------------------------------------
    // Main control
    // -------------------------------------------------------------------------
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            state       <= S_IDLE;
            HADDR       <= 32'd0;
            HTRANS      <= `HTRANS_IDLE;
            HWRITE      <= 1'b0;
            HSIZE       <= `HSIZE_WORD;
            HBURST      <= `HBURST_SINGLE;
            HWDATA      <= 32'd0;
            total_beats <= 5'd1;
            addr_beat   <= 5'd0;
            data_valid  <= 1'b0;
            data_write  <= 1'b0;
            data_beat   <= 5'd0;
            data_last   <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
        end else begin
            done <= 1'b0;   // default: 'done' is a single-cycle pulse

            case (state)
                // -------------------------------------------------------------
                S_IDLE: begin
                    if (start) begin
                        // first beat goes into the address phase
                        HADDR       <= cmd_addr;
                        HTRANS      <= `HTRANS_NONSEQ;
                        HWRITE      <= cmd_write;
                        HSIZE       <= cmd_size;
                        HBURST      <= cmd_burst;
                        total_beats <= burst_beats(cmd_burst, cmd_len);
                        addr_beat   <= 5'd0;
                        error       <= 1'b0;
                        state       <= S_BUSY;
                    end
                end

                // -------------------------------------------------------------
                S_BUSY: begin
                    if (HREADY) begin
                        // (1) the beat in the data phase completes now
                        //     (read data is captured in the read buffer above)
                        if (data_valid && data_last) begin
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end

                        // (2) the address phase moves into the data phase
                        data_valid <= addr_active;
                        data_write <= HWRITE;
                        data_beat  <= addr_beat;
                        data_last  <= addr_last;
                        HWDATA     <= wbuf[addr_beat[3:0]];

                        // (3) put the next beat in the address phase
                        if (addr_active && !addr_last) begin
                            HTRANS    <= `HTRANS_SEQ;
                            HADDR     <= next_addr(HADDR, HSIZE, HBURST);
                            addr_beat <= addr_beat + 1'b1;
                        end else begin
                            HTRANS    <= `HTRANS_IDLE;
                        end
                    end
                    else if (data_valid && HRESP) begin
                        // first cycle of a two-cycle ERROR response:
                        // cancel the rest of the burst
                        error     <= 1'b1;
                        data_last <= 1'b1;
                        HTRANS    <= `HTRANS_IDLE;
                    end
                    // else: wait state -> hold everything
                end
            endcase
        end
    end

endmodule
