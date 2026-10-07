// =============================================================================
// Module      : ahb_slave
// Description : AHB-Lite memory slave (4 KB by default).
//
//   - Captures the address phase when it is selected and HREADY is high.
//   - In the data phase it can insert WAIT_STATES cycles (HREADYOUT = 0).
//   - Writes use byte lanes, so byte / half-word / word writes only change
//     the addressed bytes (little-endian).
//   - Always answers OKAY.
// =============================================================================
`include "ahb_defines.vh"

`timescale 1ns / 1ps

module ahb_slave #(
    parameter ADDR_WIDTH  = 12,   // 2**12 = 4 KB
    parameter WAIT_STATES = 0     // wait cycles added to every transfer
) (
    input  wire        HCLK,
    input  wire        HRESETn,
    input  wire        HSEL,
    input  wire [31:0] HADDR,
    input  wire [1:0]  HTRANS,
    input  wire        HWRITE,
    input  wire [2:0]  HSIZE,
    input  wire [31:0] HWDATA,
    input  wire        HREADY,
    output wire        HREADYOUT,
    output wire        HRESP,
    output wire [31:0] HRDATA
);

    localparam WORDS = 1 << (ADDR_WIDTH - 2);

    reg [31:0] mem [0:WORDS-1];

    // information captured from the address phase
    reg                  dp_active;     // this slave owns the data phase
    reg                  dp_write;
    reg [ADDR_WIDTH-1:0] dp_addr;
    reg [2:0]            dp_size;
    reg [3:0]            wait_cnt;      // remaining wait states

    wire [ADDR_WIDTH-3:0] word_idx = dp_addr[ADDR_WIDTH-1:2];
    wire                  dp_done  = dp_active && (wait_cnt == 0);
    wire [3:0]            byte_en;

    integer b;   // byte-lane loop index
    integer i;   // memory init loop index

    // -------------------------------------------------------------------------
    // Byte enables from size and the two low address bits
    //   byte : 0001 shifted by addr[1:0]
    //   half : 0011 shifted by 0 or 2
    //   word : 1111
    // -------------------------------------------------------------------------
    assign byte_en = (dp_size == `HSIZE_BYTE) ? (4'b0001 << dp_addr[1:0]) :
                     (dp_size == `HSIZE_HALF) ? (4'b0011 << {dp_addr[1], 1'b0}) :
                                                4'b1111;

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------
    assign HREADYOUT = !(dp_active && wait_cnt != 0);   // low while waiting
    assign HRESP     = `HRESP_OKAY;
    assign HRDATA    = mem[word_idx];

    // -------------------------------------------------------------------------
    // Address phase capture and wait-state counter
    // -------------------------------------------------------------------------
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            dp_active <= 1'b0;
            dp_write  <= 1'b0;
            dp_addr   <= 0;
            dp_size   <= 3'd0;
            wait_cnt  <= 4'd0;
        end else if (dp_active && wait_cnt != 0) begin
            wait_cnt <= wait_cnt - 1'b1;                // still waiting
        end else if (HREADY) begin
            dp_active <= HSEL && HTRANS[1];             // NONSEQ or SEQ for us
            dp_write  <= HWRITE;
            dp_addr   <= HADDR[ADDR_WIDTH-1:0];
            dp_size   <= HSIZE;
            wait_cnt  <= WAIT_STATES;
        end
    end

    // -------------------------------------------------------------------------
    // Memory write at the end of the data phase (HWDATA is valid now)
    // -------------------------------------------------------------------------
    always @(posedge HCLK) begin
        if (HRESETn && dp_done && dp_write) begin
            for (b = 0; b < 4; b = b + 1)
                if (byte_en[b])
                    mem[word_idx][8*b +: 8] <= HWDATA[8*b +: 8];
        end
    end

    // clear the memory at start of simulation
    initial begin
        for (i = 0; i < WORDS; i = i + 1)
            mem[i] = 32'd0;
    end

endmodule
