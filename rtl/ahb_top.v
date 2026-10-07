// =============================================================================
// Module      : ahb_top
// Description : Complete AHB-Lite system.
//
//                          +------------+
//      command / data ---->| ahb_master |
//                          +------------+
//                            |  HADDR, HTRANS, HWRITE, HSIZE, HBURST, HWDATA
//          +-----------------+-----------------+------------------+
//          |                 |                 |                  |
//    +-----------+    +-------------+   +-------------+  +-------------------+
//    |ahb_decoder|    |  ahb_slave  |   |  ahb_slave  |  | ahb_default_slave |
//    +-----------+    |  S0: 0 wait |   |  S1: 2 wait |  |  ERROR response   |
//          | HSEL     +-------------+   +-------------+  +-------------------+
//          v                 |                 |                  |
//    +--------------------------------------------------------------------+
//    |   ahb_mux : HRDATA / HREADY / HRESP of the selected slave           |
//    +--------------------------------------------------------------------+
//                            |  back to master (HREADY also to all slaves)
// =============================================================================
`include "ahb_defines.vh"

`timescale 1ns / 1ps

module ahb_top (
    input  wire        HCLK,
    input  wire        HRESETn,

    // command interface (to the master)
    input  wire        start,
    input  wire        cmd_write,
    input  wire [31:0] cmd_addr,
    input  wire [2:0]  cmd_size,
    input  wire [2:0]  cmd_burst,
    input  wire [4:0]  cmd_len,
    output wire        busy,
    output wire        done,
    output wire        error,

    // data buffers (in the master)
    input  wire        buf_we,
    input  wire [3:0]  buf_idx,
    input  wire [31:0] buf_wdata,
    output wire [31:0] buf_rdata
);

    // ---------------- shared AHB bus ----------------
    wire [31:0] HADDR, HWDATA, HRDATA;
    wire [1:0]  HTRANS;
    wire        HWRITE;
    wire [2:0]  HSIZE, HBURST;
    wire        HREADY, HRESP;

    // ---------------- slave select and responses ----------------
    wire        HSEL_S0, HSEL_S1, HSEL_DEF;
    wire [31:0] HRDATA_S0, HRDATA_S1, HRDATA_DEF;
    wire        HREADYOUT_S0, HREADYOUT_S1, HREADYOUT_DEF;
    wire        HRESP_S0, HRESP_S1, HRESP_DEF;

    // ---------------- master ----------------
    ahb_master u_master (
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
        .buf_rdata (buf_rdata),
        .HADDR     (HADDR),
        .HTRANS    (HTRANS),
        .HWRITE    (HWRITE),
        .HSIZE     (HSIZE),
        .HBURST    (HBURST),
        .HWDATA    (HWDATA),
        .HRDATA    (HRDATA),
        .HREADY    (HREADY),
        .HRESP     (HRESP)
    );

    // ---------------- decoder ----------------
    ahb_decoder u_decoder (
        .HADDR    (HADDR),
        .HSEL_S0  (HSEL_S0),
        .HSEL_S1  (HSEL_S1),
        .HSEL_DEF (HSEL_DEF)
    );

    // ---------------- slave 0 : no wait states ----------------
    ahb_slave #(.WAIT_STATES(0)) u_slave0 (
        .HCLK      (HCLK),
        .HRESETn   (HRESETn),
        .HSEL      (HSEL_S0),
        .HADDR     (HADDR),
        .HTRANS    (HTRANS),
        .HWRITE    (HWRITE),
        .HSIZE     (HSIZE),
        .HWDATA    (HWDATA),
        .HREADY    (HREADY),
        .HREADYOUT (HREADYOUT_S0),
        .HRESP     (HRESP_S0),
        .HRDATA    (HRDATA_S0)
    );

    // ---------------- slave 1 : 2 wait states ----------------
    ahb_slave #(.WAIT_STATES(2)) u_slave1 (
        .HCLK      (HCLK),
        .HRESETn   (HRESETn),
        .HSEL      (HSEL_S1),
        .HADDR     (HADDR),
        .HTRANS    (HTRANS),
        .HWRITE    (HWRITE),
        .HSIZE     (HSIZE),
        .HWDATA    (HWDATA),
        .HREADY    (HREADY),
        .HREADYOUT (HREADYOUT_S1),
        .HRESP     (HRESP_S1),
        .HRDATA    (HRDATA_S1)
    );

    // ---------------- default slave : ERROR ----------------
    ahb_default_slave u_default_slave (
        .HCLK      (HCLK),
        .HRESETn   (HRESETn),
        .HSEL      (HSEL_DEF),
        .HTRANS    (HTRANS),
        .HREADY    (HREADY),
        .HREADYOUT (HREADYOUT_DEF),
        .HRESP     (HRESP_DEF),
        .HRDATA    (HRDATA_DEF)
    );

    // ---------------- multiplexer ----------------
    ahb_mux u_mux (
        .HCLK          (HCLK),
        .HRESETn       (HRESETn),
        .HSEL_S0       (HSEL_S0),
        .HSEL_S1       (HSEL_S1),
        .HSEL_DEF      (HSEL_DEF),
        .HRDATA_S0     (HRDATA_S0),
        .HREADYOUT_S0  (HREADYOUT_S0),
        .HRESP_S0      (HRESP_S0),
        .HRDATA_S1     (HRDATA_S1),
        .HREADYOUT_S1  (HREADYOUT_S1),
        .HRESP_S1      (HRESP_S1),
        .HRDATA_DEF    (HRDATA_DEF),
        .HREADYOUT_DEF (HREADYOUT_DEF),
        .HRESP_DEF     (HRESP_DEF),
        .HRDATA        (HRDATA),
        .HREADY        (HREADY),
        .HRESP         (HRESP)
    );

endmodule
