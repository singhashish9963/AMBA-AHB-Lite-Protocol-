// =============================================================================
// Module      : ahb_mux
// Description : Slave-to-master multiplexer for HRDATA, HREADY and HRESP.
//
//   The decoder selects a slave in the ADDRESS phase, but that slave answers
//   in the following DATA phase. So the select signals are registered (only
//   when HREADY is high, i.e. when the address phase is accepted) and the
//   registered version drives the multiplexer.
// =============================================================================
`timescale 1ns / 1ps

module ahb_mux (
    input  wire        HCLK,
    input  wire        HRESETn,

    // select signals from the decoder (address phase)
    input  wire        HSEL_S0,
    input  wire        HSEL_S1,
    input  wire        HSEL_DEF,

    // slave responses
    input  wire [31:0] HRDATA_S0,
    input  wire        HREADYOUT_S0,
    input  wire        HRESP_S0,

    input  wire [31:0] HRDATA_S1,
    input  wire        HREADYOUT_S1,
    input  wire        HRESP_S1,

    input  wire [31:0] HRDATA_DEF,
    input  wire        HREADYOUT_DEF,
    input  wire        HRESP_DEF,

    // to the master (and back to all slaves as HREADY)
    output reg  [31:0] HRDATA,
    output reg         HREADY,
    output reg         HRESP
);

    // which slave owns the current data phase (one-hot)
    reg sel_s0, sel_s1, sel_def;

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            sel_s0  <= 1'b0;
            sel_s1  <= 1'b0;
            sel_def <= 1'b1;     // default slave answers IDLE with OKAY
        end else if (HREADY) begin
            sel_s0  <= HSEL_S0;
            sel_s1  <= HSEL_S1;
            sel_def <= HSEL_DEF;
        end
    end

    always @(*) begin
        if (sel_s0) begin
            HRDATA = HRDATA_S0;
            HREADY = HREADYOUT_S0;
            HRESP  = HRESP_S0;
        end else if (sel_s1) begin
            HRDATA = HRDATA_S1;
            HREADY = HREADYOUT_S1;
            HRESP  = HRESP_S1;
        end else begin
            HRDATA = HRDATA_DEF;
            HREADY = HREADYOUT_DEF;
            HRESP  = HRESP_DEF;
        end
    end

endmodule
