// =============================================================================
// Module      : ahb_default_slave
// Description : Selected when the address does not belong to any slave.
//
//   IDLE transfer        -> OKAY, no wait states
//   NONSEQ / SEQ access  -> two-cycle ERROR response
//
//        cycle 1 : HREADYOUT = 0, HRESP = ERROR   (master may cancel the burst)
//        cycle 2 : HREADYOUT = 1, HRESP = ERROR   (transfer ends)
// =============================================================================
`include "ahb_defines.vh"

`timescale 1ns / 1ps

module ahb_default_slave (
    input  wire        HCLK,
    input  wire        HRESETn,
    input  wire        HSEL,
    input  wire [1:0]  HTRANS,
    input  wire        HREADY,
    output wire        HREADYOUT,
    output wire        HRESP,
    output wire [31:0] HRDATA
);

    localparam S_OKAY = 2'd0;
    localparam S_ERR1 = 2'd1;   // first ERROR cycle
    localparam S_ERR2 = 2'd2;   // second ERROR cycle

    reg [1:0] state;

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn)
            state <= S_OKAY;
        else begin
            case (state)
                S_ERR1  : state <= S_ERR2;
                default : // S_OKAY or S_ERR2: check for a new access
                    state <= (HSEL && HREADY && HTRANS[1]) ? S_ERR1 : S_OKAY;
            endcase
        end
    end

    assign HREADYOUT = (state != S_ERR1);
    assign HRESP     = (state == S_OKAY) ? `HRESP_OKAY : `HRESP_ERROR;
    assign HRDATA    = 32'd0;

endmodule
