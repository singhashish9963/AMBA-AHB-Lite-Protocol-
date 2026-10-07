// =============================================================================
// Module      : ahb_decoder
// Description : Address decoder. Looks at HADDR during the address phase and
//               selects exactly one slave.
//
//   0x0000_0000 - 0x0000_0FFF : Slave 0
//   0x0000_1000 - 0x0000_1FFF : Slave 1
//   anything else             : Default slave (returns ERROR)
// =============================================================================
`include "ahb_defines.vh"

`timescale 1ns / 1ps

module ahb_decoder (
    input  wire [31:0] HADDR,
    output wire        HSEL_S0,
    output wire        HSEL_S1,
    output wire        HSEL_DEF
);

    assign HSEL_S0  = (HADDR >= `SLAVE0_BASE) && (HADDR < `SLAVE0_BASE + `SLAVE_SIZE);
    assign HSEL_S1  = (HADDR >= `SLAVE1_BASE) && (HADDR < `SLAVE1_BASE + `SLAVE_SIZE);
    assign HSEL_DEF = !(HSEL_S0 || HSEL_S1);

endmodule
