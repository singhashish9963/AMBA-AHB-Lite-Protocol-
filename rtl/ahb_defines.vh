// =============================================================================
// File        : ahb_defines.vh
// Description : AMBA AHB-Lite signal encodings and the system address map.
//               Included by every RTL file and the testbench.
// =============================================================================
`ifndef AHB_DEFINES_VH
`define AHB_DEFINES_VH

// ---------------- HTRANS : transfer type ----------------
`define HTRANS_IDLE    2'b00   // no transfer
`define HTRANS_BUSY    2'b01   // pause inside a burst (not used by this master)
`define HTRANS_NONSEQ  2'b10   // first beat of a transfer / burst
`define HTRANS_SEQ     2'b11   // remaining beats of a burst

// ---------------- HBURST : burst type ----------------
`define HBURST_SINGLE  3'b000  // single transfer
`define HBURST_INCR    3'b001  // incrementing, undefined length
`define HBURST_WRAP4   3'b010  // 4-beat wrapping
`define HBURST_INCR4   3'b011  // 4-beat incrementing
`define HBURST_WRAP8   3'b100  // 8-beat wrapping
`define HBURST_INCR8   3'b101  // 8-beat incrementing
`define HBURST_WRAP16  3'b110  // 16-beat wrapping
`define HBURST_INCR16  3'b111  // 16-beat incrementing

// ---------------- HSIZE : transfer size ----------------
`define HSIZE_BYTE     3'b000  // 8 bits
`define HSIZE_HALF     3'b001  // 16 bits
`define HSIZE_WORD     3'b010  // 32 bits

// ---------------- HRESP : transfer response ----------------
`define HRESP_OKAY     1'b0
`define HRESP_ERROR    1'b1

// ---------------- Address map ----------------
//   0x0000_0000 - 0x0000_0FFF : Slave 0 (memory, no wait states)
//   0x0000_1000 - 0x0000_1FFF : Slave 1 (memory, with wait states)
//   anything else             : Default slave (ERROR response)
`define SLAVE0_BASE    32'h0000_0000
`define SLAVE1_BASE    32'h0000_1000
`define SLAVE_SIZE     32'h0000_1000   // 4 KB per slave

`endif
