# AMBA AHB-Lite Protocol — RTL Design & Verification (Verilog)

A complete **AMBA AHB-Lite bus system** in Verilog: a pipelined **master**,
an **address decoder**, a **slave multiplexer**, two **memory slaves** and a
**default slave**. It is verified with a **self-checking testbench** that
includes a reference memory model and bus **protocol checkers**.

Based on the ARM *AMBA 3 AHB-Lite Protocol Specification* (IHI 0033).

---

## Table of Contents
1. [Features](#features)
2. [System Architecture](#system-architecture)
3. [Directory Structure](#directory-structure)
4. [Module Description](#module-description)
5. [AHB-Lite Protocol Basics](#ahb-lite-protocol-basics)
6. [Master Command Interface](#master-command-interface)
7. [Verification](#verification)
8. [Simulation Results](#simulation-results)
9. [How to Run](#how-to-run)
10. [Limitations and Future Work](#limitations-and-future-work)

---

## Features

- **Pipelined** address and data phases
- Transfer types: **IDLE, NONSEQ, SEQ**
- Burst types: **SINGLE, INCR (1–16 beats), INCR4/8/16, WRAP4/8/16**
- Transfer sizes: **byte, half-word, word** (`HSIZE`) with byte-lane writes
- **Wait states** using `HREADY` (slave 1 adds 2 wait cycles to every transfer)
- **Two-cycle ERROR response** (`HRESP`) from the default slave, with **burst cancellation** by the master
- Address **decoder** and data-phase **multiplexer** for multiple slaves
- Self-checking testbench with **reference memory**, **protocol checkers**,
  **directed + random tests** and **coverage counters**

---

## System Architecture

```
                     start / cmd_* / buffers
                               │
                        ┌──────▼──────┐
                        │ ahb_master  │
                        └──────┬──────┘
         HADDR, HTRANS, HWRITE, HSIZE, HBURST, HWDATA (shared bus)
       ┌───────────────┬───────┴───────┬────────────────────┐
       ▼               ▼               ▼                    ▼
 ┌───────────┐  ┌─────────────┐ ┌─────────────┐  ┌───────────────────┐
 │ahb_decoder│  │  ahb_slave  │ │  ahb_slave  │  │ ahb_default_slave │
 │           │  │  Slave 0    │ │  Slave 1    │  │ (unmapped address)│
 │  HADDR →  │  │  0 waits    │ │  2 waits    │  │  ERROR response   │
 │  HSEL_x   │  └──────┬──────┘ └──────┬──────┘  └─────────┬─────────┘
 └─────┬─────┘         │ HRDATA, HREADYOUT, HRESP           │
       │ HSEL          ▼               ▼                    ▼
       │        ┌────────────────────────────────────────────────┐
       └───────►│ ahb_mux (select registered for the data phase) │
                └───────────────────────┬────────────────────────┘
                                        │ HRDATA, HREADY, HRESP
                                        ▼
                              back to master (HREADY also to all slaves)
```

### Address Map

| Address range | Slave | Behaviour |
|---|---|---|
| `0x0000_0000 – 0x0000_0FFF` | Slave 0 | 4 KB memory, **0 wait states** |
| `0x0000_1000 – 0x0000_1FFF` | Slave 1 | 4 KB memory, **2 wait states** |
| anything else | Default slave | **ERROR** response |

---

## Directory Structure

```
AMBA-AHB-Lite-Protocol-/
├── rtl/
│   ├── ahb_defines.vh       # HTRANS / HBURST / HSIZE / HRESP codes, address map
│   ├── ahb_top.v            # Top level – connects the whole system
│   ├── ahb_master.v         # Pipelined AHB-Lite master (FSM)
│   ├── ahb_decoder.v        # Address decoder (HSEL generation)
│   ├── ahb_mux.v            # Slave-to-master response multiplexer
│   ├── ahb_slave.v          # Memory slave with wait states and byte lanes
│   └── ahb_default_slave.v  # Two-cycle ERROR response for unmapped addresses
├── tb/
│   └── tb_ahb.v             # Self-checking testbench
└── readme.md
```

---

## Module Description

| Module | Description |
|---|---|
| `ahb_master` | Two-state FSM (`IDLE`, `BUSY`). Takes a command and generates NONSEQ/SEQ beats with correct INCR/WRAP addresses. Holds the bus during wait states and cancels the burst on ERROR. Has a 16-word write buffer and a 16-word read buffer. |
| `ahb_decoder` | Combinational. Turns `HADDR` into one-hot `HSEL_S0`, `HSEL_S1`, `HSEL_DEF`. |
| `ahb_mux` | Registers the slave select when `HREADY` is high, so the response comes from the slave that owns the **data phase**. Routes `HRDATA`, `HREADY`, `HRESP` to the master. |
| `ahb_slave` | Captures the address phase, inserts `WAIT_STATES` cycles, writes only the enabled byte lanes, always answers OKAY. |
| `ahb_default_slave` | Answers IDLE with OKAY. Answers NONSEQ/SEQ with the two-cycle ERROR response. |
| `ahb_top` | Instantiates and connects all modules. |

---

## AHB-Lite Protocol Basics

### Pipelined transfers
Every transfer has an **address phase** followed by a **data phase**. The
address of the next beat is driven **during** the data phase of the current beat.

```
HCLK      __|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
HTRANS      | NONSEQ| SEQ  | SEQ  | SEQ  | IDLE |
HADDR       |  A0   |  A1  |  A2  |  A3  |      |
HWDATA/     |       |  D0  |  D1  |  D2  |  D3  |
HRDATA
            └ addr  └ data A0 / addr A1 ...
```

### Burst address rules
| Burst | Next address |
|---|---|
| INCR / INCR4/8/16 | `addr + size` |
| WRAP4/8/16 | `addr + size`, wrapping inside a window of `beats × size` bytes |

Example: **WRAP4, word, start 0x38**. The window is 4 × 4 = 16 bytes, so the
addresses are `0x38 → 0x3C → 0x30 → 0x34`.

Incrementing bursts must **not cross a 1 KB boundary**. The testbench
respects this rule when it generates random commands.

### Wait states
A slave drives `HREADY = 0` to extend the data phase. The master must keep
address, control and write data **unchanged** until `HREADY = 1`.

### Two-cycle ERROR response
```
               cycle 1          cycle 2
HREADY           0                1
HRESP          ERROR            ERROR
master action  cancel burst     transfer ends
               (HTRANS = IDLE)
```
The first cycle gives the master time to cancel the remaining beats of the burst.

---

## Master Command Interface

| Signal | Dir | Description |
|---|---|---|
| `start` | in | One-cycle pulse to start a command |
| `cmd_write` | in | 1 = write, 0 = read |
| `cmd_addr[31:0]` | in | Start address (aligned to `cmd_size`) |
| `cmd_size[2:0]` | in | `HSIZE` – byte / half-word / word |
| `cmd_burst[2:0]` | in | `HBURST` – SINGLE, INCR, WRAP4 … INCR16 |
| `cmd_len[4:0]` | in | Number of beats (only for INCR) |
| `busy` | out | Command in progress |
| `done` | out | One-cycle pulse when the command finishes |
| `error` | out | An ERROR response was received |
| `buf_we`, `buf_idx`, `buf_wdata` | in | Load write data for each beat (before `start`) |
| `buf_rdata` | out | Read data of beat `buf_idx` (after `done`) |

**Example write burst:** load 4 words into the buffer, then pulse `start` with
`cmd_burst = INCR4`. The master drives NONSEQ followed by 3 × SEQ and pulses
`done` when the last data phase completes.

---

## Verification

The testbench (`tb/tb_ahb.v`) is **self-checking** and prints
`TEST PASSED` or `TEST FAILED`.

```
 ┌───────────────┐  command   ┌─────────┐   AHB bus   ┌──────────────────────┐
 │ run_command() │──────────► │ ahb_top │ ──────────► │ Protocol checkers    │
 │ directed +    │            │  (DUT)  │             │ (every clock cycle)  │
 │ random tests  │◄────────── │         │             └──────────────────────┘
 └──────┬────────┘ done/data  └─────────┘
        │
        ▼
 ┌──────────────────────────────────┐
 │ Reference memory (byte array)    │  updated on writes, compared on reads
 └──────────────────────────────────┘
```

### Checks after every command
| Check | Description |
|---|---|
| **Read data** | Every byte read is compared with the reference memory. |
| **Error flag** | Must be set for unmapped addresses only. |
| **Beats on bus** | Must equal the burst length. Must be **1** for an ERROR, which proves the burst was cancelled. |

### Protocol checkers (every clock cycle)
| Checker | Rule |
|---|---|
| Control stable in wait | `HADDR`, `HTRANS`, `HWRITE`, `HSIZE`, `HBURST` do not change while `HREADY = 0` |
| Write data stable in wait | `HWDATA` does not change while a write data phase is stalled |
| Two-cycle ERROR | `(HREADY=0, ERROR)` is always followed by `(HREADY=1, ERROR)`, and never appears without it |
| Burst address | Every SEQ address follows the INCR / WRAP rule |

### Test plan
| Test | What is tested |
|---|---|
| 1. Single transfers | Word write/read on slave 0 and on slave 1 (with wait states) |
| 2. Byte / half-word | Partial writes only change the addressed bytes. Byte INCR4 burst. |
| 3. All burst types | INCR, INCR4/8/16, WRAP4/8/16 write + read-back. Start address `0x..38`, so WRAP bursts actually wrap. |
| 4. Error responses | Read and INCR4 write to an unmapped address. The burst must be cancelled, and the bus must keep working afterwards. |
| 5. Random | 300 random commands: random slave (10 % unmapped), size, burst, length and direction |

---

## Simulation Results

Output of the full regression (Icarus Verilog 13):

```
[50000] Test 1: single transfers
[296000] Test 2: byte / half-word transfers
[949000] Test 3: all burst types
[5181000] Test 4: error responses
[5529000] Test 5: random commands

==================== TEST SUMMARY ====================
 Commands run               : 334
 Data mismatches            : 0
 Command check failures     : 0
 Protocol check failures    : 0
------------------------------------------------------
 Coverage
   SINGLE/INCR              : 50 / 49
   WRAP4/INCR4              : 37 / 41
   WRAP8/INCR8              : 50 / 31
   WRAP16/INCR16            : 40 / 36
   byte/half/word           : 119 / 104 / 111
   wait-state cycles        : 1848
   ERROR responses          : 25
   address wrap-arounds     : 110
------------------------------------------------------
 RESULT : *** TEST PASSED ***
======================================================
```

The testbench was also checked against **deliberately broken designs**, and it
reports failures for:
- a master that **does not cancel** a burst on ERROR (caught by the beat-count check and the burst address checker);
- **WRAP bursts treated as INCR** (caught by the burst address checker);
- a slave that **ignores byte lanes** (caught by the read-data check).

---

## How to Run

### Option 1 — Xilinx Vivado (xsim)
1. **Create Project** → RTL Project → *Do not specify sources at this time*.
2. **Add Sources → Add or create design sources** → add all files in `rtl/`
   (including `ahb_defines.vh`).
3. **Add Sources → Add or create simulation sources** → add `tb/tb_ahb.v`.
4. Right-click `tb_ahb` → **Set as Top**.
5. **Run Simulation → Run Behavioral Simulation**, then type `run all` in the Tcl console.

> If Vivado cannot find `ahb_defines.vh`, select it in the Sources window and
> set **File Properties → Global Include**, or add the `rtl` folder under
> *Settings → General → Verilog options → Include Directories*.

Useful waveform signals: `dut/HADDR`, `dut/HTRANS`, `dut/HBURST`, `dut/HREADY`,
`dut/HRESP`, `dut/HWDATA`, `dut/HRDATA`, `dut/u_master/state`.

### Option 2 — Icarus Verilog + GTKWave
```bash
iverilog -g2005 -Wall -I rtl -o ahb_sim rtl/*.v tb/tb_ahb.v
vvp ahb_sim
gtkwave tb_ahb.vcd
```

### Option 3 — EDA Playground
Add `ahb_defines.vh` as a separate file, paste the remaining `rtl/` files
into **design.sv** and `tb/tb_ahb.v` into **testbench.sv**, then click **Run**.

---

## Limitations and Future Work

- The master runs **one command at a time**. A new command starts after the
  previous one is done, so the slave select in the mux never changes in the
  middle of a stalled data phase in this setup.
- `BUSY` transfers, `HPROT`, `HMASTLOCK` and multiple masters are not implemented.
- Wait states are fixed per slave.
- Possible extensions:
  - **AHB-to-APB bridge** with APB peripherals;
  - overlapping back-to-back commands in the master;
  - random wait states in the slaves;
  - a SystemVerilog/UVM testbench with SVA assertions and functional coverage.

---

## References
- ARM, *AMBA 3 AHB-Lite Protocol Specification* (IHI 0033A).

---

**Author:** Ashish Singh — B.Tech ECE, MNNIT Allahabad
