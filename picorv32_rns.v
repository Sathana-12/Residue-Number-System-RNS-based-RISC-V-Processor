/*
 *  PicoRV32 -- A Small RISC-V (RV32I) Processor Core
 *  RNS-Extended Version: 64-bit ADD/SUB via Residue Number System
 *
 *  Original Copyright (C) 2015  Claire Xenia Wolf <claire@yosyshq.com>
 *  RNS Extension: Forward Converter, RNS ALU (add/sub), CRT Reverse Converter
 *
 *  RNS Moduli Set: {M1 = 2^32-1, M2 = 2^32+1, M3 = 2^31-1}
 *  Dynamic Range:  M1 * M2 * M3 > 2^95  (covers full 64-bit results)
 *
 *  Forward Converter  : placed immediately after operand read from register file
 *  RNS ALU            : performs add/sub on each residue channel independently
 *  Reverse Converter  : CRT-based reconstruction to 64-bit binary result
 *
 *  All non-ALU instructions (branches, loads, stores, shifts, etc.) pass through
 *  the existing 32-bit datapath unchanged.
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 */

/* verilator lint_off WIDTH */
/* verilator lint_off PINMISSING */
/* verilator lint_off CASEOVERLAP */
/* verilator lint_off CASEINCOMPLETE */

`timescale 1 ns / 1 ps

`ifdef DEBUG
  `define debug(debug_command) debug_command
`else
  `define debug(debug_command)
`endif

`ifdef FORMAL
  `define FORMAL_KEEP (* keep *)
  `define assert(assert_expr) assert(assert_expr)
`else
  `ifdef DEBUGNETS
    `define FORMAL_KEEP (* keep *)
  `else
    `define FORMAL_KEEP
  `endif
  `define assert(assert_expr) empty_statement
`endif

`define PICORV32_V

/*======================================================================
 *  RNS FORWARD CONVERTER
 *  Input : 64-bit binary X  (or zero-extended 32-bit)
 *  Output: three residues   x1 = X mod M1
 *                           x2 = X mod M2
 *                           x3 = X mod M3
 *
 *  M1 = 2^32 - 1   (Mersenne prime)
 *  M2 = 2^32 + 1   (Pseudo-Mersenne)
 *  M3 = 2^31 - 1   (Mersenne prime)
 *
 *  Efficient reduction:
 *    X mod (2^n - 1) : sum of n-bit blocks, then one conditional subtract
 *    X mod (2^n + 1) : alternating sum of n-bit blocks
 *    X mod (2^n - 1) : same as first with n=31
 *======================================================================*/
module rns_forward_converter (
    input  [63:0] X,
    output [31:0] r1,   // X mod M1  (2^32-1)
    output [32:0] r2,   // X mod M2  (2^32+1)  -- needs 33 bits to represent M2-1=2^32
    output [30:0] r3    // X mod M3  (2^31-1)
);
    // ---- r1 = X mod (2^32 - 1) ----
    // X = X_hi * 2^32 + X_lo
    // X mod (2^32-1): fold: sum = X_lo + X_hi, if sum >= M1 subtract M1
    wire [32:0] s1 = {1'b0, X[31:0]} + {1'b0, X[63:32]};
    // s1 may be up to 2*(2^32-1) = 2^33-2, so one more fold
    wire [32:0] s1b = s1[32] ? (s1 - 33'h1_FFFF_FFFF) : s1; // subtract M1 if carry
    assign r1 = s1b[31:0];

    // ---- r2 = X mod (2^32 + 1) ----
    // X = X_hi * 2^32 + X_lo
    // 2^32 ≡ -1 mod (2^32+1), so X ≡ X_lo - X_hi mod M2
    // Result may be negative: add M2 if so
    wire signed [33:0] d2 = {2'b0, X[31:0]} - {2'b0, X[63:32]};
    wire [33:0] d2_pos = d2[33] ? (d2 + 34'h2_0000_0001) : d2; // add M2
    assign r2 = d2_pos[32:0];

    // ---- r3 = X mod (2^31 - 1) ----
    // Fold 64-bit X into 31-bit chunks:
    // X = b[0:30] + b[31:61]*2^31 + b[62:63]*2^62
    // 2^31 ≡ 1 mod M3, so each 31-bit block adds directly
    wire [31:0] chunk0 = {1'b0, X[30: 0]};           // bits 30:0
    wire [31:0] chunk1 = {1'b0, X[61:31]};            // bits 61:31
    wire [ 1:0] chunk2 =        X[63:62];             // bits 63:62
    wire [32:0] sum3a  = {1'b0, chunk0} + {1'b0, chunk1};
    wire [32:0] sum3b  = sum3a + {31'b0, chunk2};
    // Fold once more (sum may be up to 3*(2^31-1) < 2^33)
    wire [31:0] fold3  = sum3b[31:0] + {30'b0, sum3b[32]};
    // One final conditional subtract
    wire [31:0] fold3b = (fold3 >= 32'h7FFF_FFFF) ? (fold3 - 32'h7FFF_FFFF) : fold3;
    assign r3 = fold3b[30:0];

endmodule


/*======================================================================
 *  RNS ALU  (add and subtract only)
 *  All arithmetic done channel-wise modulo each modulus.
 *
 *  For ADD:  ri_out = (ri_a + ri_b) mod Mi
 *  For SUB:  ri_out = (ri_a - ri_b + Mi) mod Mi
 *======================================================================*/
module rns_alu (
    input         is_sub,   // 1 = subtract, 0 = add
    // Residues of operand A
    input  [31:0] a1,
    input  [32:0] a2,
    input  [30:0] a3,
    // Residues of operand B
    input  [31:0] b1,
    input  [32:0] b2,
    input  [30:0] b3,
    // Result residues
    output [31:0] out1,
    output [32:0] out2,
    output [30:0] out3
);
    // ---- Channel 1: mod (2^32 - 1) ----
    wire [32:0] add1 = is_sub ? ({1'b0,a1} + {1'b1,~b1})   // a - b = a + ~b + 1; here use a + (M1-b)
                               : ({1'b0,a1} + {1'b0, b1});
    // For sub: (a1 - b1 + M1): compute a1 + (M1 - b1)
    wire [32:0] sub1_val = {1'b0,a1} + (33'h1_FFFF_FFFF - {1'b0,b1});
    wire [32:0] ch1_raw  = is_sub ? sub1_val : add1;
    // Reduce mod M1
    wire [32:0] ch1_fold = ch1_raw[32] ? (ch1_raw - 33'h1_FFFF_FFFF) : ch1_raw;
    assign out1 = ch1_fold[32] ? (ch1_fold[31:0] - 32'hFFFF_FFFE) : ch1_fold[31:0]; // safety second fold

    // ---- Channel 2: mod (2^32 + 1) ----
    wire [33:0] sub2_val = {1'b0,a2} + (34'h2_0000_0001 - {1'b0,b2});
    wire [33:0] add2_raw = {1'b0,a2} + {1'b0,b2};
    wire [33:0] ch2_raw  = is_sub ? sub2_val : add2_raw;
    wire [33:0] ch2_fold = (ch2_raw >= 34'h2_0000_0001) ? (ch2_raw - 34'h2_0000_0001) : ch2_raw;
    assign out2 = ch2_fold[32:0];

    // ---- Channel 3: mod (2^31 - 1) ----
    wire [31:0] sub3_val = {1'b0,a3} + (32'h7FFF_FFFF - {1'b0,b3});
    wire [31:0] add3_raw = {1'b0,a3} + {1'b0,b3};
    wire [31:0] ch3_raw  = is_sub ? sub3_val : add3_raw;
    wire [31:0] ch3_fold = (ch3_raw >= 32'h7FFF_FFFF) ? (ch3_raw - 32'h7FFF_FFFF) : ch3_raw;
    assign out3 = ch3_fold[30:0];

endmodule


/*======================================================================
 *  RNS REVERSE CONVERTER  (CRT-based, 64-bit output)
 *
 *  Reconstructs X from residues (r1, r2, r3) using the
 *  Mixed-Radix Conversion (MRC) technique -- a common hardware-friendly
 *  approach for this moduli set.
 *
 *  M1 = 2^32-1, M2 = 2^32+1, M3 = 2^31-1
 *  M  = M1*M2*M3  (dynamic range, ~2^95)
 *
 *  MRC coefficients (precomputed constants, all mod their respective Mi):
 *    a1 = r1
 *    a2 = (r2 - a1) * M1^{-1} mod M2
 *    a3 = ((r3 - a1) * M1^{-1} mod M3  - a2) * (M1*M2)^{-1} mod M3
 *
 *  Then X = a1 + a2*M1 + a3*M1*M2  (exact integer, truncated to 64 bits)
 *
 *  Precomputed inverses:
 *    M1^{-1} mod M2 = (2^32-1)^{-1} mod (2^32+1)
 *       2^32 ≡ -1 mod M2  => (2^32-1) ≡ -2 mod M2 => (-2)^{-1} mod M2
 *       (-2)^{-1} mod (2^32+1) = (2^32+1+1)/2 = (2^32+2)/2 = 2^31+1
 *    => inv_M1_M2 = 2^31 + 1 = 32'h8000_0001
 *
 *    M1^{-1} mod M3 = (2^32-1)^{-1} mod (2^31-1)
 *       2^32-1 = 2*(2^31-1)+1  => 2^32-1 ≡ 1 mod M3
 *    => inv_M1_M3 = 1
 *
 *    (M1*M2)^{-1} mod M3:  M1*M2 = (2^32-1)(2^32+1) = 2^64-1
 *       2^64-1 mod (2^31-1): since 2^31≡1 mod M3, 2^64 = (2^31)^2 * 2^2 ≡ 4,
 *       so 2^64-1 ≡ 3 mod M3
 *    => inv_M1M2_M3 = 3^{-1} mod (2^31-1) = (2*(2^31-1)+1)/3 = (2^32-1)/3
 *       (2^32-1)/3 = 0x5555_5555
 *    => inv_M1M2_M3 = 31'h5555_5555  (but mod 2^31-1 = 0x7FFF_FFFF)
 *       check: 3 * 0x5555_5555 = 0xFFFF_FFFF = 2^32-1 ≡ 1 mod M3  ✓
 *    => inv_M1M2_M3 = 31'h2AAA_AAAB  -- wait, let's recheck
 *       We need X s.t. 3X ≡ 1 mod (2^31-1=0x7FFF_FFFF)
 *       3 * 0x2AAA_AAAB = 0x8000_0001 = 2^31+1 = (2^31-1)+2 ≡ 2 mod M3  -- no
 *       Try extended Euclidean: gcd(3, 0x7FFF_FFFF)
 *       0x7FFF_FFFF = 3 * 0x2AAA_AAAA + 1  => 1 = 0x7FFF_FFFF - 3*0x2AAA_AAAA
 *       => 3^{-1} mod M3 = M3 - 0x2AAA_AAAA = 0x7FFF_FFFF - 0x2AAA_AAAA = 0x5555_5555
 *    => inv_M1M2_M3 = 31'h5555_5555   (verified: 3*0x5555_5555=0xFFFF_FFFF=2M3+1≡1 ✓)
 *
 *  Note: This reverse converter truncates to 64 bits. For correctly
 *  signed subtraction results, the caller should interpret reg_out64
 *  as a signed 64-bit value and truncate to 32 bits if needed.
 *======================================================================*/
module rns_reverse_converter (
    input  [31:0] r1,   // residue mod M1 = 2^32-1
    input  [32:0] r2,   // residue mod M2 = 2^32+1
    input  [30:0] r3,   // residue mod M3 = 2^31-1
    output [63:0] X     // reconstructed 64-bit value
);
    // Constants
    localparam [32:0] M1         = 33'h1_FFFF_FFFF; // 2^32-1  (stored in 33 bits)
    localparam [32:0] M2         = 33'h1_0000_0001; // 2^32+1
    localparam [30:0] M3         = 31'h7FFF_FFFF;   // 2^31-1
    localparam [32:0] INV_M1_M2  = 33'h0_8000_0001; // M1^{-1} mod M2
    localparam [30:0] INV_M1_M3  = 31'h0000_0001;   // M1^{-1} mod M3 = 1
    localparam [30:0] INV_M1M2_M3= 31'h5555_5555;   // (M1*M2)^{-1} mod M3

    // ---- Step 1: a1 = r1 ----
    wire [31:0] a1 = r1;

    // ---- Step 2: a2 = (r2 - a1) * INV_M1_M2 mod M2 ----
    // r2 - a1 mod M2:  a1 is in [0, M1=2^32-2], r2 in [0, M2-1=2^32]
    // Extend a1 to 33 bits and compute difference mod M2
    wire [33:0] diff2 = {1'b0, r2} - {2'b0, a1};  // may underflow
    wire [33:0] diff2_pos = diff2[33] ? (diff2 + {1'b0, M2}) : diff2;
    // diff2_pos is in [0, M2-1], fits 33 bits
    // Now multiply by INV_M1_M2 = 0x8000_0001 mod M2
    // (d * 0x8000_0001) mod (2^32+1)
    // d * 0x8000_0001 = d*2^31 + d
    // To do mod 2^32+1: split product
    wire [64:0] prod2_full = diff2_pos[32:0] * {32'b0, INV_M1_M2}; // 33*33=66b; cap at 65
    // Reduce mod M2 = 2^32+1:
    // prod = hi*2^32 + lo  =>  prod mod (2^32+1) = lo - hi + (add M2 if negative)
    wire [32:0] lo2  = prod2_full[32:0];
    wire [32:0] hi2  = {1'b0, prod2_full[64:33]}; // upper 32 bits (bit 64 is always 0 for valid inputs)
    wire signed [33:0] red2 = {1'b0, lo2} - {1'b0, hi2};
    wire [33:0] red2_pos = red2[33] ? (red2 + {1'b0, M2}) : red2;
    wire [32:0] a2 = red2_pos[32:0];

    // ---- Step 3: a3 = ((r3 - a1)*INV_M1_M3 - a2) * INV_M1M2_M3 mod M3 ----
    // Sub-step 3a: (r3 - a1) mod M3
    // a1 mod M3: a1 in [0, 2^32-2]; a1 mod M3:
    //   fold 32-bit a1 into 31-bit chunks: a1[30:0] + a1[31]  (since 2^31 ≡ 1 mod M3)
    wire [31:0] a1_mod_m3_raw = {1'b0, a1[30:0]} + {31'b0, a1[31]};
    wire [30:0] a1_mod_m3 = (a1_mod_m3_raw >= {1'b0, M3}) ?
                             a1_mod_m3_raw[30:0] - M3 : a1_mod_m3_raw[30:0];

    wire [31:0] diff3a = {1'b0, r3} - {1'b0, a1_mod_m3};
    wire [30:0] diff3a_pos = diff3a[31] ? (diff3a[30:0] + M3) : diff3a[30:0];

    // INV_M1_M3 = 1, so no multiply needed: t3a = diff3a_pos
    wire [30:0] t3a = diff3a_pos;

    // Sub-step 3b: a2 mod M3
    // a2 in [0, M2-1=2^32]; reduce mod M3 (2^31-1):
    // fold: a2[30:0] + a2[32:31]  (since 2^31 ≡ 1 mod M3)
    wire [31:0] a2_fold = {1'b0, a2[30:0]} + {30'b0, a2[32:31]};
    wire [30:0] a2_mod_m3 = (a2_fold >= {1'b0, M3}) ? a2_fold[30:0] - M3 : a2_fold[30:0];

    // Sub-step 3c: (t3a - a2_mod_m3) mod M3
    wire [31:0] diff3b = {1'b0, t3a} - {1'b0, a2_mod_m3};
    wire [30:0] diff3b_pos = diff3b[31] ? (diff3b[30:0] + M3) : diff3b[30:0];

    // Multiply diff3b_pos * INV_M1M2_M3 (= 0x5555_5555) mod M3
    wire [61:0] prod3_full = {31'b0, diff3b_pos} * {31'b0, INV_M1M2_M3};
    // Reduce mod M3=2^31-1: fold 62-bit product in 31-bit chunks
    // 2^31 ≡ 1 mod M3 => each chunk of 31 bits contributes directly
    wire [30:0] p3_c0 = prod3_full[30: 0];
    wire [30:0] p3_c1 = prod3_full[61:31];
    wire [31:0] sum3_r = {1'b0, p3_c0} + {1'b0, p3_c1};
    wire [30:0] a3 = (sum3_r >= {1'b0, M3}) ? sum3_r[30:0] - M3 : sum3_r[30:0];

    // ---- Step 4: X = a1 + a2*M1 + a3*M1*M2  (truncated to 64 bits) ----
    // M1 = 2^32 - 1
    // a2*M1 = a2*2^32 - a2   (a2 up to 2^32, so product up to 2^64 - 2^32)
    // a3*M1*M2 = a3*(2^64-1) (a3 up to 2^31-2, product up to ~2^95)
    // We truncate to 64 bits.

    // a2 * M1 (truncate to 64 bits):
    //   = a2 * 2^32 - a2
    //   a2 is 33 bits; a2*2^32 shifts left 32 bits
    wire [64:0] a2_times_M1 = {a2, 32'b0} - {32'b0, a2}; // 65-bit, take [63:0]

    // a3 * M1 * M2 (truncate to 64 bits):
    //   M1*M2 = (2^32-1)(2^32+1) = 2^64-1
    //   a3*(2^64-1) = a3*2^64 - a3
    //   Truncated to 64 bits: upper 64 bits of a3*2^64 are zero (it's a left shift by 64),
    //   so (a3*(2^64-1)) mod 2^64 = (0 - a3) mod 2^64 = 2^64 - a3
    //   But for small results (correct arithmetic), the true result fits <2^64,
    //   so a3 should be 0 for 32-bit inputs. For 64-bit: the term adds a3*(2^64-1).
    //   Truncated: just -a3 mod 2^64 = {~a3[63:0]+1} -- simplify: since a3 < 2^31,
    //   this is 64'hFFFF_FFFF_FFFF_FFFF - a3 + 1 (if a3 != 0)
    //   For correctness with 64-bit range inputs (a3 will be non-zero for large values):
    wire [63:0] a3_times_M1M2 = (~{33'b0, a3} + 64'b1); // 2^64 - a3 (mod 2^64)

    assign X = {1'b0, a1}              // a1 as 64-bit
             + a2_times_M1[63:0]       // a2*M1 (64-bit trunc)
             + a3_times_M1M2;          // a3*M1*M2 (64-bit trunc)

endmodule


/*======================================================================
 *  picorv32  (RNS extended)
 *
 *  Changes from original:
 *  1. reg_op1_64, reg_op2_64 — 64-bit operand registers (zero-extend from 32-bit regs,
 *     or concatenate two 32-bit regs for true 64-bit operation)
 *  2. rns_forward_converter instances for both operands — instantiated right after
 *     operand read, before the ALU
 *  3. rns_alu — replaces the original alu_add_sub for ADD/SUB only
 *  4. rns_reverse_converter — produces reg_out64 (64-bit); lower 32 bits go to
 *     reg_out for the register file write, upper 32 bits available on reg_out_hi
 *  5. All other ALU ops (compare, xor, or, and, shifts) still use original 32-bit paths
 *  6. reg_out_hi output port added for software to read the high 32 bits of 64-bit result
 *======================================================================*/

module picorv32_rns #(
    parameter [ 0:0] ENABLE_COUNTERS = 1,
    parameter [ 0:0] ENABLE_COUNTERS64 = 1,
    parameter [ 0:0] ENABLE_REGS_16_31 = 1,
    parameter [ 0:0] ENABLE_REGS_DUALPORT = 1,
    parameter [ 0:0] LATCHED_MEM_RDATA = 0,
    parameter [ 0:0] TWO_STAGE_SHIFT = 1,
    parameter [ 0:0] BARREL_SHIFTER = 0,
    parameter [ 0:0] TWO_CYCLE_COMPARE = 0,
    parameter [ 0:0] TWO_CYCLE_ALU = 0,
    parameter [ 0:0] COMPRESSED_ISA = 0,
    parameter [ 0:0] CATCH_MISALIGN = 1,
    parameter [ 0:0] CATCH_ILLINSN = 1,
    parameter [ 0:0] ENABLE_PCPI = 0,
    parameter [ 0:0] ENABLE_MUL = 0,
    parameter [ 0:0] ENABLE_FAST_MUL = 0,
    parameter [ 0:0] ENABLE_DIV = 0,
    parameter [ 0:0] ENABLE_IRQ = 0,
    parameter [ 0:0] ENABLE_IRQ_QREGS = 1,
    parameter [ 0:0] ENABLE_IRQ_TIMER = 1,
    parameter [ 0:0] ENABLE_TRACE = 0,
    parameter [ 0:0] REGS_INIT_ZERO = 0,
    parameter [31:0] MASKED_IRQ = 32'h 0000_0000,
    parameter [31:0] LATCHED_IRQ = 32'h ffff_ffff,
    parameter [31:0] PROGADDR_RESET = 32'h 0000_0000,
    parameter [31:0] PROGADDR_IRQ = 32'h 0000_0010,
    parameter [31:0] STACKADDR = 32'h ffff_ffff
) (
    input clk, resetn,
    output reg trap,

    output reg        mem_valid,
    output reg        mem_instr,
    input             mem_ready,

    output reg [31:0] mem_addr,
    output reg [31:0] mem_wdata,
    output reg [ 3:0] mem_wstrb,
    input      [31:0] mem_rdata,

    // Look-Ahead Interface
    output            mem_la_read,
    output            mem_la_write,
    output     [31:0] mem_la_addr,
    output reg [31:0] mem_la_wdata,
    output reg [ 3:0] mem_la_wstrb,

    // Pico Co-Processor Interface (PCPI)
    output reg        pcpi_valid,
    output reg [31:0] pcpi_insn,
    output     [31:0] pcpi_rs1,
    output     [31:0] pcpi_rs2,
    input             pcpi_wr,
    input      [31:0] pcpi_rd,
    input             pcpi_wait,
    input             pcpi_ready,

    // IRQ Interface
    input      [31:0] irq,
    output reg [31:0] eoi,

    // RNS 64-bit result high word (upper 32 bits of 64-bit ADD/SUB result)
    output reg [31:0] reg_out_hi,

    // Trace Interface
    output reg        trace_valid,
    output reg [35:0] trace_data
);

    localparam integer irq_timer    = 0;
    localparam integer irq_ebreak   = 1;
    localparam integer irq_buserror = 2;

    localparam integer irqregs_offset = ENABLE_REGS_16_31 ? 32 : 16;
    localparam integer regfile_size   = (ENABLE_REGS_16_31 ? 32 : 16) + 4*ENABLE_IRQ*ENABLE_IRQ_QREGS;
    localparam integer regindex_bits  = (ENABLE_REGS_16_31 ? 5 : 4) + ENABLE_IRQ*ENABLE_IRQ_QREGS;

    localparam WITH_PCPI = ENABLE_PCPI || ENABLE_MUL || ENABLE_FAST_MUL || ENABLE_DIV;

    localparam [35:0] TRACE_BRANCH = {4'b 0001, 32'b 0};
    localparam [35:0] TRACE_ADDR   = {4'b 0010, 32'b 0};
    localparam [35:0] TRACE_IRQ    = {4'b 1000, 32'b 0};

    reg [63:0] count_cycle, count_instr;
    reg [31:0] reg_pc, reg_next_pc, reg_op1, reg_op2, reg_out;
    reg [4:0]  reg_sh;

    reg [31:0] next_insn_opcode;
    reg [31:0] dbg_insn_opcode;
    reg [31:0] dbg_insn_addr;

    wire dbg_mem_valid = mem_valid;
    wire dbg_mem_instr = mem_instr;
    wire dbg_mem_ready = mem_ready;
    wire [31:0] dbg_mem_addr  = mem_addr;
    wire [31:0] dbg_mem_wdata = mem_wdata;
    wire [ 3:0] dbg_mem_wstrb = mem_wstrb;
    wire [31:0] dbg_mem_rdata = mem_rdata;

    assign pcpi_rs1 = reg_op1;
    assign pcpi_rs2 = reg_op2;

    wire [31:0] next_pc;

    reg irq_delay;
    reg irq_active;
    reg [31:0] irq_mask;
    reg [31:0] irq_pending;
    reg [31:0] timer;

`ifndef PICORV32_REGS
    reg [31:0] cpuregs [0:regfile_size-1];

    integer i;
    initial begin
        if (REGS_INIT_ZERO) begin
            for (i = 0; i < regfile_size; i = i+1)
                cpuregs[i] = 0;
        end
    end
`endif

    task empty_statement;
        begin end
    endtask

    // =========================================================
    //  RNS FORWARD CONVERTER INSTANTIATION
    //  Placed right after operands are read from the register file.
    //  Converts reg_op1 and reg_op2 (32-bit) into RNS residues.
    //  For 64-bit operation, reg_op1/reg_op2 carry the lower 32 bits;
    //  reg_op1_hi / reg_op2_hi (driven externally or zero) carry the upper 32 bits.
    //
    //  In this implementation, reg_op1_64 and reg_op2_64 zero-extend the
    //  32-bit register values. To use full 64-bit operands, the user
    //  should write the 64-bit value as:  {upper_word, lower_word}
    //  by pairing two 32-bit RISC-V register reads (see usage note below).
    // =========================================================

    // 64-bit extended operands (zero-extend 32-bit reg values)
    wire [63:0] rns_op1_64 = {32'b0, reg_op1};
    wire [63:0] rns_op2_64 = {32'b0, reg_op2};

    // Residues for operand 1
    wire [31:0] rns_r1_a;
    wire [32:0] rns_r2_a;
    wire [30:0] rns_r3_a;

    // Residues for operand 2
    wire [31:0] rns_r1_b;
    wire [32:0] rns_r2_b;
    wire [30:0] rns_r3_b;

    // Forward converters — right after input operands
    rns_forward_converter fwd_conv_a (
        .X  (rns_op1_64),
        .r1 (rns_r1_a),
        .r2 (rns_r2_a),
        .r3 (rns_r3_a)
    );

    rns_forward_converter fwd_conv_b (
        .X  (rns_op2_64),
        .r1 (rns_r1_b),
        .r2 (rns_r2_b),
        .r3 (rns_r3_b)
    );

    // =========================================================
    //  RNS ALU  (add / subtract in RNS domain)
    // =========================================================

    reg  rns_is_sub;   // driven in always @* below (after instr_sub is declared)

    wire [31:0] rns_out1;
    wire [32:0] rns_out2;
    wire [30:0] rns_out3;

    rns_alu rns_alu_inst (
        .is_sub (rns_is_sub),
        .a1     (rns_r1_a),
        .a2     (rns_r2_a),
        .a3     (rns_r3_a),
        .b1     (rns_r1_b),
        .b2     (rns_r2_b),
        .b3     (rns_r3_b),
        .out1   (rns_out1),
        .out2   (rns_out2),
        .out3   (rns_out3)
    );

    // =========================================================
    //  RNS REVERSE CONVERTER  (CRT reconstruction to 64 bits)
    // =========================================================

    wire [63:0] rns_result_64;

    rns_reverse_converter rev_conv (
        .r1 (rns_out1),
        .r2 (rns_out2),
        .r3 (rns_out3),
        .X  (rns_result_64)
    );

    // The 64-bit RNS result:
    //   lower 32 bits → reg_out (written to destination register)
    //   upper 32 bits → reg_out_hi (accessible as a sideband output)
    wire [31:0] rns_result_lo = rns_result_64[31:0];
    wire [31:0] rns_result_hi = rns_result_64[63:32];

    // =========================================================
    //  INSTRUCTION DECODE (unchanged from original)
    // =========================================================

    reg instr_lui, instr_auipc, instr_jal, instr_jalr;
    reg instr_beq, instr_bne, instr_blt, instr_bge, instr_bltu, instr_bgeu;
    reg instr_lb, instr_lh, instr_lw, instr_lbu, instr_lhu, instr_sb, instr_sh, instr_sw;
    reg instr_addi, instr_slti, instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai;
    reg instr_add, instr_sub, instr_sll, instr_slt, instr_sltu, instr_xor, instr_srl, instr_sra, instr_or, instr_and;
    reg instr_rdcycle, instr_rdcycleh, instr_rdinstr, instr_rdinstrh, instr_ecall_ebreak, instr_fence;
    reg instr_getq, instr_setq, instr_retirq, instr_maskirq, instr_waitirq, instr_timer;
    wire instr_trap;
    // Drive rns_is_sub combinationally from instr_sub (now declared)
    always @* rns_is_sub = instr_sub;


    reg [regindex_bits-1:0] decoded_rd, decoded_rs1;
    reg [4:0] decoded_rs2;
    reg [31:0] decoded_imm, decoded_imm_j;
    reg decoder_trigger;
    reg decoder_trigger_q;
    reg decoder_pseudo_trigger;
    reg decoder_pseudo_trigger_q;
    reg compressed_instr;

    reg is_lui_auipc_jal;
    reg is_lb_lh_lw_lbu_lhu;
    reg is_slli_srli_srai;
    reg is_jalr_addi_slti_sltiu_xori_ori_andi;
    reg is_sb_sh_sw;
    reg is_sll_srl_sra;
    reg is_lui_auipc_jal_jalr_addi_add_sub;
    reg is_slti_blt_slt;
    reg is_sltiu_bltu_sltu;
    reg is_beq_bne_blt_bge_bltu_bgeu;
    reg is_lbu_lhu_lw;
    reg is_alu_reg_imm;
    reg is_alu_reg_reg;
    reg is_compare;

    assign instr_trap = (CATCH_ILLINSN || WITH_PCPI) && !{instr_lui, instr_auipc, instr_jal, instr_jalr,
            instr_beq, instr_bne, instr_blt, instr_bge, instr_bltu, instr_bgeu,
            instr_lb, instr_lh, instr_lw, instr_lbu, instr_lhu, instr_sb, instr_sh, instr_sw,
            instr_addi, instr_slti, instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai,
            instr_add, instr_sub, instr_sll, instr_slt, instr_sltu, instr_xor, instr_srl, instr_sra, instr_or, instr_and,
            instr_rdcycle, instr_rdcycleh, instr_rdinstr, instr_rdinstrh, instr_fence,
            instr_getq, instr_setq, instr_retirq, instr_maskirq, instr_waitirq, instr_timer};

    wire is_rdcycle_rdcycleh_rdinstr_rdinstrh;
    assign is_rdcycle_rdcycleh_rdinstr_rdinstrh = |{instr_rdcycle, instr_rdcycleh, instr_rdinstr, instr_rdinstrh};

    reg [63:0] new_ascii_instr;
    `FORMAL_KEEP reg [63:0] dbg_ascii_instr;
    `FORMAL_KEEP reg [31:0] dbg_insn_imm;
    `FORMAL_KEEP reg [4:0]  dbg_insn_rs1;
    `FORMAL_KEEP reg [4:0]  dbg_insn_rs2;
    `FORMAL_KEEP reg [4:0]  dbg_insn_rd;
    `FORMAL_KEEP reg [31:0] dbg_rs1val;
    `FORMAL_KEEP reg [31:0] dbg_rs2val;
    `FORMAL_KEEP reg        dbg_rs1val_valid;
    `FORMAL_KEEP reg        dbg_rs2val_valid;

    always @* begin
        new_ascii_instr = "";
        if (instr_lui)      new_ascii_instr = "lui";
        if (instr_auipc)    new_ascii_instr = "auipc";
        if (instr_jal)      new_ascii_instr = "jal";
        if (instr_jalr)     new_ascii_instr = "jalr";
        if (instr_beq)      new_ascii_instr = "beq";
        if (instr_bne)      new_ascii_instr = "bne";
        if (instr_blt)      new_ascii_instr = "blt";
        if (instr_bge)      new_ascii_instr = "bge";
        if (instr_bltu)     new_ascii_instr = "bltu";
        if (instr_bgeu)     new_ascii_instr = "bgeu";
        if (instr_lb)       new_ascii_instr = "lb";
        if (instr_lh)       new_ascii_instr = "lh";
        if (instr_lw)       new_ascii_instr = "lw";
        if (instr_lbu)      new_ascii_instr = "lbu";
        if (instr_lhu)      new_ascii_instr = "lhu";
        if (instr_sb)       new_ascii_instr = "sb";
        if (instr_sh)       new_ascii_instr = "sh";
        if (instr_sw)       new_ascii_instr = "sw";
        if (instr_addi)     new_ascii_instr = "addi";
        if (instr_slti)     new_ascii_instr = "slti";
        if (instr_sltiu)    new_ascii_instr = "sltiu";
        if (instr_xori)     new_ascii_instr = "xori";
        if (instr_ori)      new_ascii_instr = "ori";
        if (instr_andi)     new_ascii_instr = "andi";
        if (instr_slli)     new_ascii_instr = "slli";
        if (instr_srli)     new_ascii_instr = "srli";
        if (instr_srai)     new_ascii_instr = "srai";
        if (instr_add)      new_ascii_instr = "add";
        if (instr_sub)      new_ascii_instr = "sub";
        if (instr_sll)      new_ascii_instr = "sll";
        if (instr_slt)      new_ascii_instr = "slt";
        if (instr_sltu)     new_ascii_instr = "sltu";
        if (instr_xor)      new_ascii_instr = "xor";
        if (instr_srl)      new_ascii_instr = "srl";
        if (instr_sra)      new_ascii_instr = "sra";
        if (instr_or)       new_ascii_instr = "or";
        if (instr_and)      new_ascii_instr = "and";
        if (instr_rdcycle)  new_ascii_instr = "rdcycle";
        if (instr_rdcycleh) new_ascii_instr = "rdcycleh";
        if (instr_rdinstr)  new_ascii_instr = "rdinstr";
        if (instr_rdinstrh) new_ascii_instr = "rdinstrh";
        if (instr_fence)    new_ascii_instr = "fence";
        if (instr_getq)     new_ascii_instr = "getq";
        if (instr_setq)     new_ascii_instr = "setq";
        if (instr_retirq)   new_ascii_instr = "retirq";
        if (instr_maskirq)  new_ascii_instr = "maskirq";
        if (instr_waitirq)  new_ascii_instr = "waitirq";
        if (instr_timer)    new_ascii_instr = "timer";
    end

    reg [63:0] q_ascii_instr;
    reg [31:0] q_insn_imm;
    reg [31:0] q_insn_opcode;
    reg [4:0]  q_insn_rs1;
    reg [4:0]  q_insn_rs2;
    reg [4:0]  q_insn_rd;
    reg        dbg_next;

    wire launch_next_insn;
    reg  dbg_valid_insn;

    reg [63:0] cached_ascii_instr;
    reg [31:0] cached_insn_imm;
    reg [31:0] cached_insn_opcode;
    reg [4:0]  cached_insn_rs1;
    reg [4:0]  cached_insn_rs2;
    reg [4:0]  cached_insn_rd;

    always @(posedge clk) begin
        q_ascii_instr  <= dbg_ascii_instr;
        q_insn_imm     <= dbg_insn_imm;
        q_insn_opcode  <= dbg_insn_opcode;
        q_insn_rs1     <= dbg_insn_rs1;
        q_insn_rs2     <= dbg_insn_rs2;
        q_insn_rd      <= dbg_insn_rd;
        dbg_next       <= launch_next_insn;

        if (!resetn || trap)
            dbg_valid_insn <= 0;
        else if (launch_next_insn)
            dbg_valid_insn <= 1;

        if (decoder_trigger_q) begin
            cached_ascii_instr  <= new_ascii_instr;
            cached_insn_imm     <= decoded_imm;
            if (&next_insn_opcode[1:0])
                cached_insn_opcode <= next_insn_opcode;
            else
                cached_insn_opcode <= {16'b0, next_insn_opcode[15:0]};
            cached_insn_rs1 <= decoded_rs1;
            cached_insn_rs2 <= decoded_rs2;
            cached_insn_rd  <= decoded_rd;
        end

        if (launch_next_insn)
            dbg_insn_addr <= next_pc;
    end

    always @* begin
        dbg_ascii_instr  = q_ascii_instr;
        dbg_insn_imm     = q_insn_imm;
        dbg_insn_opcode  = q_insn_opcode;
        dbg_insn_rs1     = q_insn_rs1;
        dbg_insn_rs2     = q_insn_rs2;
        dbg_insn_rd      = q_insn_rd;

        if (dbg_next) begin
            if (decoder_pseudo_trigger_q) begin
                dbg_ascii_instr = cached_ascii_instr;
                dbg_insn_imm    = cached_insn_imm;
                dbg_insn_opcode = cached_insn_opcode;
                dbg_insn_rs1    = cached_insn_rs1;
                dbg_insn_rs2    = cached_insn_rs2;
                dbg_insn_rd     = cached_insn_rd;
            end else begin
                dbg_ascii_instr = new_ascii_instr;
                if (&next_insn_opcode[1:0])
                    dbg_insn_opcode = next_insn_opcode;
                else
                    dbg_insn_opcode = {16'b0, next_insn_opcode[15:0]};
                dbg_insn_imm = decoded_imm;
                dbg_insn_rs1 = decoded_rs1;
                dbg_insn_rs2 = decoded_rs2;
                dbg_insn_rd  = decoded_rd;
            end
        end
    end

    // =========================================================
    //  MEMORY INTERFACE (unchanged)
    // =========================================================

    reg [1:0]  mem_state;
    reg [1:0]  mem_wordsize;
    reg [31:0] mem_rdata_word;
    reg [31:0] mem_rdata_q;
    reg mem_do_prefetch;
    reg mem_do_rinst;
    reg mem_do_rdata;
    reg mem_do_wdata;

    wire mem_xfer;
    reg  mem_la_secondword, mem_la_firstword_reg, last_mem_valid;
    wire mem_la_firstword = COMPRESSED_ISA && (mem_do_prefetch || mem_do_rinst) && next_pc[1] && !mem_la_secondword;
    wire mem_la_firstword_xfer = COMPRESSED_ISA && mem_xfer && (!last_mem_valid ? mem_la_firstword : mem_la_firstword_reg);

    reg  prefetched_high_word;
    reg  clear_prefetched_high_word;
    reg  [15:0] mem_16bit_buffer;

    wire [31:0] mem_rdata_latched_noshuffle;
    wire [31:0] mem_rdata_latched;

    wire mem_la_use_prefetched_high_word = COMPRESSED_ISA && mem_la_firstword && prefetched_high_word && !clear_prefetched_high_word;
    assign mem_xfer = (mem_valid && mem_ready) || (mem_la_use_prefetched_high_word && mem_do_rinst);

    wire mem_busy = |{mem_do_prefetch, mem_do_rinst, mem_do_rdata, mem_do_wdata};
    wire mem_done = resetn && ((mem_xfer && |mem_state && (mem_do_rinst || mem_do_rdata || mem_do_wdata)) || (&mem_state && mem_do_rinst)) &&
            (!mem_la_firstword || (~&mem_rdata_latched[1:0] && mem_xfer));

    assign mem_la_write = resetn && !mem_state && mem_do_wdata;
    assign mem_la_read  = resetn && ((!mem_la_use_prefetched_high_word && !mem_state && (mem_do_rinst || mem_do_prefetch || mem_do_rdata)) ||
            (COMPRESSED_ISA && mem_xfer && (!last_mem_valid ? mem_la_firstword : mem_la_firstword_reg) && !mem_la_secondword && &mem_rdata_latched[1:0]));
    assign mem_la_addr  = (mem_do_prefetch || mem_do_rinst) ? {next_pc[31:2] + mem_la_firstword_xfer, 2'b00} : {reg_op1[31:2], 2'b00};

    assign mem_rdata_latched_noshuffle = (mem_xfer || LATCHED_MEM_RDATA) ? mem_rdata : mem_rdata_q;

    assign mem_rdata_latched = COMPRESSED_ISA && mem_la_use_prefetched_high_word ? {16'bx, mem_16bit_buffer} :
            COMPRESSED_ISA && mem_la_secondword ? {mem_rdata_latched_noshuffle[15:0], mem_16bit_buffer} :
            COMPRESSED_ISA && mem_la_firstword  ? {16'bx, mem_rdata_latched_noshuffle[31:16]} : mem_rdata_latched_noshuffle;

    always @(posedge clk) begin
        if (!resetn) begin
            mem_la_firstword_reg <= 0;
            last_mem_valid       <= 0;
        end else begin
            if (!last_mem_valid)
                mem_la_firstword_reg <= mem_la_firstword;
            last_mem_valid <= mem_valid && !mem_ready;
        end
    end

    always @* begin
        (* full_case *)
        case (mem_wordsize)
            0: begin
                mem_la_wdata = reg_op2;
                mem_la_wstrb = 4'b1111;
                mem_rdata_word = mem_rdata;
            end
            1: begin
                mem_la_wdata = {2{reg_op2[15:0]}};
                mem_la_wstrb = reg_op1[1] ? 4'b1100 : 4'b0011;
                case (reg_op1[1])
                    1'b0: mem_rdata_word = {16'b0, mem_rdata[15: 0]};
                    1'b1: mem_rdata_word = {16'b0, mem_rdata[31:16]};
                endcase
            end
            2: begin
                mem_la_wdata = {4{reg_op2[7:0]}};
                mem_la_wstrb = 4'b0001 << reg_op1[1:0];
                case (reg_op1[1:0])
                    2'b00: mem_rdata_word = {24'b0, mem_rdata[ 7: 0]};
                    2'b01: mem_rdata_word = {24'b0, mem_rdata[15: 8]};
                    2'b10: mem_rdata_word = {24'b0, mem_rdata[23:16]};
                    2'b11: mem_rdata_word = {24'b0, mem_rdata[31:24]};
                endcase
            end
        endcase
    end

    always @(posedge clk) begin
        if (mem_xfer) begin
            mem_rdata_q      <= COMPRESSED_ISA ? mem_rdata_latched : mem_rdata;
            next_insn_opcode <= COMPRESSED_ISA ? mem_rdata_latched : mem_rdata;
        end

        if (COMPRESSED_ISA && mem_done && (mem_do_prefetch || mem_do_rinst)) begin
            case (mem_rdata_latched[1:0])
                2'b00: begin
                    case (mem_rdata_latched[15:13])
                        3'b000: begin
                            mem_rdata_q[14:12] <= 3'b000;
                            mem_rdata_q[31:20] <= {2'b0, mem_rdata_latched[10:7], mem_rdata_latched[12:11], mem_rdata_latched[5], mem_rdata_latched[6], 2'b00};
                        end
                        3'b010: begin
                            mem_rdata_q[31:20] <= {5'b0, mem_rdata_latched[5], mem_rdata_latched[12:10], mem_rdata_latched[6], 2'b00};
                            mem_rdata_q[14:12] <= 3'b010;
                        end
                        3'b110: begin
                            {mem_rdata_q[31:25], mem_rdata_q[11:7]} <= {5'b0, mem_rdata_latched[5], mem_rdata_latched[12:10], mem_rdata_latched[6], 2'b00};
                            mem_rdata_q[14:12] <= 3'b010;
                        end
                    endcase
                end
                2'b01: begin
                    case (mem_rdata_latched[15:13])
                        3'b000: begin
                            mem_rdata_q[14:12] <= 3'b000;
                            mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                        end
                        3'b010: begin
                            mem_rdata_q[14:12] <= 3'b000;
                            mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                        end
                        3'b011: begin
                            if (mem_rdata_latched[11:7] == 2) begin
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[4:3],
                                        mem_rdata_latched[5], mem_rdata_latched[2], mem_rdata_latched[6], 4'b0000});
                            end else begin
                                mem_rdata_q[31:12] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                            end
                        end
                        3'b100: begin
                            if (mem_rdata_latched[11:10] == 2'b00) begin
                                mem_rdata_q[31:25] <= 7'b0000000;
                                mem_rdata_q[14:12] <= 3'b101;
                            end
                            if (mem_rdata_latched[11:10] == 2'b01) begin
                                mem_rdata_q[31:25] <= 7'b0100000;
                                mem_rdata_q[14:12] <= 3'b101;
                            end
                            if (mem_rdata_latched[11:10] == 2'b10) begin
                                mem_rdata_q[14:12] <= 3'b111;
                                mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                            end
                            if (mem_rdata_latched[12:10] == 3'b011) begin
                                if (mem_rdata_latched[6:5] == 2'b00) mem_rdata_q[14:12] <= 3'b000;
                                if (mem_rdata_latched[6:5] == 2'b01) mem_rdata_q[14:12] <= 3'b100;
                                if (mem_rdata_latched[6:5] == 2'b10) mem_rdata_q[14:12] <= 3'b110;
                                if (mem_rdata_latched[6:5] == 2'b11) mem_rdata_q[14:12] <= 3'b111;
                                mem_rdata_q[31:25] <= mem_rdata_latched[6:5] == 2'b00 ? 7'b0100000 : 7'b0000000;
                            end
                        end
                        3'b110: begin
                            mem_rdata_q[14:12] <= 3'b000;
                            { mem_rdata_q[31], mem_rdata_q[7], mem_rdata_q[30:25], mem_rdata_q[11:8] } <=
                                    $signed({mem_rdata_latched[12], mem_rdata_latched[6:5], mem_rdata_latched[2],
                                            mem_rdata_latched[11:10], mem_rdata_latched[4:3]});
                        end
                        3'b111: begin
                            mem_rdata_q[14:12] <= 3'b001;
                            { mem_rdata_q[31], mem_rdata_q[7], mem_rdata_q[30:25], mem_rdata_q[11:8] } <=
                                    $signed({mem_rdata_latched[12], mem_rdata_latched[6:5], mem_rdata_latched[2],
                                            mem_rdata_latched[11:10], mem_rdata_latched[4:3]});
                        end
                    endcase
                end
                2'b10: begin
                    case (mem_rdata_latched[15:13])
                        3'b000: begin
                            mem_rdata_q[31:25] <= 7'b0000000;
                            mem_rdata_q[14:12] <= 3'b001;
                        end
                        3'b010: begin
                            mem_rdata_q[31:20] <= {4'b0, mem_rdata_latched[3:2], mem_rdata_latched[12], mem_rdata_latched[6:4], 2'b00};
                            mem_rdata_q[14:12] <= 3'b010;
                        end
                        3'b100: begin
                            if (mem_rdata_latched[12] == 0 && mem_rdata_latched[6:2] == 0) begin
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:20] <= 12'b0;
                            end
                            if (mem_rdata_latched[12] == 0 && mem_rdata_latched[6:2] != 0) begin
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:25] <= 7'b0000000;
                            end
                            if (mem_rdata_latched[12] != 0 && mem_rdata_latched[11:7] != 0 && mem_rdata_latched[6:2] == 0) begin
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:20] <= 12'b0;
                            end
                            if (mem_rdata_latched[12] != 0 && mem_rdata_latched[6:2] != 0) begin
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:25] <= 7'b0000000;
                            end
                        end
                        3'b110: begin
                            {mem_rdata_q[31:25], mem_rdata_q[11:7]} <= {4'b0, mem_rdata_latched[8:7], mem_rdata_latched[12:9], 2'b00};
                            mem_rdata_q[14:12] <= 3'b010;
                        end
                    endcase
                end
            endcase
        end
    end

    // =========================================================
    //  INSTRUCTION DECODER TRIGGER (unchanged)
    // =========================================================

    always @(posedge clk) begin
        is_lui_auipc_jal <= |{instr_lui, instr_auipc, instr_jal};
        is_lui_auipc_jal_jalr_addi_add_sub <= |{instr_lui, instr_auipc, instr_jal, instr_jalr, instr_addi, instr_add, instr_sub};
        is_slti_blt_slt   <= |{instr_slti, instr_blt, instr_slt};
        is_sltiu_bltu_sltu <= |{instr_sltiu, instr_bltu, instr_sltu};
        is_lbu_lhu_lw     <= |{instr_lbu, instr_lhu, instr_lw};
        is_compare        <= |{is_beq_bne_blt_bge_bltu_bgeu, instr_slti, instr_slt, instr_sltiu, instr_sltu};

        if (mem_do_rinst && mem_done) begin
            instr_lui     <= mem_rdata_latched[6:0] == 7'b0110111;
            instr_auipc   <= mem_rdata_latched[6:0] == 7'b0010111;
            instr_jal     <= mem_rdata_latched[6:0] == 7'b1101111;
            instr_jalr    <= mem_rdata_latched[6:0] == 7'b1100111 && mem_rdata_latched[14:12] == 3'b000;
            instr_retirq  <= mem_rdata_latched[6:0] == 7'b0001011 && mem_rdata_latched[31:25] == 7'b0000010 && ENABLE_IRQ;
            instr_waitirq <= mem_rdata_latched[6:0] == 7'b0001011 && mem_rdata_latched[31:25] == 7'b0000100 && ENABLE_IRQ;

            is_beq_bne_blt_bge_bltu_bgeu <= mem_rdata_latched[6:0] == 7'b1100011;
            is_lb_lh_lw_lbu_lhu          <= mem_rdata_latched[6:0] == 7'b0000011;
            is_sb_sh_sw                  <= mem_rdata_latched[6:0] == 7'b0100011;
            is_alu_reg_imm               <= mem_rdata_latched[6:0] == 7'b0010011;
            is_alu_reg_reg               <= mem_rdata_latched[6:0] == 7'b0110011;

            { decoded_imm_j[31:20], decoded_imm_j[10:1], decoded_imm_j[11], decoded_imm_j[19:12], decoded_imm_j[0] } <=
                $signed({mem_rdata_latched[31:12], 1'b0});

            decoded_rd  <= mem_rdata_latched[11:7];
            decoded_rs1 <= mem_rdata_latched[19:15];
            decoded_rs2 <= mem_rdata_latched[24:20];

            if (mem_rdata_latched[6:0] == 7'b0001011 && mem_rdata_latched[31:25] == 7'b0000000 && ENABLE_IRQ && ENABLE_IRQ_QREGS)
                decoded_rs1[regindex_bits-1] <= 1;

            if (mem_rdata_latched[6:0] == 7'b0001011 && mem_rdata_latched[31:25] == 7'b0000010 && ENABLE_IRQ)
                decoded_rs1 <= ENABLE_IRQ_QREGS ? irqregs_offset : 3;

            compressed_instr <= 0;
            if (COMPRESSED_ISA && mem_rdata_latched[1:0] != 2'b11) begin
                compressed_instr <= 1;
                decoded_rd  <= 0;
                decoded_rs1 <= 0;
                decoded_rs2 <= 0;

                { decoded_imm_j[31:11], decoded_imm_j[4], decoded_imm_j[9:8], decoded_imm_j[10], decoded_imm_j[6],
                  decoded_imm_j[7], decoded_imm_j[3:1], decoded_imm_j[5], decoded_imm_j[0] } <=
                    $signed({mem_rdata_latched[12:2], 1'b0});

                case (mem_rdata_latched[1:0])
                    2'b00: begin
                        case (mem_rdata_latched[15:13])
                            3'b000: begin is_alu_reg_imm <= |mem_rdata_latched[12:5]; decoded_rs1 <= 2; decoded_rd <= 8 + mem_rdata_latched[4:2]; end
                            3'b010: begin is_lb_lh_lw_lbu_lhu <= 1; decoded_rs1 <= 8 + mem_rdata_latched[9:7]; decoded_rd <= 8 + mem_rdata_latched[4:2]; end
                            3'b110: begin is_sb_sh_sw <= 1; decoded_rs1 <= 8 + mem_rdata_latched[9:7]; decoded_rs2 <= 8 + mem_rdata_latched[4:2]; end
                        endcase
                    end
                    2'b01: begin
                        case (mem_rdata_latched[15:13])
                            3'b000: begin is_alu_reg_imm <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= mem_rdata_latched[11:7]; end
                            3'b001: begin instr_jal <= 1; decoded_rd <= 1; end
                            3'b010: begin is_alu_reg_imm <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= 0; end
                            3'b011: begin
                                if (mem_rdata_latched[12] || mem_rdata_latched[6:2]) begin
                                    if (mem_rdata_latched[11:7] == 2) begin is_alu_reg_imm <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= mem_rdata_latched[11:7]; end
                                    else begin instr_lui <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= 0; end
                                end
                            end
                            3'b100: begin
                                if (!mem_rdata_latched[11] && !mem_rdata_latched[12]) begin is_alu_reg_imm <= 1; decoded_rd <= 8 + mem_rdata_latched[9:7]; decoded_rs1 <= 8 + mem_rdata_latched[9:7]; decoded_rs2 <= {mem_rdata_latched[12], mem_rdata_latched[6:2]}; end
                                if (mem_rdata_latched[11:10] == 2'b10) begin is_alu_reg_imm <= 1; decoded_rd <= 8 + mem_rdata_latched[9:7]; decoded_rs1 <= 8 + mem_rdata_latched[9:7]; end
                                if (mem_rdata_latched[12:10] == 3'b011) begin is_alu_reg_reg <= 1; decoded_rd <= 8 + mem_rdata_latched[9:7]; decoded_rs1 <= 8 + mem_rdata_latched[9:7]; decoded_rs2 <= 8 + mem_rdata_latched[4:2]; end
                            end
                            3'b101: begin instr_jal <= 1; end
                            3'b110: begin is_beq_bne_blt_bge_bltu_bgeu <= 1; decoded_rs1 <= 8 + mem_rdata_latched[9:7]; decoded_rs2 <= 0; end
                            3'b111: begin is_beq_bne_blt_bge_bltu_bgeu <= 1; decoded_rs1 <= 8 + mem_rdata_latched[9:7]; decoded_rs2 <= 0; end
                        endcase
                    end
                    2'b10: begin
                        case (mem_rdata_latched[15:13])
                            3'b000: begin if (!mem_rdata_latched[12]) begin is_alu_reg_imm <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= mem_rdata_latched[11:7]; decoded_rs2 <= {mem_rdata_latched[12], mem_rdata_latched[6:2]}; end end
                            3'b010: begin if (mem_rdata_latched[11:7]) begin is_lb_lh_lw_lbu_lhu <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= 2; end end
                            3'b100: begin
                                if (mem_rdata_latched[12] == 0 && mem_rdata_latched[11:7] != 0 && mem_rdata_latched[6:2] == 0) begin instr_jalr <= 1; decoded_rd <= 0; decoded_rs1 <= mem_rdata_latched[11:7]; end
                                if (mem_rdata_latched[12] == 0 && mem_rdata_latched[6:2] != 0) begin is_alu_reg_reg <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= 0; decoded_rs2 <= mem_rdata_latched[6:2]; end
                                if (mem_rdata_latched[12] != 0 && mem_rdata_latched[11:7] != 0 && mem_rdata_latched[6:2] == 0) begin instr_jalr <= 1; decoded_rd <= 1; decoded_rs1 <= mem_rdata_latched[11:7]; end
                                if (mem_rdata_latched[12] != 0 && mem_rdata_latched[6:2] != 0) begin is_alu_reg_reg <= 1; decoded_rd <= mem_rdata_latched[11:7]; decoded_rs1 <= mem_rdata_latched[11:7]; decoded_rs2 <= mem_rdata_latched[6:2]; end
                            end
                            3'b110: begin is_sb_sh_sw <= 1; decoded_rs1 <= 2; decoded_rs2 <= mem_rdata_latched[6:2]; end
                        endcase
                    end
                endcase
            end
        end

        if (decoder_trigger && !decoder_pseudo_trigger) begin
            pcpi_insn <= WITH_PCPI ? mem_rdata_q : 'bx;

            instr_beq   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b000;
            instr_bne   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b001;
            instr_blt   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b100;
            instr_bge   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b101;
            instr_bltu  <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b110;
            instr_bgeu  <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b111;

            instr_lb    <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b000;
            instr_lh    <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b001;
            instr_lw    <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b010;
            instr_lbu   <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b100;
            instr_lhu   <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b101;

            instr_sb    <= is_sb_sh_sw && mem_rdata_q[14:12] == 3'b000;
            instr_sh    <= is_sb_sh_sw && mem_rdata_q[14:12] == 3'b001;
            instr_sw    <= is_sb_sh_sw && mem_rdata_q[14:12] == 3'b010;

            instr_addi  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b000;
            instr_slti  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b010;
            instr_sltiu <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b011;
            instr_xori  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b100;
            instr_ori   <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b110;
            instr_andi  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b111;

            instr_slli  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000;
            instr_srli  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000;
            instr_srai  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000;

            instr_add   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b000 && mem_rdata_q[31:25] == 7'b0000000;
            instr_sub   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b000 && mem_rdata_q[31:25] == 7'b0100000;
            instr_sll   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000;
            instr_slt   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b010 && mem_rdata_q[31:25] == 7'b0000000;
            instr_sltu  <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b011 && mem_rdata_q[31:25] == 7'b0000000;
            instr_xor   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b100 && mem_rdata_q[31:25] == 7'b0000000;
            instr_srl   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000;
            instr_sra   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000;
            instr_or    <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b110 && mem_rdata_q[31:25] == 7'b0000000;
            instr_and   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b111 && mem_rdata_q[31:25] == 7'b0000000;

            instr_rdcycle  <= ((mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11000000000000000010) ||
                               (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11000000000100000010)) && ENABLE_COUNTERS;
            instr_rdcycleh <= ((mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11001000000000000010) ||
                               (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11001000000100000010)) && ENABLE_COUNTERS && ENABLE_COUNTERS64;
            instr_rdinstr  <=  (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11000000001000000010) && ENABLE_COUNTERS;
            instr_rdinstrh <=  (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11001000001000000010) && ENABLE_COUNTERS && ENABLE_COUNTERS64;

            instr_ecall_ebreak <= ((mem_rdata_q[6:0] == 7'b1110011 && !mem_rdata_q[31:21] && !mem_rdata_q[19:7]) ||
                    (COMPRESSED_ISA && mem_rdata_q[15:0] == 16'h9002));
            instr_fence <= (mem_rdata_q[6:0] == 7'b0001111 && !mem_rdata_q[14:12]);

            instr_getq    <= mem_rdata_q[6:0] == 7'b0001011 && mem_rdata_q[31:25] == 7'b0000000 && ENABLE_IRQ && ENABLE_IRQ_QREGS;
            instr_setq    <= mem_rdata_q[6:0] == 7'b0001011 && mem_rdata_q[31:25] == 7'b0000001 && ENABLE_IRQ && ENABLE_IRQ_QREGS;
            instr_maskirq <= mem_rdata_q[6:0] == 7'b0001011 && mem_rdata_q[31:25] == 7'b0000011 && ENABLE_IRQ;
            instr_timer   <= mem_rdata_q[6:0] == 7'b0001011 && mem_rdata_q[31:25] == 7'b0000101 && ENABLE_IRQ && ENABLE_IRQ_TIMER;

            is_slli_srli_srai <= is_alu_reg_imm && |{
                mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000,
                mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000,
                mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000
            };

            is_jalr_addi_slti_sltiu_xori_ori_andi <= instr_jalr || is_alu_reg_imm && |{
                mem_rdata_q[14:12] == 3'b000,
                mem_rdata_q[14:12] == 3'b010,
                mem_rdata_q[14:12] == 3'b011,
                mem_rdata_q[14:12] == 3'b100,
                mem_rdata_q[14:12] == 3'b110,
                mem_rdata_q[14:12] == 3'b111
            };

            is_sll_srl_sra <= is_alu_reg_reg && |{
                mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000,
                mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000,
                mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000
            };

            is_lui_auipc_jal_jalr_addi_add_sub <= 0;
            is_compare <= 0;

            (* parallel_case *)
            case (1'b1)
                instr_jal:
                    decoded_imm <= decoded_imm_j;
                |{instr_lui, instr_auipc}:
                    decoded_imm <= mem_rdata_q[31:12] << 12;
                |{instr_jalr, is_lb_lh_lw_lbu_lhu, is_alu_reg_imm}:
                    decoded_imm <= $signed(mem_rdata_q[31:20]);
                is_beq_bne_blt_bge_bltu_bgeu:
                    decoded_imm <= $signed({mem_rdata_q[31], mem_rdata_q[7], mem_rdata_q[30:25], mem_rdata_q[11:8], 1'b0});
                is_sb_sh_sw:
                    decoded_imm <= $signed({mem_rdata_q[31:25], mem_rdata_q[11:7]});
                default:
                    decoded_imm <= 1'bx;
            endcase
        end

        if (!resetn) begin
            is_beq_bne_blt_bge_bltu_bgeu <= 0;
            is_compare <= 0;
            instr_beq   <= 0; instr_bne   <= 0; instr_blt   <= 0; instr_bge   <= 0;
            instr_bltu  <= 0; instr_bgeu  <= 0;
            instr_addi  <= 0; instr_slti  <= 0; instr_sltiu <= 0; instr_xori  <= 0;
            instr_ori   <= 0; instr_andi  <= 0;
            instr_add   <= 0; instr_sub   <= 0; instr_sll   <= 0; instr_slt   <= 0;
            instr_sltu  <= 0; instr_xor   <= 0; instr_srl   <= 0; instr_sra   <= 0;
            instr_or    <= 0; instr_and   <= 0;
            instr_fence <= 0;
        end
    end

    // =========================================================
    //  INTERNAL PCPI CORES
    // =========================================================

    wire        pcpi_mul_wr;
    wire [31:0] pcpi_mul_rd;
    wire        pcpi_mul_wait;
    wire        pcpi_mul_ready;

    wire        pcpi_div_wr;
    wire [31:0] pcpi_div_rd;
    wire        pcpi_div_wait;
    wire        pcpi_div_ready;

    reg        pcpi_int_wr;
    reg [31:0] pcpi_int_rd;
    reg        pcpi_int_wait;
    reg        pcpi_int_ready;

    generate if (ENABLE_FAST_MUL) begin
        picorv32_pcpi_fast_mul pcpi_mul (.clk(clk), .resetn(resetn), .pcpi_valid(pcpi_valid), .pcpi_insn(pcpi_insn), .pcpi_rs1(pcpi_rs1), .pcpi_rs2(pcpi_rs2), .pcpi_wr(pcpi_mul_wr), .pcpi_rd(pcpi_mul_rd), .pcpi_wait(pcpi_mul_wait), .pcpi_ready(pcpi_mul_ready));
    end else if (ENABLE_MUL) begin
        picorv32_pcpi_mul pcpi_mul (.clk(clk), .resetn(resetn), .pcpi_valid(pcpi_valid), .pcpi_insn(pcpi_insn), .pcpi_rs1(pcpi_rs1), .pcpi_rs2(pcpi_rs2), .pcpi_wr(pcpi_mul_wr), .pcpi_rd(pcpi_mul_rd), .pcpi_wait(pcpi_mul_wait), .pcpi_ready(pcpi_mul_ready));
    end else begin
        assign pcpi_mul_wr = 0; assign pcpi_mul_rd = 32'bx; assign pcpi_mul_wait = 0; assign pcpi_mul_ready = 0;
    end endgenerate

    generate if (ENABLE_DIV) begin
        picorv32_pcpi_div pcpi_div (.clk(clk), .resetn(resetn), .pcpi_valid(pcpi_valid), .pcpi_insn(pcpi_insn), .pcpi_rs1(pcpi_rs1), .pcpi_rs2(pcpi_rs2), .pcpi_wr(pcpi_div_wr), .pcpi_rd(pcpi_div_rd), .pcpi_wait(pcpi_div_wait), .pcpi_ready(pcpi_div_ready));
    end else begin
        assign pcpi_div_wr = 0; assign pcpi_div_rd = 32'bx; assign pcpi_div_wait = 0; assign pcpi_div_ready = 0;
    end endgenerate

    always @* begin
        pcpi_int_wr = 0;
        pcpi_int_rd = 32'bx;
        pcpi_int_wait  = |{ENABLE_PCPI && pcpi_wait,  (ENABLE_MUL || ENABLE_FAST_MUL) && pcpi_mul_wait,  ENABLE_DIV && pcpi_div_wait};
        pcpi_int_ready = |{ENABLE_PCPI && pcpi_ready, (ENABLE_MUL || ENABLE_FAST_MUL) && pcpi_mul_ready, ENABLE_DIV && pcpi_div_ready};
        (* parallel_case *)
        case (1'b1)
            ENABLE_PCPI && pcpi_ready:                       begin pcpi_int_wr = ENABLE_PCPI ? pcpi_wr : 0; pcpi_int_rd = ENABLE_PCPI ? pcpi_rd : 0; end
            (ENABLE_MUL || ENABLE_FAST_MUL) && pcpi_mul_ready: begin pcpi_int_wr = pcpi_mul_wr; pcpi_int_rd = pcpi_mul_rd; end
            ENABLE_DIV && pcpi_div_ready:                    begin pcpi_int_wr = pcpi_div_wr; pcpi_int_rd = pcpi_div_rd; end
        endcase
    end

    // =========================================================
    //  ALU  -- MODIFIED FOR RNS
    //
    //  ADD and SUB now use the RNS path (rns_result_lo).
    //  All other operations (compare, xor, or, and, shifts) use
    //  original 32-bit combinational logic unchanged.
    //  alu_add_sub now comes from the RNS reverse converter output.
    // =========================================================

    reg [31:0] alu_out, alu_out_q;
    reg        alu_out_0, alu_out_0_q;
    reg        alu_wait, alu_wait_2;

    // RNS-backed add/sub result (32-bit low word of 64-bit result)
    wire [31:0] alu_add_sub = rns_result_lo;

    // Remaining ALU ops (unchanged 32-bit)
    reg alu_eq, alu_ltu, alu_lts;
    reg [31:0] alu_shl, alu_shr;

    generate if (TWO_CYCLE_ALU) begin
        always @(posedge clk) begin
            // Note: alu_add_sub is now driven by RNS combinationally;
            // in TWO_CYCLE_ALU mode we register the RNS result one extra cycle.
            // reg_op1/reg_op2 are already stable when we reach exec.
            alu_eq  <= reg_op1 == reg_op2;
            alu_lts <= $signed(reg_op1) < $signed(reg_op2);
            alu_ltu <= reg_op1 < reg_op2;
            alu_shl <= reg_op1 << reg_op2[4:0];
            alu_shr <= $signed({instr_sra || instr_srai ? reg_op1[31] : 1'b0, reg_op1}) >>> reg_op2[4:0];
        end
    end else begin
        always @* begin
            alu_eq  = reg_op1 == reg_op2;
            alu_lts = $signed(reg_op1) < $signed(reg_op2);
            alu_ltu = reg_op1 < reg_op2;
            alu_shl = reg_op1 << reg_op2[4:0];
            alu_shr = $signed({instr_sra || instr_srai ? reg_op1[31] : 1'b0, reg_op1}) >>> reg_op2[4:0];
        end
    end endgenerate

    always @* begin
        alu_out_0 = 'bx;
        (* parallel_case, full_case *)
        case (1'b1)
            instr_beq:  alu_out_0 = alu_eq;
            instr_bne:  alu_out_0 = !alu_eq;
            instr_bge:  alu_out_0 = !alu_lts;
            instr_bgeu: alu_out_0 = !alu_ltu;
            is_slti_blt_slt  && (!TWO_CYCLE_COMPARE || !{instr_beq,instr_bne,instr_bge,instr_bgeu}): alu_out_0 = alu_lts;
            is_sltiu_bltu_sltu && (!TWO_CYCLE_COMPARE || !{instr_beq,instr_bne,instr_bge,instr_bgeu}): alu_out_0 = alu_ltu;
        endcase

        alu_out = 'bx;
        (* parallel_case, full_case *)
        case (1'b1)
            // ADD / SUB:  use RNS result (low 32 bits)
            is_lui_auipc_jal_jalr_addi_add_sub:
                alu_out = alu_add_sub;
            is_compare:
                alu_out = alu_out_0;
            instr_xori || instr_xor:
                alu_out = reg_op1 ^ reg_op2;
            instr_ori || instr_or:
                alu_out = reg_op1 | reg_op2;
            instr_andi || instr_and:
                alu_out = reg_op1 & reg_op2;
            BARREL_SHIFTER && (instr_sll || instr_slli):
                alu_out = alu_shl;
            BARREL_SHIFTER && (instr_srl || instr_srli || instr_sra || instr_srai):
                alu_out = alu_shr;
        endcase
    end

    reg        cpuregs_write;
    reg [31:0] cpuregs_wrdata;
    reg [31:0] cpuregs_rs1;
    reg [31:0] cpuregs_rs2;
    reg [regindex_bits-1:0] decoded_rs;

    reg        latched_store;
    reg        latched_stalu;
    reg        latched_branch;
    reg        latched_compr;
    reg        latched_trace;
    reg        latched_is_lu;
    reg        latched_is_lh;
    reg        latched_is_lb;
    reg [regindex_bits-1:0] latched_rd;

    reg [31:0] current_pc;
    assign next_pc = latched_store && latched_branch ? reg_out & ~1 : reg_next_pc;

    reg [3:0] pcpi_timeout_counter;
    reg       pcpi_timeout;

    reg [31:0] next_irq_pending;
    reg        do_waitirq;

    localparam cpu_state_trap   = 8'b10000000;
    localparam cpu_state_fetch  = 8'b01000000;
    localparam cpu_state_ld_rs1 = 8'b00100000;
    localparam cpu_state_ld_rs2 = 8'b00010000;
    localparam cpu_state_exec   = 8'b00001000;
    localparam cpu_state_shift  = 8'b00000100;
    localparam cpu_state_stmem  = 8'b00000010;
    localparam cpu_state_ldmem  = 8'b00000001;

    reg [7:0] cpu_state;
    reg [1:0] irq_state;

    reg clear_prefetched_high_word_q;
    always @(posedge clk) clear_prefetched_high_word_q <= clear_prefetched_high_word;

    always @* begin
        clear_prefetched_high_word = clear_prefetched_high_word_q;
        if (!prefetched_high_word)
            clear_prefetched_high_word = 0;
        if (latched_branch || irq_state || !resetn)
            clear_prefetched_high_word = COMPRESSED_ISA;
    end

    // =========================================================
    //  REGISTER FILE & CPU STATE MACHINE (unchanged structure)
    // =========================================================


    reg set_mem_do_rinst;
    reg set_mem_do_rdata;
    reg set_mem_do_wdata;

    always @* begin
        cpuregs_write  = 0;
        cpuregs_wrdata = 'bx;

        if (cpu_state == cpu_state_fetch) begin
            (* parallel_case *)
            case (1'b1)
                latched_branch: begin
                    cpuregs_wrdata = reg_pc + (latched_compr ? 2 : 4);
                    cpuregs_write  = 1;
                end
                latched_store && !latched_branch: begin
                    cpuregs_wrdata = latched_stalu ? alu_out_q : reg_out;
                    cpuregs_write  = 1;
                end
                ENABLE_IRQ && irq_state[0]: begin
                    cpuregs_wrdata = reg_next_pc | latched_compr;
                    cpuregs_write  = 1;
                end
                ENABLE_IRQ && irq_state[1]: begin
                    cpuregs_wrdata = irq_pending & ~irq_mask;
                    cpuregs_write  = 1;
                end
            endcase
        end
    end

`ifndef PICORV32_REGS
    always @(posedge clk) begin
        if (resetn && cpuregs_write && latched_rd)
            cpuregs[latched_rd] <= cpuregs_wrdata;
    end

    always @* begin
        decoded_rs = 'bx;
        if (ENABLE_REGS_DUALPORT) begin
            cpuregs_rs1 = decoded_rs1 ? cpuregs[decoded_rs1] : 0;
            cpuregs_rs2 = decoded_rs2 ? cpuregs[decoded_rs2] : 0;
        end else begin
            decoded_rs  = (cpu_state == cpu_state_ld_rs2) ? decoded_rs2 : decoded_rs1;
            cpuregs_rs1 = decoded_rs ? cpuregs[decoded_rs] : 0;
            cpuregs_rs2 = cpuregs_rs1;
        end
    end
`endif

    assign launch_next_insn = cpu_state == cpu_state_fetch && decoder_trigger &&
            (!ENABLE_IRQ || irq_delay || irq_active || !(irq_pending & ~irq_mask));

    // =========================================================
    //  MAIN STATE MACHINE
    // =========================================================

    always @(posedge clk) begin
        trap <= 0;
        reg_sh  <= 'bx;
        reg_out <= 'bx;
        set_mem_do_rinst = 0;
        set_mem_do_rdata = 0;
        set_mem_do_wdata = 0;

        alu_out_0_q <= alu_out_0;
        alu_out_q   <= alu_out;

        alu_wait   <= 0;
        alu_wait_2 <= 0;

        if (launch_next_insn) begin
            dbg_rs1val       <= 'bx;
            dbg_rs2val       <= 'bx;
            dbg_rs1val_valid <= 0;
            dbg_rs2val_valid <= 0;
        end

        if (WITH_PCPI && CATCH_ILLINSN) begin
            if (resetn && pcpi_valid && !pcpi_int_wait) begin
                if (pcpi_timeout_counter)
                    pcpi_timeout_counter <= pcpi_timeout_counter - 1;
            end else
                pcpi_timeout_counter <= ~0;
            pcpi_timeout <= !pcpi_timeout_counter;
        end

        if (ENABLE_COUNTERS) begin
            count_cycle <= resetn ? count_cycle + 1 : 0;
            if (!ENABLE_COUNTERS64) count_cycle[63:32] <= 0;
        end else begin
            count_cycle <= 'bx;
            count_instr <= 'bx;
        end

        next_irq_pending = ENABLE_IRQ ? irq_pending & LATCHED_IRQ : 'bx;

        if (ENABLE_IRQ && ENABLE_IRQ_TIMER && timer)
            timer <= timer - 1;

        decoder_trigger         <= mem_do_rinst && mem_done;
        decoder_trigger_q       <= decoder_trigger;
        decoder_pseudo_trigger  <= 0;
        decoder_pseudo_trigger_q <= decoder_pseudo_trigger;
        do_waitirq <= 0;

        trace_valid <= 0;
        if (!ENABLE_TRACE) trace_data <= 'bx;

        if (!resetn) begin
            reg_pc      <= PROGADDR_RESET;
            reg_next_pc <= PROGADDR_RESET;
            if (ENABLE_COUNTERS) count_instr <= 0;
            latched_store  <= 0;
            latched_stalu  <= 0;
            latched_branch <= 0;
            latched_trace  <= 0;
            latched_is_lu  <= 0;
            latched_is_lh  <= 0;
            latched_is_lb  <= 0;
            pcpi_valid     <= 0;
            pcpi_timeout   <= 0;
            irq_active     <= 0;
            irq_delay      <= 0;
            irq_mask       <= ~0;
            next_irq_pending = 0;
            irq_state      <= 0;
            eoi            <= 0;
            timer          <= 0;
            reg_out_hi     <= 0;
            if (~STACKADDR) begin
                latched_store <= 1;
                latched_rd    <= 2;
                reg_out       <= STACKADDR;
            end
            cpu_state <= cpu_state_fetch;
        end else
        (* parallel_case, full_case *)
        case (cpu_state)
            cpu_state_trap: begin
                trap <= 1;
            end

            cpu_state_fetch: begin
                mem_do_rinst <= !decoder_trigger && !do_waitirq;
                mem_wordsize <= 0;

                current_pc = reg_next_pc;

                (* parallel_case *)
                case (1'b1)
                    latched_branch: begin
                        current_pc = latched_store ? (latched_stalu ? alu_out_q : reg_out) & ~1 : reg_next_pc;
                    end
                    latched_store && !latched_branch: begin
                        // store to reg file
                    end
                    ENABLE_IRQ && irq_state[0]: begin
                        current_pc = PROGADDR_IRQ;
                        irq_active <= 1;
                        mem_do_rinst <= 1;
                    end
                    ENABLE_IRQ && irq_state[1]: begin
                        eoi <= irq_pending & ~irq_mask;
                        next_irq_pending = next_irq_pending & irq_mask;
                    end
                endcase

                if (ENABLE_TRACE && latched_trace) begin
                    latched_trace <= 0;
                    trace_valid   <= 1;
                    if (latched_branch)
                        trace_data <= (irq_active ? TRACE_IRQ : 0) | TRACE_BRANCH | (current_pc & 32'hfffffffe);
                    else
                        trace_data <= (irq_active ? TRACE_IRQ : 0) | (latched_stalu ? alu_out_q : reg_out);
                end

                reg_pc      <= current_pc;
                reg_next_pc <= current_pc;

                latched_store  <= 0;
                latched_stalu  <= 0;
                latched_branch <= 0;
                latched_is_lu  <= 0;
                latched_is_lh  <= 0;
                latched_is_lb  <= 0;
                latched_rd     <= decoded_rd;
                latched_compr  <= compressed_instr;

                if (ENABLE_IRQ && ((decoder_trigger && !irq_active && !irq_delay && |(irq_pending & ~irq_mask)) || irq_state)) begin
                    irq_state <= irq_state == 2'b00 ? 2'b01 : irq_state == 2'b01 ? 2'b10 : 2'b00;
                    latched_compr <= latched_compr;
                    if (ENABLE_IRQ_QREGS) latched_rd <= irqregs_offset | irq_state[0];
                    else latched_rd <= irq_state[0] ? 4 : 3;
                end else
                if (ENABLE_IRQ && (decoder_trigger || do_waitirq) && instr_waitirq) begin
                    if (irq_pending) begin
                        latched_store <= 1;
                        reg_out       <= irq_pending;
                        reg_next_pc   <= current_pc + (compressed_instr ? 2 : 4);
                        mem_do_rinst  <= 1;
                    end else
                        do_waitirq <= 1;
                end else
                if (decoder_trigger) begin
                    irq_delay   <= irq_active;
                    reg_next_pc <= current_pc + (compressed_instr ? 2 : 4);
                    if (ENABLE_TRACE) latched_trace <= 1;
                    if (ENABLE_COUNTERS) begin
                        count_instr <= count_instr + 1;
                        if (!ENABLE_COUNTERS64) count_instr[63:32] <= 0;
                    end
                    if (instr_jal) begin
                        mem_do_rinst <= 1;
                        reg_next_pc  <= current_pc + decoded_imm_j;
                        latched_branch <= 1;
                    end else begin
                        mem_do_rinst   <= 0;
                        mem_do_prefetch <= !instr_jalr && !instr_retirq;
                        cpu_state      <= cpu_state_ld_rs1;
                    end
                end
            end

            cpu_state_ld_rs1: begin
                reg_op1 <= 'bx;
                reg_op2 <= 'bx;

                (* parallel_case *)
                case (1'b1)
                    (CATCH_ILLINSN || WITH_PCPI) && instr_trap: begin
                        if (WITH_PCPI) begin
                            reg_op1 <= cpuregs_rs1;
                            dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                            if (ENABLE_REGS_DUALPORT) begin
                                pcpi_valid <= 1;
                                reg_sh  <= cpuregs_rs2;
                                reg_op2 <= cpuregs_rs2;
                                dbg_rs2val <= cpuregs_rs2; dbg_rs2val_valid <= 1;
                                if (pcpi_int_ready) begin
                                    mem_do_rinst  <= 1;
                                    pcpi_valid    <= 0;
                                    reg_out       <= pcpi_int_rd;
                                    latched_store <= pcpi_int_wr;
                                    cpu_state     <= cpu_state_fetch;
                                end else
                                if (CATCH_ILLINSN && (pcpi_timeout || instr_ecall_ebreak)) begin
                                    pcpi_valid <= 0;
                                    if (ENABLE_IRQ && !irq_mask[irq_ebreak] && !irq_active) begin
                                        next_irq_pending[irq_ebreak] = 1;
                                        cpu_state <= cpu_state_fetch;
                                    end else cpu_state <= cpu_state_trap;
                                end else cpu_state <= cpu_state_ld_rs2;
                            end else begin
                                if (ENABLE_IRQ && !irq_mask[irq_ebreak] && !irq_active) begin
                                    next_irq_pending[irq_ebreak] = 1;
                                    cpu_state <= cpu_state_fetch;
                                end else cpu_state <= cpu_state_trap;
                            end
                        end else begin
                            if (ENABLE_IRQ && !irq_mask[irq_ebreak] && !irq_active) begin
                                next_irq_pending[irq_ebreak] = 1;
                                cpu_state <= cpu_state_fetch;
                            end else cpu_state <= cpu_state_trap;
                        end
                    end
                    ENABLE_COUNTERS && is_rdcycle_rdcycleh_rdinstr_rdinstrh: begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            instr_rdcycle:                  reg_out <= count_cycle[31:0];
                            instr_rdcycleh && ENABLE_COUNTERS64: reg_out <= count_cycle[63:32];
                            instr_rdinstr:                  reg_out <= count_instr[31:0];
                            instr_rdinstrh && ENABLE_COUNTERS64: reg_out <= count_instr[63:32];
                        endcase
                        latched_store <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                    is_lui_auipc_jal: begin
                        reg_op1 <= instr_lui ? 0 : reg_pc;
                        reg_op2 <= decoded_imm;
                        if (TWO_CYCLE_ALU) alu_wait <= 1;
                        else mem_do_rinst <= mem_do_prefetch;
                        cpu_state <= cpu_state_exec;
                    end
                    ENABLE_IRQ && ENABLE_IRQ_QREGS && instr_getq: begin
                        reg_out <= cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        latched_store <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                    ENABLE_IRQ && ENABLE_IRQ_QREGS && instr_setq: begin
                        reg_out <= cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        latched_rd <= latched_rd | irqregs_offset;
                        latched_store <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                    ENABLE_IRQ && instr_retirq: begin
                        eoi <= 0;
                        irq_active <= 0;
                        latched_branch <= 1;
                        latched_store  <= 1;
                        reg_out <= CATCH_MISALIGN ? (cpuregs_rs1 & 32'hfffffffe) : cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                    ENABLE_IRQ && instr_maskirq: begin
                        latched_store <= 1;
                        reg_out  <= irq_mask;
                        irq_mask <= cpuregs_rs1 | MASKED_IRQ;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                    ENABLE_IRQ && ENABLE_IRQ_TIMER && instr_timer: begin
                        latched_store <= 1;
                        reg_out <= timer;
                        timer   <= cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                    is_lb_lh_lw_lbu_lhu && !instr_trap: begin
                        reg_op1 <= cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        cpu_state    <= cpu_state_ldmem;
                        mem_do_rinst <= 1;
                    end
                    is_slli_srli_srai && !BARREL_SHIFTER: begin
                        reg_op1 <= cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        reg_sh    <= decoded_rs2;
                        cpu_state <= cpu_state_shift;
                    end
                    is_jalr_addi_slti_sltiu_xori_ori_andi, is_slli_srli_srai && BARREL_SHIFTER: begin
                        reg_op1 <= cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        reg_op2 <= is_slli_srli_srai && BARREL_SHIFTER ? decoded_rs2 : decoded_imm;
                        if (TWO_CYCLE_ALU) alu_wait <= 1;
                        else mem_do_rinst <= mem_do_prefetch;
                        cpu_state <= cpu_state_exec;
                    end
                    default: begin
                        reg_op1 <= cpuregs_rs1;
                        dbg_rs1val <= cpuregs_rs1; dbg_rs1val_valid <= 1;
                        if (ENABLE_REGS_DUALPORT) begin
                            reg_sh  <= cpuregs_rs2;
                            reg_op2 <= cpuregs_rs2;
                            dbg_rs2val <= cpuregs_rs2; dbg_rs2val_valid <= 1;
                            (* parallel_case *)
                            case (1'b1)
                                is_sb_sh_sw: begin cpu_state <= cpu_state_stmem; mem_do_rinst <= 1; end
                                is_sll_srl_sra && !BARREL_SHIFTER: begin cpu_state <= cpu_state_shift; end
                                default: begin
                                    if (TWO_CYCLE_ALU || (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu)) begin
                                        alu_wait_2 <= TWO_CYCLE_ALU && (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu);
                                        alu_wait   <= 1;
                                    end else
                                        mem_do_rinst <= mem_do_prefetch;
                                    cpu_state <= cpu_state_exec;
                                end
                            endcase
                        end else
                            cpu_state <= cpu_state_ld_rs2;
                    end
                endcase
            end

            cpu_state_ld_rs2: begin
                reg_sh  <= cpuregs_rs2;
                reg_op2 <= cpuregs_rs2;
                dbg_rs2val <= cpuregs_rs2; dbg_rs2val_valid <= 1;

                (* parallel_case *)
                case (1'b1)
                    WITH_PCPI && instr_trap: begin
                        pcpi_valid <= 1;
                        if (pcpi_int_ready) begin
                            mem_do_rinst  <= 1;
                            pcpi_valid    <= 0;
                            reg_out       <= pcpi_int_rd;
                            latched_store <= pcpi_int_wr;
                            cpu_state     <= cpu_state_fetch;
                        end else
                        if (CATCH_ILLINSN && (pcpi_timeout || instr_ecall_ebreak)) begin
                            pcpi_valid <= 0;
                            if (ENABLE_IRQ && !irq_mask[irq_ebreak] && !irq_active) begin
                                next_irq_pending[irq_ebreak] = 1;
                                cpu_state <= cpu_state_fetch;
                            end else cpu_state <= cpu_state_trap;
                        end
                    end
                    is_sb_sh_sw: begin cpu_state <= cpu_state_stmem; mem_do_rinst <= 1; end
                    is_sll_srl_sra && !BARREL_SHIFTER: begin cpu_state <= cpu_state_shift; end
                    default: begin
                        if (TWO_CYCLE_ALU || (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu)) begin
                            alu_wait_2 <= TWO_CYCLE_ALU && (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu);
                            alu_wait   <= 1;
                        end else
                            mem_do_rinst <= mem_do_prefetch;
                        cpu_state <= cpu_state_exec;
                    end
                endcase
            end

            cpu_state_exec: begin
                // reg_out_hi captures the upper 32 bits of the RNS 64-bit result
                // for ADD/SUB instructions; zero for all others.
                reg_out    <= reg_pc + decoded_imm;
                reg_out_hi <= (instr_add || instr_sub || instr_addi) ? rns_result_hi : 32'b0;

                if ((TWO_CYCLE_ALU || TWO_CYCLE_COMPARE) && (alu_wait || alu_wait_2)) begin
                    mem_do_rinst <= mem_do_prefetch && !alu_wait_2;
                    alu_wait     <= alu_wait_2;
                end else
                if (is_beq_bne_blt_bge_bltu_bgeu) begin
                    latched_rd     <= 0;
                    latched_store  <= TWO_CYCLE_COMPARE ? alu_out_0_q : alu_out_0;
                    latched_branch <= TWO_CYCLE_COMPARE ? alu_out_0_q : alu_out_0;
                    if (mem_done) cpu_state <= cpu_state_fetch;
                    if (TWO_CYCLE_COMPARE ? alu_out_0_q : alu_out_0) begin
                        decoder_trigger  <= 0;
                        set_mem_do_rinst = 1;
                    end
                end else begin
                    latched_branch <= instr_jalr;
                    latched_store  <= 1;
                    latched_stalu  <= 1;
                    cpu_state      <= cpu_state_fetch;
                end
            end

            cpu_state_shift: begin
                latched_store <= 1;
                if (reg_sh == 0) begin
                    reg_out      <= reg_op1;
                    mem_do_rinst <= mem_do_prefetch;
                    cpu_state    <= cpu_state_fetch;
                end else if (TWO_STAGE_SHIFT && reg_sh >= 4) begin
                    (* parallel_case, full_case *)
                    case (1'b1)
                        instr_slli || instr_sll: reg_op1 <= reg_op1 << 4;
                        instr_srli || instr_srl: reg_op1 <= reg_op1 >> 4;
                        instr_srai || instr_sra: reg_op1 <= $signed(reg_op1) >>> 4;
                    endcase
                    reg_sh <= reg_sh - 4;
                end else begin
                    (* parallel_case, full_case *)
                    case (1'b1)
                        instr_slli || instr_sll: reg_op1 <= reg_op1 << 1;
                        instr_srli || instr_srl: reg_op1 <= reg_op1 >> 1;
                        instr_srai || instr_sra: reg_op1 <= $signed(reg_op1) >>> 1;
                    endcase
                    reg_sh <= reg_sh - 1;
                end
            end

            cpu_state_stmem: begin
                if (ENABLE_TRACE) reg_out <= reg_op2;
                if (!mem_do_prefetch || mem_done) begin
                    if (!mem_do_wdata) begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            instr_sb: mem_wordsize <= 2;
                            instr_sh: mem_wordsize <= 1;
                            instr_sw: mem_wordsize <= 0;
                        endcase
                        if (ENABLE_TRACE) begin
                            trace_valid <= 1;
                            trace_data  <= (irq_active ? TRACE_IRQ : 0) | TRACE_ADDR | ((reg_op1 + decoded_imm) & 32'hffffffff);
                        end
                        reg_op1      <= reg_op1 + decoded_imm;
                        set_mem_do_wdata = 1;
                    end
                    if (!mem_do_prefetch && mem_done) begin
                        cpu_state           <= cpu_state_fetch;
                        decoder_trigger     <= 1;
                        decoder_pseudo_trigger <= 1;
                    end
                end
            end

            cpu_state_ldmem: begin
                latched_store <= 1;
                if (!mem_do_prefetch || mem_done) begin
                    if (!mem_do_rdata) begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            instr_lb || instr_lbu: mem_wordsize <= 2;
                            instr_lh || instr_lhu: mem_wordsize <= 1;
                            instr_lw:              mem_wordsize <= 0;
                        endcase
                        latched_is_lu <= is_lbu_lhu_lw;
                        latched_is_lh <= instr_lh;
                        latched_is_lb <= instr_lb;
                        if (ENABLE_TRACE) begin
                            trace_valid <= 1;
                            trace_data  <= (irq_active ? TRACE_IRQ : 0) | TRACE_ADDR | ((reg_op1 + decoded_imm) & 32'hffffffff);
                        end
                        reg_op1      <= reg_op1 + decoded_imm;
                        set_mem_do_rdata = 1;
                    end
                    if (!mem_do_prefetch && mem_done) begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            latched_is_lu: reg_out <= mem_rdata_word;
                            latched_is_lh: reg_out <= $signed(mem_rdata_word[15:0]);
                            latched_is_lb: reg_out <= $signed(mem_rdata_word[7:0]);
                        endcase
                        decoder_trigger     <= 1;
                        decoder_pseudo_trigger <= 1;
                        cpu_state           <= cpu_state_fetch;
                    end
                end
            end
        endcase

        if (ENABLE_IRQ) begin
            next_irq_pending = next_irq_pending | irq;
            if (ENABLE_IRQ_TIMER && timer)
                if (timer - 1 == 0)
                    next_irq_pending[irq_timer] = 1;
        end

        if (CATCH_MISALIGN && resetn && (mem_do_rdata || mem_do_wdata)) begin
            if (mem_wordsize == 0 && reg_op1[1:0] != 0) begin
                if (ENABLE_IRQ && !irq_mask[irq_buserror] && !irq_active)
                    next_irq_pending[irq_buserror] = 1;
                else cpu_state <= cpu_state_trap;
            end
            if (mem_wordsize == 1 && reg_op1[0] != 0) begin
                if (ENABLE_IRQ && !irq_mask[irq_buserror] && !irq_active)
                    next_irq_pending[irq_buserror] = 1;
                else cpu_state <= cpu_state_trap;
            end
        end
        if (CATCH_MISALIGN && resetn && mem_do_rinst && (COMPRESSED_ISA ? reg_pc[0] : |reg_pc[1:0])) begin
            if (ENABLE_IRQ && !irq_mask[irq_buserror] && !irq_active)
                next_irq_pending[irq_buserror] = 1;
            else cpu_state <= cpu_state_trap;
        end
        if (!CATCH_ILLINSN && decoder_trigger_q && !decoder_pseudo_trigger_q && instr_ecall_ebreak)
            cpu_state <= cpu_state_trap;

        if (!resetn || mem_done) begin
            mem_do_prefetch <= 0;
            mem_do_rinst    <= 0;
            mem_do_rdata    <= 0;
            mem_do_wdata    <= 0;
        end

        if (set_mem_do_rinst) mem_do_rinst <= 1;
        if (set_mem_do_rdata) mem_do_rdata <= 1;
        if (set_mem_do_wdata) mem_do_wdata <= 1;

        irq_pending <= next_irq_pending & ~MASKED_IRQ;

        if (!CATCH_MISALIGN) begin
            if (COMPRESSED_ISA) begin
                reg_pc[0]      <= 0;
                reg_next_pc[0] <= 0;
            end else begin
                reg_pc[1:0]      <= 0;
                reg_next_pc[1:0] <= 0;
            end
        end
        current_pc = 'bx;
    end

endmodule

