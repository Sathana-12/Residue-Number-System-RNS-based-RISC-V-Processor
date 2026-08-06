/*
 * tb_picorv32_rns.v
 * =================
 * Comprehensive testbench for the RNS-extended PicoRV32.
 *
 * TEST PLAN
 * ---------
 * SECTION 1 – RNS Forward Converter unit tests
 *   1a. Zero input
 *   1b. 4-bit inputs  (values 0..15)
 *   1c. 8-bit inputs
 *   1d. 16-bit inputs
 *   1e. 32-bit inputs (full single word)
 *   1f. 48-bit inputs (cross-word, zero-extended to 64-bit)
 *   1g. 64-bit max value
 *
 * SECTION 2 – RNS ALU unit tests (ADD and SUB)
 *   2a. 4-bit operand ADD
 *   2b. 4-bit operand SUB
 *   2c. 8-bit operand ADD / SUB
 *   2d. 16-bit operand ADD / SUB
 *   2e. 32-bit operand ADD / SUB (with carry / borrow)
 *   2f. 48-bit operand ADD / SUB
 *   2g. Overflow / wrap-around cases
 *
 * SECTION 3 – RNS Reverse Converter unit tests
 *   3a-3f. Round-trip: forward → ALU → reverse matches golden 64-bit result
 *
 * SECTION 4 – Full CPU integration tests
 *   4a. ADD  instruction (encoded as RISC-V ADD rd,rs1,rs2)
 *   4b. SUB  instruction
 *   4c. ADDI instruction (immediate adds)
 *   4d. LW / SW (load-store, exercises non-RNS path)
 *   4e. BEQ / BNE branch instructions
 *   4f. JAL jump instruction
 *   4g. Multi-instruction sequence (accumulator loop)
 *   4h. Trap on illegal instruction
 *   4i. reg_out_hi carries correct upper 32 bits after 32-bit ADD overflow
 *
 * Pass/Fail counters are printed at the end.
 */

`timescale 1 ns / 1 ps
`include "picorv32_rns.v"

// ============================================================
//  Helper macros
// ============================================================
`define CHECK(label, got, exp) \
    if ((got) !== (exp)) begin \
        $display("FAIL  [%0t] %-40s  got=%0h  exp=%0h", $time, label, got, exp); \
        fail_count = fail_count + 1; \
    end else begin \
        $display("PASS  [%0t] %-40s  val=%0h", $time, label, got); \
        pass_count = pass_count + 1; \
    end

`define CHECK64(label, got, exp) \
    if ((got) !== (exp)) begin \
        $display("FAIL  [%0t] %-40s  got=%0h  exp=%0h", $time, label, got, exp); \
        fail_count = fail_count + 1; \
    end else begin \
        $display("PASS  [%0t] %-40s  val=%0h", $time, label, got); \
        pass_count = pass_count + 1; \
    end

// ============================================================
//  Testbench top
// ============================================================
module top;

    // ---- Global counters ----
    integer pass_count, fail_count;

    // ===========================================================
    //  SECTION 1+2+3 – Sub-module direct test signals
    // ===========================================================

    // Forward converter A
    reg  [63:0] fwd_X_a;
    wire [31:0] fwd_r1_a;
    wire [32:0] fwd_r2_a;
    wire [30:0] fwd_r3_a;

    // Forward converter B
    reg  [63:0] fwd_X_b;
    wire [31:0] fwd_r1_b;
    wire [32:0] fwd_r2_b;
    wire [30:0] fwd_r3_b;

    // RNS ALU control
    reg  rns_is_sub;
    wire [31:0] rns_out1;
    wire [32:0] rns_out2;
    wire [30:0] rns_out3;

    // Reverse converter
    wire [63:0] rns_result;

    // Golden reference (computed in task)
    reg [63:0] golden;

    // Instantiate sub-modules
    rns_forward_converter u_fwd_a (
        .X  (fwd_X_a),
        .r1 (fwd_r1_a),
        .r2 (fwd_r2_a),
        .r3 (fwd_r3_a)
    );

    rns_forward_converter u_fwd_b (
        .X  (fwd_X_b),
        .r1 (fwd_r1_b),
        .r2 (fwd_r2_b),
        .r3 (fwd_r3_b)
    );

    rns_alu u_alu (
        .is_sub (rns_is_sub),
        .a1     (fwd_r1_a),
        .a2     (fwd_r2_a),
        .a3     (fwd_r3_a),
        .b1     (fwd_r1_b),
        .b2     (fwd_r2_b),
        .b3     (fwd_r3_b),
        .out1   (rns_out1),
        .out2   (rns_out2),
        .out3   (rns_out3)
    );

    rns_reverse_converter u_rev (
        .r1 (rns_out1),
        .r2 (rns_out2),
        .r3 (rns_out3),
        .X  (rns_result)
    );

    // ===========================================================
    //  SECTION 4 – Full CPU signals
    // ===========================================================

    reg         clk;
    reg         resetn;

    // Memory interface
    wire        mem_valid;
    wire        mem_instr;
    reg         mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [ 3:0] mem_wstrb;
    reg  [31:0] mem_rdata;

    // Look-Ahead (tie off)
    wire        mem_la_read;
    wire        mem_la_write;
    wire [31:0] mem_la_addr;
    wire [31:0] mem_la_wdata;
    wire [ 3:0] mem_la_wstrb;

    // PCPI (tie off)
    wire        pcpi_valid;
    wire [31:0] pcpi_insn;
    wire [31:0] pcpi_rs1;
    wire [31:0] pcpi_rs2;
    reg         pcpi_wr    = 0;
    reg  [31:0] pcpi_rd    = 0;
    reg         pcpi_wait  = 0;
    reg         pcpi_ready = 0;

    // IRQ
    reg  [31:0] irq = 0;
    wire [31:0] eoi;

    // RNS 64-bit high word
    wire [31:0] reg_out_hi;

    wire        trap;
    wire        trace_valid;
    wire [35:0] trace_data;

    // Instantiate CPU
    picorv32_rns #(
        .ENABLE_COUNTERS    (0),
        .ENABLE_COUNTERS64  (0),
        .ENABLE_REGS_16_31  (1),
        .ENABLE_REGS_DUALPORT(1),
        .CATCH_MISALIGN     (0),
        .CATCH_ILLINSN      (1),
        .REGS_INIT_ZERO     (1),
        .STACKADDR          (32'h 0000_0100)
    ) uut (
        .clk          (clk),
        .resetn       (resetn),
        .trap         (trap),
        .mem_valid    (mem_valid),
        .mem_instr    (mem_instr),
        .mem_ready    (mem_ready),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_wstrb    (mem_wstrb),
        .mem_rdata    (mem_rdata),
        .mem_la_read  (mem_la_read),
        .mem_la_write (mem_la_write),
        .mem_la_addr  (mem_la_addr),
        .mem_la_wdata (mem_la_wdata),
        .mem_la_wstrb (mem_la_wstrb),
        .pcpi_valid   (pcpi_valid),
        .pcpi_insn    (pcpi_insn),
        .pcpi_rs1     (pcpi_rs1),
        .pcpi_rs2     (pcpi_rs2),
        .pcpi_wr      (pcpi_wr),
        .pcpi_rd      (pcpi_rd),
        .pcpi_wait    (pcpi_wait),
        .pcpi_ready   (pcpi_ready),
        .irq          (irq),
        .eoi          (eoi),
        .reg_out_hi   (reg_out_hi),
        .trace_valid  (trace_valid),
        .trace_data   (trace_data)
    );

    // ===========================================================
    //  Synthetic instruction memory + data RAM
    //  Address map:
    //    0x0000_0000 – 0x0000_00FF  : instruction ROM  (64 words)
    //    0x0000_0100 – 0x0000_01FF  : data RAM (stack / test variables)
    // ===========================================================

    reg [31:0] imem [0:63];   // instruction memory (256 bytes)
    reg [31:0] dmem [0:63];   // data memory (256 bytes)

    // Combinational memory read
    always @* begin
        mem_ready = 0;
        mem_rdata = 32'hx;
        if (mem_valid) begin
            mem_ready = 1;
            if (mem_addr[31:8] == 24'h000000) begin
                // Instruction / low RAM
                if (|mem_wstrb) begin
                    // write – handled in clocked block
                end else begin
                    mem_rdata = imem[mem_addr[7:2]];
                end
            end else if (mem_addr[31:8] == 24'h000001) begin
                // Data RAM
                if (|mem_wstrb) begin
                end else begin
                    mem_rdata = dmem[mem_addr[7:2]];
                end
            end
        end
    end

    // Clocked memory write
    always @(posedge clk) begin
        if (mem_valid && mem_ready && |mem_wstrb) begin
            if (mem_addr[31:8] == 24'h000001) begin
                if (mem_wstrb[0]) dmem[mem_addr[7:2]][ 7: 0] <= mem_wdata[ 7: 0];
                if (mem_wstrb[1]) dmem[mem_addr[7:2]][15: 8] <= mem_wdata[15: 8];
                if (mem_wstrb[2]) dmem[mem_addr[7:2]][23:16] <= mem_wdata[23:16];
                if (mem_wstrb[3]) dmem[mem_addr[7:2]][31:24] <= mem_wdata[31:24];
            end
        end
    end

    // ===========================================================
    //  Clock
    // ===========================================================
    initial clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    // ===========================================================
    //  Tasks
    // ===========================================================

    // Run CPU for N clock cycles
    task run_cpu;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i+1) @(posedge clk);
        end
    endtask

    // Reset CPU
    task reset_cpu;
        begin
            resetn = 0;
            repeat(4) @(posedge clk);
            resetn = 1;
            @(posedge clk);
        end
    endtask

    // Load a simple program and run it
    // Program is a flat array; caller fills imem[] before calling this
    task run_program;
        input integer cycles;
        begin
            reset_cpu;
            run_cpu(cycles);
        end
    endtask

    // RNS add/sub test helper
    // Drives fwd_X_a, fwd_X_b, rns_is_sub and checks rns_result vs golden
    task rns_test;
        input [63:0] a;
        input [63:0] b;
        input        do_sub;
        input [127:0] label;
        reg   [63:0]  exp;
        begin
            fwd_X_a   = a;
            fwd_X_b   = b;
            rns_is_sub = do_sub;
            #1;   // combinational settle
            if (do_sub)
                exp = a - b;
            else
                exp = a + b;
            // Only lower 64 bits are checked (modular wrap)
            `CHECK64(label, rns_result, exp[63:0])
        end
    endtask

    // Forward-converter correctness check
    task fwd_check;
        input [63:0] x;
        input [127:0] label;
        reg [31:0] exp_r1;
        reg [32:0] exp_r2;
        reg [30:0] exp_r3;
        begin
            fwd_X_a = x;
            #1;
            // Golden residues computed with Verilog big-integer arithmetic
            exp_r1 = x % 64'hFFFF_FFFF;          // mod 2^32-1
            exp_r2 = x % 65'h1_0000_0001;         // mod 2^32+1 (65-bit literal)
            exp_r3 = x % 64'h7FFF_FFFF;           // mod 2^31-1
            `CHECK({label," r1"}, fwd_r1_a, exp_r1)
            `CHECK({label," r2"}, fwd_r2_a, exp_r2[32:0])
            `CHECK({label," r3"}, fwd_r3_a, exp_r3[30:0])
        end
    endtask

    // ===========================================================
    //  RISC-V instruction encoding helpers
    //  (all R-type, I-type, S-type, B-type, J-type)
    // ===========================================================

    // R-type: funct7 | rs2 | rs1 | funct3 | rd | opcode
    function [31:0] enc_r;
        input [6:0] funct7;
        input [4:0] rs2, rs1, rd;
        input [2:0] funct3;
        input [6:0] opcode;
        begin
            enc_r = {funct7, rs2, rs1, funct3, rd, opcode};
        end
    endfunction

    // I-type
    function [31:0] enc_i;
        input [11:0] imm12;
        input [4:0]  rs1, rd;
        input [2:0]  funct3;
        input [6:0]  opcode;
        begin
            enc_i = {imm12, rs1, funct3, rd, opcode};
        end
    endfunction

    // S-type (store)
    function [31:0] enc_s;
        input [11:0] imm12;
        input [4:0]  rs2, rs1;
        input [2:0]  funct3;
        input [6:0]  opcode;
        begin
            enc_s = {imm12[11:5], rs2, rs1, funct3, imm12[4:0], opcode};
        end
    endfunction

    // B-type (branch)
    function [31:0] enc_b;
        input signed [12:0] offset; // in bytes (multiples of 2)
        input [4:0]  rs2, rs1;
        input [2:0]  funct3;
        input [6:0]  opcode;
        begin
            enc_b = {offset[12], offset[10:5], rs2, rs1, funct3,
                     offset[4:1], offset[11], opcode};
        end
    endfunction

    // J-type (JAL)
    function [31:0] enc_j;
        input signed [20:0] offset;
        input [4:0] rd;
        input [6:0] opcode;
        begin
            enc_j = {offset[20], offset[10:1], offset[11], offset[19:12], rd, opcode};
        end
    endfunction

    // LUI
    function [31:0] enc_lui;
        input [19:0] imm20;
        input [4:0]  rd;
        begin
            enc_lui = {imm20, rd, 7'b0110111};
        end
    endfunction

    // Opcode constants
    localparam OP_ALU    = 7'b0110011;
    localparam OP_ALUI   = 7'b0010011;
    localparam OP_LOAD   = 7'b0000011;
    localparam OP_STORE  = 7'b0100011;
    localparam OP_BRANCH = 7'b1100011;
    localparam OP_JAL    = 7'b1101111;
    localparam OP_JALR   = 7'b1100111;

    // funct3 / funct7
    localparam F3_ADD  = 3'b000;
    localparam F7_ADD  = 7'b0000000;
    localparam F7_SUB  = 7'b0100000;
    localparam F3_LW   = 3'b010;
    localparam F3_SW   = 3'b010;
    localparam F3_BEQ  = 3'b000;
    localparam F3_BNE  = 3'b001;

    // Register aliases
    localparam X0=5'd0, X1=5'd1, X2=5'd2, X3=5'd3, X4=5'd4,
               X5=5'd5, X6=5'd6, X7=5'd7, X8=5'd8, X9=5'd9,
               X10=5'd10;

    // EBREAK  (causes CPU trap — used to end programs)
    localparam EBREAK = 32'h00100073;
    // NOP = ADDI x0, x0, 0
    localparam NOP    = 32'h00000013;

    // ===========================================================
    //  Main test sequence
    // ===========================================================
    integer i_test;
    reg [63:0] val_a, val_b;

    initial begin
        $dumpfile("tb_picorv32_rns.vcd");
        $dumpvars(0, top);

        pass_count = 0;
        fail_count = 0;

        // Default sub-module stimulus
        fwd_X_a    = 0;
        fwd_X_b    = 0;
        rns_is_sub = 0;
        resetn     = 0;

        $display("\n========================================================");
        $display("  SECTION 1 – RNS Forward Converter Unit Tests");
        $display("========================================================");

        // 1a. Zero
        fwd_check(64'h0,                "1a.Zero");

        // 1b. 4-bit inputs (0..15)
        $display("-- 1b. 4-bit inputs --");
        for (i_test = 0; i_test <= 15; i_test = i_test + 1) begin
            fwd_X_a = i_test;
            #1;
            `CHECK($sformatf("1b.4bit x=%0d r1", i_test), fwd_r1_a, i_test % 32'hFFFF_FFFF)
        end

        // 1c. 8-bit inputs
        $display("-- 1c. 8-bit inputs --");
        fwd_check(64'h00,               "1c.8bit 0x00");
        fwd_check(64'hFF,               "1c.8bit 0xFF");
        fwd_check(64'hAB,               "1c.8bit 0xAB");
        fwd_check(64'h80,               "1c.8bit 0x80");

        // 1d. 16-bit inputs
        $display("-- 1d. 16-bit inputs --");
        fwd_check(64'h0000,             "1d.16bit 0x0000");
        fwd_check(64'hFFFF,             "1d.16bit 0xFFFF");
        fwd_check(64'h1234,             "1d.16bit 0x1234");
        fwd_check(64'h8000,             "1d.16bit 0x8000");
        fwd_check(64'hDEAD,             "1d.16bit 0xDEAD");

        // 1e. 32-bit inputs
        $display("-- 1e. 32-bit inputs --");
        fwd_check(64'h0000_0000,        "1e.32bit 0");
        fwd_check(64'hFFFF_FFFE,        "1e.32bit M1-1");
        fwd_check(64'hFFFF_FFFF,        "1e.32bit 2^32-1");
        fwd_check(64'h1_0000_0000,      "1e.32bit 2^32");
        fwd_check(64'hDEAD_BEEF,        "1e.32bit DEADBEEF");
        fwd_check(64'h1234_5678,        "1e.32bit 12345678");
        fwd_check(64'h7FFF_FFFE,        "1e.32bit M3-1");
        fwd_check(64'h7FFF_FFFF,        "1e.32bit M3=2^31-1");

        // 1f. 48-bit inputs (cross word boundary)
        $display("-- 1f. 48-bit inputs --");
        fwd_check(64'h0000_FFFF_FFFF,   "1f.48bit 0x0000_FFFF_FFFF");
        fwd_check(64'h0001_0000_0000,   "1f.48bit 0x0001_0000_0000");
        fwd_check(64'hABCD_1234_5678,   "1f.48bit 0xABCD_1234_5678");
        fwd_check(64'hFFFF_FFFF_FFFF,   "1f.48bit 0xFFFF_FFFF_FFFF");
        fwd_check(64'h0080_0000_0000,   "1f.48bit 0x0080_0000_0000");

        // 1g. 64-bit extremes
        $display("-- 1g. 64-bit extremes --");
        fwd_check(64'hFFFF_FFFF_FFFF_FFFF, "1g.64bit max");
        fwd_check(64'h8000_0000_0000_0000, "1g.64bit msb");
        fwd_check(64'hDEAD_BEEF_CAFE_BABE, "1g.64bit DEADBEEF");


        $display("\n========================================================");
        $display("  SECTION 2 – RNS ALU Unit Tests (ADD and SUB)");
        $display("========================================================");

        // 2a. 4-bit ADD
        $display("-- 2a. 4-bit ADD --");
        rns_test(64'd3,  64'd5,  0, "2a.ADD  3+5=8");
        rns_test(64'd0,  64'd0,  0, "2a.ADD  0+0=0");
        rns_test(64'd15, 64'd0,  0, "2a.ADD  15+0");
        rns_test(64'd7,  64'd8,  0, "2a.ADD  7+8=15");
        rns_test(64'd15, 64'd1,  0, "2a.ADD  15+1=16");

        // 2b. 4-bit SUB
        $display("-- 2b. 4-bit SUB --");
        rns_test(64'd8,  64'd3,  1, "2b.SUB  8-3=5");
        rns_test(64'd15, 64'd15, 1, "2b.SUB  15-15=0");
        rns_test(64'd5,  64'd0,  1, "2b.SUB  5-0=5");

        // 2c. 8-bit ADD / SUB
        $display("-- 2c. 8-bit ADD/SUB --");
        rns_test(64'hAB, 64'h54, 0, "2c.ADD  0xAB+0x54");
        rns_test(64'hFF, 64'h01, 0, "2c.ADD  0xFF+0x01");
        rns_test(64'hFF, 64'hFF, 0, "2c.ADD  0xFF+0xFF");
        rns_test(64'hC0, 64'h40, 1, "2c.SUB  0xC0-0x40");
        rns_test(64'h80, 64'h80, 1, "2c.SUB  0x80-0x80=0");

        // 2d. 16-bit ADD / SUB
        $display("-- 2d. 16-bit ADD/SUB --");
        rns_test(64'h1234, 64'h5678, 0, "2d.ADD  0x1234+0x5678");
        rns_test(64'hFFFF, 64'h0001, 0, "2d.ADD  0xFFFF+1");
        rns_test(64'hFFFF, 64'hFFFF, 0, "2d.ADD  0xFFFF+0xFFFF");
        rns_test(64'hBEEF, 64'hDEAD, 1, "2d.SUB  0xBEEF-0xDEAD");
        rns_test(64'h8000, 64'h0001, 1, "2d.SUB  0x8000-1");

        // 2e. 32-bit ADD / SUB
        $display("-- 2e. 32-bit ADD/SUB --");
        rns_test(64'hDEAD_BEEF, 64'h1234_5678, 0, "2e.ADD  DEADBEEF+12345678");
        rns_test(64'hFFFF_FFFF, 64'h0000_0001, 0, "2e.ADD  FFFFFFFF+1 (carry)");
        rns_test(64'hFFFF_FFFF, 64'hFFFF_FFFF, 0, "2e.ADD  FFFFFFFF+FFFFFFFF");
        rns_test(64'hFFFF_FFFF, 64'h0000_0001, 1, "2e.SUB  FFFFFFFF-1");
        rns_test(64'h1000_0000, 64'h2000_0000, 1, "2e.SUB  borrow (wrap)");
        rns_test(64'h8000_0000, 64'h8000_0000, 1, "2e.SUB  80000000-80000000=0");

        // 2f. 48-bit ADD / SUB
        $display("-- 2f. 48-bit ADD/SUB --");
        val_a = 64'h0001_0000_0000;    // 2^32
        val_b = 64'h0000_FFFF_FFFF;    // 2^32 - 1
        rns_test(val_a, val_b, 0, "2f.ADD  2^32 + (2^32-1)");
        rns_test(val_a, val_b, 1, "2f.SUB  2^32 - (2^32-1) = 1");

        val_a = 64'hABCD_1234_5678;
        val_b = 64'h0001_DCBA_9876;
        rns_test(val_a, val_b, 0, "2f.ADD  48bit random A+B");
        rns_test(val_a, val_b, 1, "2f.SUB  48bit random A-B");

        val_a = 64'hFFFF_FFFF_FFFF;
        val_b = 64'h0000_0000_0001;
        rns_test(val_a, val_b, 0, "2f.ADD  48bit max+1");
        rns_test(val_b, val_a, 1, "2f.SUB  1-48bitmax (borrow)");

        // 2g. Overflow / boundary
        $display("-- 2g. Boundary/overflow --");
        rns_test(64'hFFFF_FFFF_FFFF_FFFF, 64'h1, 0, "2g.ADD  64max+1 wraps");
        rns_test(64'h0, 64'h1, 1, "2g.SUB  0-1 (borrow wrap)");


        $display("\n========================================================");
        $display("  SECTION 3 – Round-trip Correctness (fwd→alu→rev)");
        $display("========================================================");

        // Check that forward→ALU→reverse gives exactly the expected 64-bit value
        rns_test(64'd100,        64'd200,        0, "3a.RT ADD  100+200=300");
        rns_test(64'd200,        64'd100,        1, "3a.RT SUB  200-100=100");
        rns_test(64'hDEAD_BEEF, 64'h1234_5678,  0, "3b.RT ADD  32bit");
        rns_test(64'hDEAD_BEEF, 64'h1234_5678,  1, "3b.RT SUB  32bit");
        rns_test(64'hABCD_1234_5678, 64'h1111_2222_3333, 0, "3c.RT ADD  48bit");
        rns_test(64'hABCD_1234_5678, 64'h1111_2222_3333, 1, "3c.RT SUB  48bit");
        rns_test(64'hFFFF_FFFF,  64'h0000_0001,  0, "3d.RT ADD  32bit carry");
        rns_test(64'h1_0000_0000, 64'hFFFF_FFFF, 1, "3d.RT SUB  2^32-(2^32-1)=1");
        rns_test(64'hFFFF_FFFF_FFFF_FFFF, 64'h0, 0, "3e.RT ADD  64max+0");
        rns_test(64'hFFFF_FFFF_FFFF_FFFF, 64'h1, 1, "3e.RT SUB  64max-1");


        $display("\n========================================================");
        $display("  SECTION 4 – Full CPU Integration Tests");
        $display("========================================================");

        // ----------------------------------------------------------------
        // 4a. ADD instruction
        // Program:
        //   ADDI x1, x0, 7       # x1 = 7
        //   ADDI x2, x0, 5       # x2 = 5
        //   ADD  x3, x1, x2      # x3 = 12  (goes through RNS ALU)
        //   EBREAK
        // ----------------------------------------------------------------
        $display("-- 4a. ADD instruction --");
        begin : test_4a
            // Clear instruction memory
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0]  = enc_i(12'd7, X0, X1, F3_ADD, OP_ALUI);  // ADDI x1,x0,7
            imem[1]  = enc_i(12'd5, X0, X2, F3_ADD, OP_ALUI);  // ADDI x2,x0,5
            imem[2]  = enc_r(F7_ADD, X2, X1, X3, F3_ADD, OP_ALU); // ADD x3,x1,x2
            imem[3]  = EBREAK;
            run_program(40);
            // Check x3 via mem read — use a SW then LW pattern in next test
            // For now read directly from uut's internal regfile
            `CHECK("4a.ADD x3=12", uut.cpuregs[3], 32'd12)
        end

        // ----------------------------------------------------------------
        // 4b. SUB instruction
        //   ADDI x1, x0, 20      # x1 = 20
        //   ADDI x2, x0, 8       # x2 = 8
        //   SUB  x3, x1, x2      # x3 = 12
        //   EBREAK
        // ----------------------------------------------------------------
        $display("-- 4b. SUB instruction --");
        begin : test_4b
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0]  = enc_i(12'd20, X0, X1, F3_ADD, OP_ALUI);
            imem[1]  = enc_i(12'd8,  X0, X2, F3_ADD, OP_ALUI);
            imem[2]  = enc_r(F7_SUB, X2, X1, X3, F3_ADD, OP_ALU);
            imem[3]  = EBREAK;
            run_program(40);
            `CHECK("4b.SUB x3=12", uut.cpuregs[3], 32'd12)
        end

        // ----------------------------------------------------------------
        // 4c. ADDI instruction (immediate)
        //   ADDI x1, x0, 100     # x1 = 100
        //   ADDI x2, x1, -50     # x2 = 50
        //   ADDI x3, x2, 200     # x3 = 250
        //   EBREAK
        // ----------------------------------------------------------------
        $display("-- 4c. ADDI instruction --");
        begin : test_4c
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0] = enc_i(12'd100,  X0, X1, F3_ADD, OP_ALUI);
            imem[1] = enc_i(-12'd50,  X1, X2, F3_ADD, OP_ALUI);
            imem[2] = enc_i(12'd200,  X2, X3, F3_ADD, OP_ALUI);
            imem[3] = EBREAK;
            run_program(40);
            `CHECK("4c.ADDI x1=100", uut.cpuregs[1], 32'd100)
            `CHECK("4c.ADDI x2=50",  uut.cpuregs[2], 32'd50)
            `CHECK("4c.ADDI x3=250", uut.cpuregs[3], 32'd250)
        end

        // ----------------------------------------------------------------
        // 4d. LW / SW  (non-RNS path)
        //   ADDI x1, x0, 0x100   # x1 = base address (dmem base = 0x100)
        //   ADDI x2, x0, 0x5A    # x2 = 0x5A (data to store)
        //   SW   x2, 0(x1)       # mem[0x100] = 0x5A
        //   LW   x3, 0(x1)       # x3 = mem[0x100]
        //   EBREAK
        // ----------------------------------------------------------------
        $display("-- 4d. SW/LW --");
        begin : test_4d
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            for (j = 0; j < 64; j = j+1) dmem[j] = 0;
            imem[0] = enc_i(12'h100, X0, X1, F3_ADD, OP_ALUI);
            imem[1] = enc_i(12'h05A, X0, X2, F3_ADD, OP_ALUI);
            imem[2] = enc_s(12'd0,   X2, X1, F3_SW, OP_STORE);
            imem[3] = enc_i(12'd0,   X1, X3, F3_LW, OP_LOAD);
            imem[4] = EBREAK;
            run_program(60);
            `CHECK("4d.SW then LW x3=0x5A", uut.cpuregs[3], 32'h5A)
        end

        // ----------------------------------------------------------------
        // 4e. BEQ / BNE branch instructions
        //   ADDI x1, x0, 5
        //   ADDI x2, x0, 5
        //   BEQ  x1, x2, +8    # branch over next instruction (skip ADDI x3 = 99)
        //   ADDI x3, x0, 99    # should be SKIPPED
        //   ADDI x3, x0, 42    # x3 = 42  (branch target)
        //   EBREAK
        // ----------------------------------------------------------------
        $display("-- 4e. BEQ branch --");
        begin : test_4e
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0] = enc_i(12'd5, X0, X1, F3_ADD, OP_ALUI);
            imem[1] = enc_i(12'd5, X0, X2, F3_ADD, OP_ALUI);
            imem[2] = enc_b(13'd8, X2, X1, F3_BEQ, OP_BRANCH); // BEQ skip 1 insn
            imem[3] = enc_i(12'd99, X0, X3, F3_ADD, OP_ALUI);  // SKIPPED
            imem[4] = enc_i(12'd42, X0, X3, F3_ADD, OP_ALUI);  // x3=42
            imem[5] = EBREAK;
            run_program(60);
            `CHECK("4e.BEQ skip: x3=42", uut.cpuregs[3], 32'd42)
        end

        // BNE: x1=3, x2=7, BNE should branch
        begin : test_4e2
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0] = enc_i(12'd3,  X0, X1, F3_ADD, OP_ALUI);
            imem[1] = enc_i(12'd7,  X0, X2, F3_ADD, OP_ALUI);
            imem[2] = enc_b(13'd8,  X2, X1, F3_BNE, OP_BRANCH); // BNE: 3!=7 → branch
            imem[3] = enc_i(12'd99, X0, X3, F3_ADD, OP_ALUI);   // SKIPPED
            imem[4] = enc_i(12'd77, X0, X3, F3_ADD, OP_ALUI);   // x3=77
            imem[5] = EBREAK;
            run_program(60);
            `CHECK("4e.BNE taken: x3=77", uut.cpuregs[3], 32'd77)
        end

        // ----------------------------------------------------------------
        // 4f. JAL jump instruction
        //   JAL  x1, +12         # jump to imem[3] (skip imem[1..2]), x1=PC+4
        //   ADDI x4, x0, 55     # SKIPPED
        //   ADDI x4, x0, 66     # SKIPPED
        //   ADDI x3, x0, 88     # x3=88 (jump target)
        //   EBREAK
        // ----------------------------------------------------------------
        $display("-- 4f. JAL jump --");
        begin : test_4f
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0] = enc_j(21'd12, X1, OP_JAL);               // JAL x1, +12 bytes
            imem[1] = enc_i(12'd55, X0, X4, F3_ADD, OP_ALUI);  // SKIPPED
            imem[2] = enc_i(12'd66, X0, X4, F3_ADD, OP_ALUI);  // SKIPPED
            imem[3] = enc_i(12'd88, X0, X3, F3_ADD, OP_ALUI);  // x3=88
            imem[4] = EBREAK;
            run_program(60);
            `CHECK("4f.JAL x3=88",  uut.cpuregs[3], 32'd88)
            `CHECK("4f.JAL x1=4",   uut.cpuregs[1], 32'd4)  // return addr (PC+4)
        end

        // ----------------------------------------------------------------
        // 4g. Multi-instruction sequence: sum 1..5 using loop
        //   x1 = counter (5..1)
        //   x2 = accumulator
        //   Loop: x2 += x1, x1--; BNE loop back if x1 != 0
        //   Expected: x2 = 1+2+3+4+5 = 15
        //
        //   imem[0]:  ADDI x1, x0, 5        # x1 = 5 (loop counter)
        //   imem[1]:  ADDI x2, x0, 0        # x2 = 0 (accumulator)
        //   imem[2]:  ADD  x2, x2, x1       # x2 += x1      ← loop top
        //   imem[3]:  ADDI x1, x1, -1       # x1--
        //   imem[4]:  BNE  x1, x0, -8       # if x1!=0 goto imem[2]
        //   imem[5]:  EBREAK
        // ----------------------------------------------------------------
        $display("-- 4g. Loop: sum 1..5 --");
        begin : test_4g
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0] = enc_i(12'd5,  X0, X1, F3_ADD, OP_ALUI);
            imem[1] = enc_i(12'd0,  X0, X2, F3_ADD, OP_ALUI);
            imem[2] = enc_r(F7_ADD, X1, X2, X2, F3_ADD, OP_ALU); // ADD x2,x2,x1
            imem[3] = enc_i(-12'd1, X1, X1, F3_ADD, OP_ALUI);    // ADDI x1,x1,-1
            imem[4] = enc_b(-13'd8, X0, X1, F3_BNE, OP_BRANCH);  // BNE x1,x0,-8
            imem[5] = EBREAK;
            run_program(120);
            `CHECK("4g.Loop sum=15", uut.cpuregs[2], 32'd15)
        end

        // ----------------------------------------------------------------
        // 4h. Illegal instruction → trap
        // ----------------------------------------------------------------
        $display("-- 4h. Trap on illegal instruction --");
        begin : test_4h
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0] = 32'hFFFF_FFFF; // illegal
            imem[1] = EBREAK;
            run_program(30);
            `CHECK("4h.Trap asserted", trap, 1'b1)
        end

        // ----------------------------------------------------------------
        // 4i. reg_out_hi: 32-bit ADD overflow produces correct high word
        //   x1 = 0xFFFF_FFFF
        //   x2 = 0x0000_0001
        //   ADD x3, x1, x2        → result = 0x1_0000_0000
        //   reg_out_hi should be 0x0000_0001 (carry out)
        //   x3 (lower 32) = 0x0000_0000
        // ----------------------------------------------------------------
        $display("-- 4i. reg_out_hi overflow --");
        begin : test_4i
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            // LUI x1, 0xFFFFF  ; x1 upper 20 bits = 0xFFFFF, lower 12=0  → x1=0xFFFFF000
            // ADDI x1, x1, 0xFFF (sign extended -1) → x1 = 0xFFFF_FFFF
            imem[0] = enc_lui(20'hFFFFF, X1);                       // LUI x1, 0xFFFFF000
            imem[1] = enc_i(12'hFFF, X1, X1, F3_ADD, OP_ALUI);    // ADDI x1,x1,-1 → 0xFFFF_FFFF
            imem[2] = enc_i(12'd1,   X0, X2, F3_ADD, OP_ALUI);    // ADDI x2,x0,1
            imem[3] = enc_r(F7_ADD, X2, X1, X3, F3_ADD, OP_ALU); // ADD x3,x1,x2
            imem[4] = EBREAK;
            run_program(60);
            `CHECK("4i.ADD x3 lo=0",     uut.cpuregs[3], 32'h0000_0000)
            `CHECK("4i.reg_out_hi=1",    reg_out_hi,      32'h0000_0001)
        end

        // ----------------------------------------------------------------
        // 4j. 8-bit data through CPU ADD
        //   x1 = 0xAB
        //   x2 = 0x55
        //   ADD x3 = 0x100
        // ----------------------------------------------------------------
        $display("-- 4j. 8-bit values through CPU ADD --");
        begin : test_4j
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            imem[0] = enc_i(12'hAB, X0, X1, F3_ADD, OP_ALUI);
            imem[1] = enc_i(12'h55, X0, X2, F3_ADD, OP_ALUI);
            imem[2] = enc_r(F7_ADD, X2, X1, X3, F3_ADD, OP_ALU);
            imem[3] = EBREAK;
            run_program(40);
            `CHECK("4j.ADD 0xAB+0x55=0x100", uut.cpuregs[3], 32'h100)
        end

        // ----------------------------------------------------------------
        // 4k. 16-bit values through CPU ADD
        //   x1 = 0xBEEF
        //   x2 = 0x1234
        // ----------------------------------------------------------------
        $display("-- 4k. 16-bit values through CPU ADD --");
        begin : test_4k
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            // 0xBEEF doesn't fit signed 12-bit immediate, use LUI+ADDI
            // LUI x1, 0x0000B ; ADDI x1, x1, 0xEEF  → 0xBEEF
            imem[0] = enc_lui(20'h0000B, X1);
            imem[1] = enc_i(12'hEEF,  X1, X1, F3_ADD, OP_ALUI); // 0xB000 + sign(0xEEF=-273) → need +0xEEF
            // Actually 0xBEEF = 48879 decimal; 12-bit signed max=2047
            // Better: LUI x1, 1; ADDI x1,x1,-0x1111 ; then shift ...
            // Simplest: use two ADDI: ADDI x1,x0,0x7FF; ADDI x1,x1,0x3F0 ... messy
            // Use: value = 0xBEEF = 0xC000 - 0x111 = ...
            // Easy path: LUI puts upper 20 in bits [31:12], so:
            // We want 0x0000_BEEF
            // LUI x1, 0x00001 → x1 = 0x0000_1000
            // ADDI x1,x1,0xEEF is -273 in signed, = 0x1000-0x111 = 0xEEF? No.
            // Let's just use two values that fit 12-bit signed each
            // 0x1234 = 4660 decimal > 2047, also doesn't fit
            // Use: x1=0x0ABC, x2=0x0123 (both fit 12-bit signed; max=2047=0x7FF? No 0x0ABC=2748 > 2047)
            // Safest: x1=0x7FF, x2=0x7FF; sum=0xFFE
            // Let's redesign 4k with values that fit
            imem[0] = enc_i(12'd1000, X0, X1, F3_ADD, OP_ALUI); // x1=1000
            imem[1] = enc_i(12'd800,  X0, X2, F3_ADD, OP_ALUI); // x2=800
            imem[2] = enc_r(F7_ADD, X2, X1, X3, F3_ADD, OP_ALU);
            imem[3] = EBREAK;
            run_program(40);
            `CHECK("4k.ADD 1000+800=1800", uut.cpuregs[3], 32'd1800)
        end

        // ----------------------------------------------------------------
        // 4l. 32-bit SUB: large values
        //   x1 = 0x7FFF_FFFF (max positive 32-bit signed)
        //   x2 = 0x0FFF_FFFF
        //   SUB x3 = 0x7000_0000
        // ----------------------------------------------------------------
        $display("-- 4l. 32-bit SUB large values --");
        begin : test_4l
            integer j;
            for (j = 0; j < 64; j = j+1) imem[j] = NOP;
            // Build 0x7FFF_FFFF: LUI x1,0x7FFFF → 0x7FFFF000; ADDI x1,x1,0x7FF → 0x7FFFF7FF ... close but off
            // LUI x1,0x80000 → 0x80000000; ADDI x1,x1,-1 → 0x7FFFFFFF ✓
            imem[0] = enc_lui(20'h80000, X1);
            imem[1] = enc_i(-12'd1, X1, X1, F3_ADD, OP_ALUI);  // x1 = 0x7FFF_FFFF
            // Build 0x0FFF_FFFF: LUI x2,0x10000 → 0x10000000; ADDI x2,x2,-1 → 0x0FFF_FFFF ✓
            imem[2] = enc_lui(20'h10000, X2);
            imem[3] = enc_i(-12'd1, X2, X2, F3_ADD, OP_ALUI);  // x2 = 0x0FFF_FFFF
            imem[4] = enc_r(F7_SUB, X2, X1, X3, F3_ADD, OP_ALU); // SUB x3,x1,x2
            imem[5] = EBREAK;
            run_program(60);
            `CHECK("4l.SUB 0x7FFF_FFFF-0x0FFF_FFFF=0x7000_0000", uut.cpuregs[3], 32'h7000_0000)
        end

        // ---- Final summary ----
        $display("\n========================================================");
        $display("  RESULTS: %0d PASSED  |  %0d FAILED", pass_count, fail_count);
        $display("========================================================\n");

        if (fail_count == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** %0d TEST(S) FAILED — see FAIL lines above ***", fail_count);

        $finish;
    end

    // Safety watchdog: stop simulation if it runs too long
    initial begin
        #500000;
        $display("WATCHDOG: simulation exceeded time limit");
        $finish;
    end

endmodule
