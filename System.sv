// 
Authors: Evyatar Shapira, Michael Kochubey, Shay Dahan
=============================================================
// FILE: System.sv (1000-ITERATION LOOP VERSION)
// =============================================================
`timescale 1ns/1ps

// -------------------------------------------------------------
// 1. TOP MODULE
// -------------------------------------------------------------
module Top(
    input  logic        reset_n,
    input  logic        clock,
    output logic [15:0] test_value 
);
    logic [31:0] pc, instr, read_data, write_data, alu_result;
    logic        mem_write;

    mips mips_inst(
        .clock(clock), .reset_n(reset_n), .pc(pc), .instr(instr),
        .mem_write(mem_write), .alu_result(alu_result),
        .write_data(write_data), .read_data(read_data)
    );

    InstrMem imem(.pc(pc), .instr(instr));

    DataMem dmem(
        .clock(clock), .reset_n(reset_n), .write_enable(mem_write), 
        .address(alu_result), .write_data(write_data), .read_data(read_data),
        .test_value(test_value)
    );
endmodule

// -------------------------------------------------------------
// 2. MIPS CPU
// -------------------------------------------------------------
module mips (
    input logic clock, reset_n,
    output logic [31:0] pc, alu_result, write_data,
    input logic [31:0] instr, read_data,
    output logic mem_write
);
    logic mem_to_reg, alu_src, reg_dest, reg_write, jump, pc_src, zero;
    logic [2:0] alu_control;
    logic loop_init_w;

    ControlUnit cu (
        .opcode(instr[31:26]), .funct(instr[5:0]), .zero_flag(zero),
        .mem_to_reg(mem_to_reg), .mem_write(mem_write), .pc_src(pc_src),
        .alu_src(alu_src), .reg_dest(reg_dest), .reg_write(reg_write),
        .jump(jump), .alu_control(alu_control), .loop_init(loop_init_w)
    );

    DataPath dp (
        .clock(clock), .reset_n(reset_n), .instruction(instr[25:0]), 
        .opcode(instr[31:26]),
        .read_data(read_data), .alu_control(alu_control), .pc_src(pc_src),
        .mem_to_reg(mem_to_reg), .alu_src(alu_src), .reg_dest(reg_dest),
        .reg_write(reg_write), .jump(jump), .zero_flag(zero),
        .pc(pc), .alu_out(alu_result), .write_data(write_data),
        .loop_init(loop_init_w)
    );
endmodule

// -------------------------------------------------------------
// 3. DATAPATH (With Robust PC Reset)
// -------------------------------------------------------------
module DataPath(
    input logic        reset_n, clock,
    input logic [25:0] instruction,
    input logic [5:0]  opcode,
    input logic [31:0] read_data,
    input logic [2:0]  alu_control,
    input logic        pc_src, mem_to_reg, alu_src, reg_dest, reg_write, jump,
    input logic        loop_init, 
    output logic       zero_flag,
    output logic [31:0] pc, alu_out, write_data
);
    logic [4:0]  write_reg_w;
    logic [31:0] src_a_w, src_b_w, src_c_w, result_w, rd2_w, sign_imm_w, imm_w;
    logic [31:0] pc_current, pc_next, pc_plus4, pc_branch, pc_final;

    // Hardware Loop Registers
    logic [31:0] loop_count, loop_start, loop_end;
    logic        loop_active;

    // --- PC LOGIC ---
    assign pc_plus4 = pc_current + 4;
    assign pc_branch = pc_plus4 + (sign_imm_w << 2);
    assign loop_active = (pc_current == loop_end) && (loop_count > 1);
    
    assign pc_final = loop_active ? loop_start : 
                      (jump ? {pc_plus4[31:28], instruction, 2'b00} : 
                      (pc_src ? pc_branch : pc_plus4));

    // PC Register with Synchronous Reset (Forces 0)
    always_ff @(posedge clock or negedge reset_n) begin
        if (!reset_n) pc_current <= 32'b0;
        else          pc_current <= pc_final;
    end
    assign pc = pc_current;

    // Loop Logic
    always_ff @(posedge clock or negedge reset_n) begin
        if (!reset_n) begin
            loop_count <= 0; loop_start <= 0; loop_end <= 0;
        end
        else if (loop_init) begin
            loop_count <= src_a_w; 
            loop_start <= pc_plus4; 
            loop_end   <= pc_branch; 
        end
        else if (loop_active) begin
            loop_count <= loop_count - 1; 
        end
    end

    // Sub-components
    assign write_reg_w = reg_dest ? instruction[15:11] : instruction[20:16];
    
    RegFile rf (
        .clock(clock), .reset_n(reset_n), .write_enable(reg_write),
        .addr1(instruction[25:21]), .addr2(instruction[20:16]), .addr3(write_reg_w),
        .write_data(result_w), .rd1(src_a_w), .rd2(rd2_w), .rd3(src_c_w)
    );
    assign write_data = rd2_w;

    // Immediate value handling
    assign sign_imm_w = {{16{instruction[15]}}, instruction[15:0]}; // Sign-extend (for ADDI, BEQ, etc.)
    
    // Select immediate based on opcode
    always_comb begin
        if (opcode == 6'b001111)      // LUI: Load upper immediate (shift left 16)
            imm_w = {instruction[15:0], 16'b0};
        else if (opcode == 6'b001101) // ORI: Zero-extend immediate
            imm_w = {16'b0, instruction[15:0]};
        else                          // Default: Sign-extend (ADDI, etc.)
            imm_w = sign_imm_w;
    end
    
    assign src_b_w = alu_src ? imm_w : rd2_w;

    ALU alu_inst (
        .src_a(src_a_w), .src_b(src_b_w), .src_c(src_c_w),
        .alu_control(alu_control), .alu_result(alu_out), .zero_flag(zero_flag)
    );

    assign result_w = mem_to_reg ? read_data : alu_out;
endmodule

// -------------------------------------------------------------
// 4. CONTROL UNIT
// -------------------------------------------------------------
module ControlUnit (
    input  logic [5:0] opcode, funct,
    input  logic       zero_flag,
    output logic       mem_to_reg, mem_write, pc_src, alu_src, 
    output logic       reg_dest, reg_write, jump, loop_init,
    output logic [2:0] alu_control
);
    logic branch;
    logic [1:0] alu_op;

    always_comb begin
        {reg_write, reg_dest, alu_src, branch, mem_write, mem_to_reg, alu_op, jump, loop_init} = 0;
        case (opcode)
            6'b000000: {reg_write, reg_dest, alu_op} = 4'b1110; // R-Type
            6'b001000: {reg_write, alu_src} = 2'b11;            // ADDI
            6'b001101: {reg_write, alu_src, alu_op} = 4'b1100;  // ORI (0x0D)
            6'b001111: {reg_write, alu_src, alu_op} = 4'b1100;  // LUI (0x0F) 
            6'b011100: {reg_write, reg_dest, alu_op} = 4'b1110; // VMAC (Special2)
            6'b111111: loop_init = 1;                           // LOOP (FC)
            6'b000100: {branch, alu_op} = 3'b101;               // BEQ
            6'b000010: jump = 1;                                // JUMP
        endcase
    end

    assign pc_src = branch & zero_flag;

    always_comb begin
        alu_control = 3'b000;
        if (alu_op == 2'b00) alu_control = 3'b010; // ADD (for LW/SW/ADDI)
        else if (alu_op == 2'b01) alu_control = 3'b110; // SUB (for BEQ)
        else if (opcode == 6'b011100) alu_control = 3'b100; // VMAC
        else if (opcode == 6'b001101) alu_control = 3'b001; // ORI
        else if (opcode == 6'b001111) alu_control = 3'b101; // LUI
        else case(funct)
            6'b100000: alu_control = 3'b010; // ADD
            6'b100101: alu_control = 3'b001; // OR
            default:   alu_control = 3'b000;
        endcase
    end
endmodule

// -------------------------------------------------------------
// 5. REGFILE
// -------------------------------------------------------------
module RegFile (
    input logic clock, reset_n, write_enable,
    input logic [4:0] addr1, addr2, addr3,
    input logic [31:0] write_data,
    output logic [31:0] rd1, rd2, rd3
);
    logic [31:0] registers [31:0];

    always_ff @(posedge clock or negedge reset_n) begin
        if (!reset_n) begin
            for (int i=0; i<32; i++) registers[i] <= 0;
        end else if (write_enable && addr3 != 0) begin
            registers[addr3] <= write_data;
        end
    end

    assign rd1 = (addr1 != 0) ? registers[addr1] : 0;
    assign rd2 = (addr2 != 0) ? registers[addr2] : 0;
    assign rd3 = (addr3 != 0) ? registers[addr3] : 0;
endmodule

// -------------------------------------------------------------
// 6. ALU
// -------------------------------------------------------------
module ALU (
    input logic [31:0] src_a, src_b, src_c,
    input logic [2:0] alu_control,
    output logic [31:0] alu_result,
    output logic zero_flag
);
    always_comb begin
        case (alu_control)
            3'b010: alu_result = src_a + src_b; // ADD
            3'b110: alu_result = src_a - src_b; // SUB
            3'b001: alu_result = src_a | src_b; // OR / ORI
            3'b101: alu_result = src_b;         // LUI (pass through immediate)
            3'b100: alu_result = $signed(src_c) + ($signed(src_a[31:16]) * $signed(src_b[31:16])) + ($signed(src_a[15:0]) * $signed(src_b[15:0])); // VMAC
            default: alu_result = 0;
        endcase
    end
    assign zero_flag = (alu_result == 0);
endmodule

// -------------------------------------------------------------
// 7. INSTRUCTION MEMORY (1000-ITERATION PROGRAM)
// -------------------------------------------------------------
module InstrMem (input logic [31:0] pc, output logic [31:0] instr);
    
    always_comb begin
        case (pc[31:2]) // Divide PC by 4 to get index
            // --- 1000-LOOP TEST PROGRAM ---
            // Target calculation per iteration: (18 × -38) + (44 × 5) = -684 + 220 = -464
            // After 1000 iterations: -464 × 1000 = -464,000
            //
            // To pack into 32-bit registers:
            // $2 = 0x0012002C = [31:16]=18, [15:0]=44
            // $3 = 0xFFDA0005 = [31:16]=-38 (0xFFDA), [15:0]=5
            
            // Load $2 with upper half = 18
            0: instr = 32'h3C020012;      // LUI $2, 0x0012      ($2 = 0x00120000)
            1: instr = 32'h3442002C;      // ORI $2, $2, 0x002C  ($2 = 0x0012002C) [18, 44]
            
            // Load $3 with upper half = -38 (0xFFDA), lower = 5
            2: instr = 32'h3C03FFDA;      // LUI $3, 0xFFDA      ($3 = 0xFFDA0000)
            3: instr = 32'h34630005;      // ORI $3, $3, 0x0005  ($3 = 0xFFDA0005) [-38, 5]
            
            // Load loop count
            4: instr = 32'h200403E8;      // ADDI $4, $0, 1000   (Count = 1000)
            
            // NOPs for pipeline stabilization
            5: instr = 32'h00000000;      // NOP
            6: instr = 32'h00000000;      // NOP

            // HARDWARE LOOP - 1000 iterations
            7: instr = 32'hFC800000;      // LOOP $4 (opcode=0x3F, rt=$4)
            8: instr = 32'h70430819;      // VMAC $1, $2, $3 (accumulate in $1)
            
            // END - Output result
            10: instr = 32'h00012825;     // OR $5, $0, $1 (move result to $5)
            11: instr = 32'h0800000B;     // JUMP 11 (infinite loop - freeze here)
            
            default: instr = 32'h00000000; // NOP for all other addresses
        endcase
    end
endmodule

// -------------------------------------------------------------
// 8. DATA MEMORY
// -------------------------------------------------------------
module DataMem (
    input logic clock, reset_n, write_enable,
    input logic [31:0] address, write_data,
    output logic [31:0] read_data,
    output logic [15:0] test_value
);
    logic [31:0] mem [255:0];
    always_ff @(posedge clock) if (write_enable) mem[address[31:2]] <= write_data;
    assign read_data = mem[address[31:2]];
    assign test_value = mem[0][15:0];
endmodule

// -------------------------------------------------------------
// 9. TESTBENCH (Extended for 1000 iterations)
// -------------------------------------------------------------
module Top_tb;
    reg reset_n, clock;
    wire [15:0] test;
    Top dut(.reset_n(reset_n), .clock(clock), .test_value(test));

    initial begin
        clock = 0; forever #5 clock = ~clock;
    end

    initial begin
        $display("========================================");
        $display("Starting 1000-iteration hardware loop test");
        $display("Calculation: (18 × -38) + (44 × 5) = -684 + 220 = -464 per iteration");
        $display("Expected result: -464 × 1000 = -464,000 (0xFFF8EE00)");
        $display("========================================");
        reset_n = 0; 
        #50;                    // Hard Reset
        reset_n = 1; 
        #15000;                 // Run for 1000 loops (need more cycles for LUI/ORI setup)
        $display("Test complete. Final accumulator value should be in register $5");
        $display("Expected: -464000 decimal or 0xFFF8EE00 hex");
        $stop;
    end
    
    // Monitor register file to see accumulation
    initial begin
        $monitor("Time=%0t PC=%h Instr=%h", $time, dut.mips_inst.pc, dut.mips_inst.instr);
    end
endmodule
