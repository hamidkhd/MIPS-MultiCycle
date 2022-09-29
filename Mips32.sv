`timescale 1ns/1ns
module MIPS32(input clk, rst);
    wire [5:0] opcode, func;
    wire zero, mem_read, mem_write, reg_write, pc_write_input, IorD, IR_write, ALU_srcA;
    wire [1:0] reg_dst, pc_src, mem_to_reg, ALU_srcB;
    wire [2:0] alu_op;
    DataPath dp(
        reg_dst, pc_src, mem_to_reg, ALU_srcB, clk, rst, reg_write,
        mem_read, mem_write, pc_write_input, IorD, IR_write, ALU_srcA,
        alu_op, opcode, func, zero
    );
    Controller cn(
        opcode, func, zero, clk, rst, mem_read, mem_write,
        pc_write_input, reg_write, IorD, IR_write, ALU_srcA,reg_dst,
        pc_src, mem_to_reg, ALU_srcB, alu_op
    );
endmodule
