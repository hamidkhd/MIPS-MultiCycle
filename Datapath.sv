`timescale 1ns/1ns
module DataPath(
        input [1:0] reg_dst, pc_src, mem_to_reg, ALU_srcB,
        input clk, rst, reg_write, mem_read, mem_write, pc_write_input, IorD, IR_write, ALU_srcA, 
        input [2:0] alu_op,
        output reg [5:0] opcode, func, output reg zero
    );
    
    reg [31:0] pc_address, next_pc, read_data1, read_data2, sign_out, alu_a_in,
        alu_b_in, alu_out, shift_out, mem_data_out, A_out, B_out, IR_out, reg_write_data,
        alu_reg_out, mem_address, MDR_out;              
    reg [4:0] write_address;
        
    Reg32 pc(next_pc, pc_write_input, clk, rst, pc_address); 
    MUX2 IorD_mux(pc_address, alu_reg_out, IorD, mem_address); 
    DataMem datamem(mem_address, B_out, mem_read, mem_write, clk, mem_data_out); 
    Reg32 IR(mem_data_out, IR_write, clk, rst, IR_out); 
    Reg32 MDR(mem_data_out, 1'b1, clk, rst, MDR_out); 
    MUX3_5 reg_write_dst_mux(IR_out[20:16], IR_out[15:11], 5'd31, reg_dst, write_address); 
    MUX3_32 reg_write_data_mux(alu_reg_out, MDR_out, pc_address, mem_to_reg, reg_write_data); 
    RegFile regfile(IR_out[25:21], IR_out[20:16], write_address, reg_write_data, clk, reg_write, read_data1, read_data2);
    Reg32 A(read_data1, 1'b1, clk, rst, A_out);
    Reg32 B(read_data2, 1'b1, clk, rst, B_out);
    MUX2 A_mux(pc_address, A_out, ALU_srcA, alu_a_in);
    MUX4 B_mux(B_out, 32'd4, sign_out, shift_out, ALU_srcB, alu_b_in);
    ALU alu (alu_a_in, alu_b_in, alu_op, alu_out, zero); 
    Reg32 alu_reg(alu_out, 1'b1, clk, rst, alu_reg_out); 
    SE se(IR_out[15:0], sign_out);
    Shift2 shift(sign_out, shift_out);
    MUX4 mux_pc(alu_out, {pc_address[31:28],IR_out[25:0],pc_address[1:0]}, alu_reg_out, A_out, pc_src, next_pc);


    assign opcode = IR_out[31:26];
    assign func = IR_out[5:0];
    
endmodule