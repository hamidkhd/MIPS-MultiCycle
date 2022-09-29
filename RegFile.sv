`timescale 1ns/1ns
module RegFile(
        input [4:0] read_reg1_address, read_reg2_address, write_reg_address,
        input [31:0] write_data,
        input clk, reg_write, 
        output reg [31:0] read_data1, read_data2
    );

    reg [31:0] reg_memory [0:31] ;

	initial begin
		for(int i = 0; i < 32; i++)
		    reg_memory[i] = 32'b0;
	end

    always @(posedge clk) begin
        if (reg_write) begin  
            reg_memory[write_reg_address] <= write_data;   
        end  
    end 
  
    assign read_data1 = reg_memory[read_reg1_address];  
    assign read_data2 = reg_memory[read_reg2_address];
	always  @(reg_memory[0]) reg_memory[0] = 32'b0;  
endmodule