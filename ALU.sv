`timescale 1ns/1ns
module ALU(input signed [31:0] a, b, input [2:0] alu_op, output reg [31:0] w, output zero);
    parameter [2:0] AND = 3'b000;
    parameter [2:0] OR = 3'b001;
    parameter [2:0] ADD = 3'b010;
    parameter [2:0] SUB = 3'b110;
	parameter [2:0] SLT = 3'b111; 
	  
	always @(alu_op, a, b) begin
		case(alu_op) 
	    	AND : w <= a & b;
	    	OR : w <= a | b;
	    	ADD : w <= a + b;
	    	SUB : w <= a - b;
	    	SLT : w <= a < b ? {31'b0, 1'b1} : 32'b0;
	  	endcase
	end
	     
	assign zero = (w == 32'b0) ? 1'b1 : 1'b0;
endmodule