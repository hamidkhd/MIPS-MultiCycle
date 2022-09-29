`timescale 1ns/1ns
module ALU_Controller(input [5:0] func, input [1:0] alu_case, output reg [2:0] alu_op);
    parameter [5:0] AND = 6'b100100;
    parameter [5:0] OR = 6'b100101;
    parameter [5:0] ADD = 6'b100000;
    parameter [5:0] SUB = 6'b100010;
    parameter [5:0] SLT = 6'b101010;

    always @(func, alu_case) begin
        if (alu_case == 2'b10) begin // add, or, add, sub, slt
            case (func) 
                AND : alu_op = 3'b000;
      		    OR : alu_op = 3'b001;
        		ADD : alu_op = 3'b010;
        		SUB : alu_op = 3'b110;
        		SLT : alu_op = 3'b111;
      		endcase
    	end
		   
	    else if (alu_case == 2'b00) begin // lw, sw, addi
	        alu_op = 3'b010;
	    end
		  
	    else if (alu_case == 2'b01) begin // beq, bne
	        alu_op = 3'b110;
	    end

	    else if (alu_case == 2'b11) begin // andi
	        alu_op = 3'b000;
	    end
  	end
endmodule