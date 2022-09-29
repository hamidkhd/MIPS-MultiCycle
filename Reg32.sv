`timescale 1ns/1ns
module Reg32(input [31:0] data, input write, clk, rst, output reg [31:0] out);
    always @(posedge clk) begin
        if (rst)
            out <= 32'b0;
        else if (write)
            out <= data;
    end
endmodule