`timescale 1ns/1ns
module Shift2(input [31:0] in, output reg [31:0] w);
    assign w = {in[29:0], 2'b0};
endmodule