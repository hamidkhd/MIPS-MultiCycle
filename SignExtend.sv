`timescale 1ns/1ns
module SE(input [15:0] in, output reg [31:0] w); 
    assign w = {{16{in[15]}}, in[15:0]};
endmodule