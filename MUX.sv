`timescale 1ns/1ns
module MUX2(input [31:0] a, b, input sel, output [31:0] w);
    assign w = sel ? b : a;  
endmodule

`timescale 1ns/1ns
module MUX3_5(input [4:0] a, b, c, input [1:0] sel, output reg [4:0] w);
    parameter [1:0] A = 2'b00;
    parameter [1:0] B = 2'b01;
    parameter [1:0] C = 2'b10;

    always @(sel,a,b,c) begin
        case(sel)
            A : w = a;
            B : w = b;
            C : w = c;
        endcase
    end
endmodule

`timescale 1ns/1ns
module MUX3_32(input [31:0] a, b, c, input [1:0] sel, output reg [31:0] w);
    parameter [1:0] A = 2'b00;
    parameter [1:0] B = 2'b01;
    parameter [1:0] C = 2'b10;

    always @(sel,a,b,c) begin
        case(sel)
            A : w = a;
            B : w = b;
            C : w = c;
        endcase
    end
endmodule

`timescale 1ns/1ns
module MUX4(input [31:0] a, b, c, d, input [1:0] sel, output reg [31:0] w);
    parameter [1:0] A = 2'b00;
    parameter [1:0] B = 2'b01;
    parameter [1:0] C = 2'b10;
    parameter [1:0] D = 2'b11;
    
    always @(sel,a,b,c,d) begin
        case(sel)
            A : w = a;
            B : w = b;
            C : w = c;
            D : w = d;
        endcase
    end
endmodule