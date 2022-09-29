`timescale 1ns/1ns
module DataMem(
        input [31:0] address, write_data, 
        input mem_read, mem_write, clk, 
        output reg [31:0] read_data
    );
    
    reg [31:0] mem_data [0:511];

    initial begin
        $readmemb("memory.data", mem_data);
    end

    always @(posedge clk) begin
        if (mem_write) begin
            mem_data[address/4] <=  write_data;
        end
    end
  
    assign read_data = mem_read ? mem_data[address/4] : read_data;
endmodule