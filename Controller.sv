`timescale 1ns/1ns
module Controller(
        input [5:0] opcode, func, input zero, clk, rst,
        output reg mem_read, mem_write, pc_write_input, reg_write,
            IorD, IR_write, ALU_srcA,
        output reg [1:0] reg_dst, pc_src, mem_to_reg, ALU_srcB,
        output reg [2:0] alu_op
    );

    parameter [5:0] R_TYPE = 6'b000000;
    parameter [5:0] ANDI = 6'b001100;
    parameter [5:0] ADDI = 6'b001000;
    parameter [5:0] LW = 6'b100011;
    parameter [5:0] SW = 6'b101011;
    parameter [5:0] BEQ = 6'b000100;
    parameter [5:0] BNE = 6'b000101;
    parameter [5:0] J = 6'b000010;
    parameter [5:0] JAL = 6'b000011;
    parameter [5:0] JR = 6'b000001;

    parameter [3:0] IF = 4'b0000;
    parameter [3:0] ID = 4'b0001;
    parameter [3:0] MEM_AD_COMPL = 4'b0010;
    parameter [3:0] LW_MEM_ACC = 4'b0011;
    parameter [3:0] MEM_READ_COMPL = 4'b0100;
    parameter [3:0] SW_MEM_ACC = 4'b0101;
    parameter [3:0] R_EXEC = 4'b0110;
    parameter [3:0] R_COMPL = 4'b0111;
    parameter [3:0] BEQ_COMPL = 4'b1000;
    parameter [3:0] J_COMPL = 4'b1001;
    parameter [3:0] BNE_COMPL = 4'b1010;
    parameter [3:0] ADDI_EXEC = 4'b1011;
    parameter [3:0] AAI_COMPL = 4'b1100;
    parameter [3:0] ANDI_EXEC = 4'b1101;
    parameter [3:0] JR_COMPL = 4'b1110;
    parameter [3:0] JAL_COMPL = 4'b1111;

    logic [3:0] ps, ns;
    reg [1:0] alu_case;
    reg pc_write, pc_wc_beq, pc_wc_bne;

    always @(ps) begin
        case(ps)
            IF : ns = ID;
            ID : begin
                    case(opcode)
                        R_TYPE : ns = R_EXEC;
                        LW,SW : ns = MEM_AD_COMPL;
                        BEQ : ns = BEQ_COMPL;
                        J : ns = J_COMPL;
                        BNE : ns = BNE_COMPL;
                        ADDI : ns = ADDI_EXEC;
                        ANDI : ns = ANDI_EXEC;
                        JR : ns = JR_COMPL;
                        JAL : ns = JAL_COMPL;
                    endcase
                end
            MEM_AD_COMPL : ns = opcode == SW ? SW_MEM_ACC : LW_MEM_ACC;
            LW_MEM_ACC : ns = MEM_READ_COMPL;
            MEM_READ_COMPL : ns = IF;
            SW_MEM_ACC : ns = IF;
            R_EXEC : ns = R_COMPL;
            R_COMPL : ns = IF;
            BEQ_COMPL : ns = IF;
            J_COMPL : ns = IF;
            BNE_COMPL : ns = IF;
            ADDI_EXEC : ns = AAI_COMPL;
            AAI_COMPL : ns = IF;
            ANDI_EXEC : ns = AAI_COMPL;
            JR_COMPL : ns = IF;
            JAL_COMPL : ns = IF;
        endcase
    end

    always @(ps) begin
        {
            mem_read, mem_write, ALU_srcA, ALU_srcB, reg_write, pc_write, IorD, IR_write,
            alu_case, pc_wc_beq, pc_wc_bne, reg_dst, mem_to_reg, pc_src, pc_write_input
        } = 18'b0;

        case (ps)
            IF : begin
                    {mem_read, IR_write, pc_write} = 3'b111;
                    IorD = 1'b0;
                    ALU_srcB = 2'b01;
                    ALU_srcA = 1'b0; 
                    alu_case = 2'b00; 
                    pc_src = 2'b00; 
                end
            ID : begin
                    ALU_srcB = 2'b11;
                    ALU_srcA = 1'b0; 
                    alu_case = 2'b00; 
                end
            MEM_AD_COMPL : begin
                    ALU_srcA = 1'b1;
                    ALU_srcB = 2'b10;
                    alu_case = 2'b00; 
                end
            LW_MEM_ACC : {mem_read, IorD} = 2'b11;
            MEM_READ_COMPL : {reg_write, reg_dst, mem_to_reg} = 5'b1_00_01;
            SW_MEM_ACC : {mem_write, IorD} = 2'b11;
            R_EXEC : {ALU_srcA, ALU_srcB, alu_case} = 5'b1_00_10;
            R_COMPL : {reg_write, reg_dst, mem_to_reg} = 5'b1_01_00;
            BEQ_COMPL : begin
                    // if (zero) pc_write = 1'b1;
                    pc_wc_beq = 1'b1;
                    ALU_srcA = 1'b1;
                    ALU_srcB = 2'b00;
                    alu_case = 2'b01;
                    pc_src = 2'b10;
                end
            J_COMPL : {pc_write, pc_src} = 3'b1_01;
            BNE_COMPL : begin
                    // if (!zero) pc_write = 1'b1;
                    pc_wc_bne = 1'b1;
                    ALU_srcA = 1'b1;
                    ALU_srcB = 2'b00;
                    alu_case = 2'b01;
                    pc_src = 2'b10;
                end
            ADDI_EXEC : {ALU_srcA, alu_case, ALU_srcB} = 5'b1_00_10;
            AAI_COMPL : {reg_write, reg_dst, mem_to_reg} = 5'b1_00_00;
            ANDI_EXEC : {ALU_srcA, alu_case, ALU_srcB} = 5'b1_11_10;
            JR_COMPL : {pc_src, pc_write} = 3'b11_1;
            JAL_COMPL : {reg_write, pc_write, pc_src, mem_to_reg, reg_dst} = 8'b1_1_01_10_10;
        endcase
    end

    always @(posedge clk) begin
        if(rst)
            ps = 4'b0;
        else
            ps <= ns;
    end

    ALU_Controller alu_controller(func, alu_case, alu_op);
    always @(pc_write, zero, pc_wc_beq, pc_wc_bne) begin
        pc_write_input = pc_write || (pc_wc_beq && zero) || (pc_wc_bne && !zero);
    end
endmodule                