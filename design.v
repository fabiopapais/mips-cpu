module ControlUnit (
    input [5:0] opcode,
    input [5:0] funct,
    output reg RegDst,
    output reg ALUSrc,
    output reg MemtoReg,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg Branch,
    output reg [2:0] ALUControl,
    output reg Jump,
    output reg JAL // Add JAL signal
);
    always @(*) begin
        case (opcode)
            6'b000000: begin // R-type instructions
                RegDst = 1;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                Jump = 0;
                JAL = 0;
                case (funct)
                    6'b100000: ALUControl = 3'b010; // add
                    6'b100010: ALUControl = 3'b110; // sub
                    // Add more R-type instructions here
                    default: ALUControl = 3'b000; // NOP
                endcase
            end
            6'b100011: begin // lw
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 1;
                RegWrite = 1;
                MemRead = 1;
                MemWrite = 0;
                Branch = 0;
                Jump = 0;
                JAL = 0;
                ALUControl = 3'b010; // add
            end
            6'b101011: begin // sw
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 1;
                Branch = 0;
                Jump = 0;
                JAL = 0;
                ALUControl = 3'b010; // add
            end
            6'b000101: begin // bne
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 1;
                Jump = 0;
                JAL = 0;
                ALUControl = 3'b110; // sub
            end
            6'b001000: begin // addi
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                Jump = 0;
                JAL = 0;
                ALUControl = 3'b010; // add
            end
            6'b000011: begin // jal
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                Jump = 1;
                JAL = 1;
                ALUControl = 3'b000; // No operation needed
            end
            default: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                Jump = 0;
                JAL = 0;
                ALUControl = 3'b000; // NOP
            end
        endcase
    end
endmodule




module ALU (
    input [31:0] op1,
    input [31:0] op2,
    input [2:0] control,
    output reg [31:0] result,
    output reg zero
);
    always @(*) begin
        case (control)
            3'b010: assign result = op1 + op2;  // add
            3'b110: assign result = op1 - op2;  // sub
            3'b111: assign result = (op1 <= op2) ? 1 : 0; // set less than or equal
            // Add more ALU operations here
            default: result = 0;
        endcase
        assign zero = (result == 0);
    end
endmodule


module InstructionMemory (
    input [31:0] address,
    output reg [31:0] instruction
);
    reg [31:0] memory [0:255];

    initial begin
      memory[0] = 32'h20080006; // addi $t0, $zero, 6
      memory[1] = 32'h20090004; // addi $t1, $zero, 4
      memory[2] = 32'h0C000003; // jal add_function
      memory[3] = 32'h01095020; // add_function: add $t2, $t0, $t1
      memory[4] = 32'h03E00008; // add_function: jr $ra
      memory[5] = 32'h200B0000; // addi $t3, $zero, 12
      memory[6] = 32'h15330001; // bne $t2, $t3, skip_add
      memory[7] = 32'h213A0003; // addi $t2, $t2, 3
      memory[8] = 32'h08000008; // skip_add: jr skip_add
    end

    always @(address) begin
        instruction = memory[address >> 2];
    end
endmodule





module DataMemory (
    input clk,
    input [31:0] address,
    input [31:0] write_data,
    input mem_write,
    input mem_read,
    output reg [31:0] read_data
);
    reg [31:0] memory [0:255];

  assign read_data = memory[address];
  
  always @(negedge clk) begin
        if (mem_write)
            memory[address >> 2] <= write_data;
    end

endmodule

module RegisterFile (
    input clk,
    input reg_write,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input [4:0] write_reg,
    input [31:0] write_data,
    output reg [31:0] read_data1,
    output reg [31:0] read_data2
);
    reg [31:0] registers [0:31];

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'd0;
    end

    always @(posedge clk) begin
        if (reg_write)
            registers[write_reg] <= write_data;
    end

    always @(*) begin
        read_data1 = registers[read_reg1];
        read_data2 = registers[read_reg2];
    end
endmodule



module SignExtend (
    input [15:0] in,
    output [31:0] out
);
    assign out = {{16{in[15]}}, in};
endmodule

module MIPS_Processor (
    input clk,
    input reset
);
    wire [31:0] pc, next_pc, pc_plus4, branch_addr, instruction;
    wire [4:0] write_reg;
    wire [31:0] read_data1, read_data2, write_data, alu_result, mem_data, sign_ext_imm, alu_op2;
    wire [2:0] alu_control;
    wire zero, pc_src, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch, reg_dst, jump, jal;

    // PC Register
    reg [31:0] pc_reg;
    assign pc = pc_reg;

    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_reg <= 0;
        else
            pc_reg <= next_pc;
    end

    // ALU Operand Mux
    assign alu_op2 = alu_src ? sign_ext_imm : (jal ? pc_plus4 : read_data2);

    // Register Destination Mux
    assign write_reg = reg_dst ? instruction[15:11] : (jal ? 5'b11111 : instruction[20:16]);

    // Write Data Mux
    assign write_data = mem_to_reg ? mem_data : alu_result;

    // Branch Address Calculation
    assign branch_addr = (sign_ext_imm << 2) + pc_plus4;
  
    // Branch Condition
    assign pc_src = branch & ~zero; // For bne, branch if the result is not zero

    // Next PC Calculation
    assign pc_plus4 = pc + 4;
    assign next_pc = jump ? {pc_plus4[31:28], instruction[25:0], 2'b00} : (pc_src ? branch_addr : pc_plus4);

    // Handle the $ra register update for jal
    always @(posedge clk) begin
        if (reset)
            regfile.registers[31] <= 32'd0;
        else if (jal) begin
            regfile.registers[31] <= pc_plus4; // Save return address in $ra
        end
    end

    // Instruction Memory
    InstructionMemory imem (
        .address(pc),
        .instruction(instruction)
    );

    // Control Unit
    ControlUnit control (
        .opcode(instruction[31:26]),
        .funct(instruction[5:0]),
        .RegDst(reg_dst),
        .ALUSrc(alu_src),
        .MemtoReg(mem_to_reg),
        .RegWrite(reg_write),
        .MemRead(mem_read),
        .MemWrite(mem_write),
        .Branch(branch),
        .ALUControl(alu_control),
        .Jump(jump),
        .JAL(jal)
    );

    // Register File
    RegisterFile regfile (
        .clk(clk),
        .reg_write(reg_write),
        .read_reg1(instruction[25:21]),
        .read_reg2(instruction[20:16]),
        .write_reg(write_reg),
        .write_data(write_data),
        .read_data1(read_data1),
        .read_data2(read_data2)
    );

    // ALU
    ALU alu (
        .op1(read_data1),
        .op2(alu_op2),
        .control(alu_control),
        .result(alu_result),
        .zero(zero)
    );

    // Data Memory
    DataMemory dmem (
        .clk(clk),
        .address(alu_result),
        .write_data(read_data2),
        .mem_write(mem_write),
        .mem_read(mem_read),
        .read_data(mem_data)
    );

    // Sign Extension
    SignExtend sign_ext (
        .in(instruction[15:0]),
        .out(sign_ext_imm)
    );

    // Monitor Register Values (for debugging purposes)
    always @(posedge clk) begin
        $display("Time: %0t | PC: %0d | Instr: %h | RegWrite: %b | ALURes: %d | Zero: %b | MemData: %d", 
                 $time, pc, instruction, reg_write, alu_result, zero, mem_data);
    end
endmodule







