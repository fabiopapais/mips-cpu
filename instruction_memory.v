module inst_mem(
    input [31:0] addr,
    output reg [31:0] instr
);
reg [31:0] memory[0:255]; // 256 words of memory

initial begin
    // Load instructions here
    // Example: memory[0] = 32'hxxxxxxxx;
end

always @(*) begin
    instr = memory[addr[31:2]]; // Fetch instruction
end
endmodule
