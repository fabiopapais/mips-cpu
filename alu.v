module alu(
    input [31:0] a,
    input [31:0] b,
    input [3:0] alu_control,
    output reg [31:0] result,
    output zero
);

assign zero = (result == 0);

always @(*) begin
    case (alu_control)
        4'b0010: result = a + b; // Add
        4'b0110: result = a - b; // Subtract
        4'b0000: result = a & b; // AND
        4'b0001: result = a | b; // OR
        default: result = 0;
    endcase
end
endmodule
