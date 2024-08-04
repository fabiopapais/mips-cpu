module MIPS_Processor_tb;
    reg clk;
    reg reset;
    MIPS_Processor uut (
        .clk(clk),
        .reset(reset)
    );

    initial begin
        clk = 0;
        reset = 1;
        #10;
        reset = 0;
    end

    always #5 clk = ~clk;

    initial begin
        $dumpfile("MIPS_Processor_tb.vcd");
        $dumpvars(0, MIPS_Processor_tb);
        #200;
        $finish;
    end
  
  always @ (*) begin
    integer i;
    
    for (i = 8; i < 12; i++) begin
      $display("Registrador %d, valor %d.", i, uut.regfile.registers[i]);
    end
    
  end
  
  
endmodule
