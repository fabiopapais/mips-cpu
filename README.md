# MIPS Single Cycle Processor

This repository contains the Verilog code for a MIPS single cycle processor. The project includes the following modules:

- **Control Unit**: Decodes the opcode and function fields of the instruction and generates control signals for the processor.
- **ALU**: Performs arithmetic and logical operations based on the control signal.
- **Instruction Memory**: Stores the instructions to be executed by the processor.
- **Data Memory**: Stores and retrieves data from memory.
- **Register File**: Stores and retrieves register values.
- **Sign Extend**: Extends the sign of a 16-bit immediate to 32 bits.

## Instructions Supported

The processor is capable of executing the following instructions:
- `add`: Add two registers.
- `sub`: Subtract two registers.
- `addi`: Add immediate to a register.
- `lw`: Load word from memory.
- `sw`: Store word to memory.
- `bne`: Branch if not equal.
- `jal`: Jump and link.
- `jr`: Jump register.

## Sample Assembly Code

```assembly
li $t0, 5          # $t0 = 5
li $t1, 3          # $t1 = 3
add $t2, $t0, $t1  # $t2 = $t0 + $t1 = 5 + 3 = 8
li $t3, 10         # $t3 = 10
ble $t2, $t3, end  # if $t2 <= $t3, jump to end
addi $t2, $t2, 3   # $t2 > $t3, so add 3: $t2 = $t2 + 3
end:
