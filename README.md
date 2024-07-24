# Components to Implement

- **Instruction Memory**: A memory module that stores the instructions of your program.
- **Data Memory**: A memory module for storing and retrieving data.
- **Program Counter (PC)**: Keeps track of the address of the next instruction to execute.
- **Instruction Fetch Unit**: Fetches the instruction from the instruction memory.
- **Instruction Decode Unit**: Decodes the fetched instruction and identifies the operation and operands.
- **Register File**: A set of registers that can be read from and written to.
- **Arithmetic Logic Unit (ALU)**: Performs arithmetic and logical operations.
- **Control Unit**: Generates control signals based on the decoded instruction.
- **ALU Control**: Determines the operation the ALU should perform based on the instruction.
- **Multiplexers (MUXes)**: Used to select between different inputs for the next operation.
- **Sign Extension Unit**: Extends the immediate values to the appropriate length.

# Example High-Level Architecture

Here's a simplified architecture for a MIPS processor:

- **Instruction Memory**: Stores the machine code of the program.
- **PC (Program Counter)**: Points to the address of the next instruction.
- **Instruction Fetch**: Fetches the instruction from the memory.
- **Instruction Decode**: Decodes the instruction to understand what needs to be done.
- **Register File**: Contains the registers, allows read/write access to the registers.
- **ALU**: Performs arithmetic and logic operations.
- **Data Memory**: Used for load/store operations.
- **Control Unit**: Directs the operation of the processor.
