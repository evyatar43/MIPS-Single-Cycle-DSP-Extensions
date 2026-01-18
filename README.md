

This document outlines the steps to compile and simulate the enhanced MIPS processor (Hardware Loop & VMAC) using ModelSim/Questa.
<img width="1203" height="714" alt="image" src="https://github.com/user-attachments/assets/67f8bad4-ec01-43f4-9a09-ad873578fc5c" />

=============================================================
1. PREREQUISITES
=============================================================
* Simulator: ModelSim or Questa Sim.
* Source File: System.sv (Contains Top, MIPS, DataPath, ALU, Memories, and Testbench).
* Program File: prog_vmac_1000.txt (Hex code for the benchmark).

=============================================================
2. SIMULATION INSTRUCTIONS
=============================================================

Step 1: Project Setup
---------------------
1. Open your simulator (ModelSim/Questa).
2. Open a new project (File > New > Project) or use your existing working directory.
3. Load the source file: Add 'System.sv' to the project.

Step 2: Compilation
-------------------
1. Compile the 'System.sv' file.
2. Ensure there are no compilation errors in the transcript.

Step 3: File Path Verification (CRITICAL)
-----------------------------------------
The simulation requires the program file 'prog_vmac_1000.txt' to be in the simulator's active working directory.

1. Type the following command in the Transcript/Console window to see the current path:
   pwd

2. Make sure the file 'prog_vmac_1000.txt' is located in that exact folder. If not, move the file there.

Step 4: Running the Simulation
------------------------------
1. Start Simulation:
   - Click Simulate > Start Simulation.
   - Expand the 'work' library.
   - Select 'Top_tb' (Note: Select 'Top_tb', NOT 'System' or 'Top').
   - Click OK.

2. Configure Waveforms:
   - In the 'Sim' or 'Objects' pane, navigate through this hierarchy:
     Top_tb -> dut -> mips_inst -> dp -> rf
   - Right-click on 'rf' (Register File) or specific signals you want to see.
   - Select 'Add to Wave'.

3. Execute:
   - Type the following two commands in the Transcript window:
     restart -f
     run 15000 ns

=============================================================
3. VERIFYING RESULTS
=============================================================
* Simulation Time: The enhanced processor should complete the task in approximately 10,070 ns.
* Expected Value: Check Register $1 (Accumulator) or Register $5 (Output). 
* The value should be: 0xFFF8EB80 (-464,000 decimal).
