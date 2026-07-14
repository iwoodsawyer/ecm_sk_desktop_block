# NEXTW EC-01M EtherCAT Master for Simulink Desktop Real-Time

This project provides a high-performance **Simulink Desktop Real-Time (SLDRT)** S-Function block for interfacing with the **NEXTW EC-01M** EtherCAT Master chip via USB. 

It is specifically designed for engineers and researchers requiring a low-latency, real-time EtherCAT solution within the Windows kernel environment, leveraging the unique capabilities of the EC-01M USB Starter Kit.

## Key Features
- **Kernel-Mode Support:** Optimized for Simulink Desktop Real-Time, allowing for reliable real-time execution in the Windows kernel.
- **Flexible Control Modes:** Supports Cyclic Sync Position (CSP), Velocity (CSV), and Torque (CST) modes.
- **Distributed Clock (DC):** Supports DC Synchronization for precise multi-axis timing.
- **Integrated I/O:** Supports both ECM local digital I/O and external EtherCAT I/O slave modules.
- **High Slave Capacity:** Supports up to 32/42 slaves depending on the packet variant (12-byte or 16-byte).

## Supported Hardware
- **Master:** [EtherCAT Universal USB Master Card ECM-SK (Starter Kit)](http://www.nextw.com.tw/product/)
- **Tested Slaves:** DIEWU EtherCAT Slave IO Modules (16 Input / 16 Output) and standard CiA 402 compliant servo drives.

## Prerequisites
- **Software:**
  - MATLAB & Simulink (R2018a or later recommended).
  - Simulink Desktop Real-Time toolbox.
  - Microsoft Visual C++ (MSVC) Compiler (configured via `mex -setup C++`).
- **Drivers:** 
  - The device uses WinUSB/SetupAPI. Ensure the vendor drivers for the ECM-SK are installed.

## Installation & Compilation
1. Clone or download this repository to your MATLAB workspace.
2. Open MATLAB and navigate to the project folder.
3. Run the compilation script:
   ```matlab
   make_EC01M_SFunction```
4. This will generate EC01M_SFunction.mexw64, which can then be used in your Simulink models.

## Block Configuration
Parameters
- Num Slaves: Number of EtherCAT slaves connected.
- Slave Types: Array of types (e.g., [1 1] for two drives). 1=DRIVE, 2=IO, 3=HSP, 4=STEP.
- Drive Mode: 8=CSP, 9=CSV, 10=CST.
- Sync Mode: 0=Free-Run, 1=DC Sync.
- DC Cycle (µs): Cycle time for Distributed Clock (e.g., 1000 for 1ms).
- Sample Time: Simulink model sample time (seconds).
- IO Refresh: Refresh rate for digital I/O frames (ticks).

## Input Ports
- Enable: Boolean (1 = Servo ON, 0 = Servo OFF).
- TargetCmd: Array of target values (Position, Velocity, or Torque) for each slave.
- IOOutputs: Bitmask for local and slave digital outputs.

## Output Ports
- ActualPos: Current position feedback (0x6064).
- StatusWord: CiA 402 status word (0x6041).
- ActualVel: Current velocity (0x606C) — Requires #define USE_16B in source.
- ECMState: Current EtherCAT Master state.
- FIFOCount: USB buffer status (monitor this to prevent overflows).
- IOInputs: Bitmask of digital inputs.

## Troubleshooting
- USB Timeout: Ensure the ECM-SK is connected and recognized in the Windows Device Manager.
- Real-Time Violations: If "FIFOCount" drops too low, increase the sample time or optimize the model.
- Velocity Feedback: Actual velocity is zero by default in the 12-byte packet mode. Recompile with USE_16B defined in EC01M_SFunction.cpp if high-speed velocity feedback is required.

## References
- NEXTW Official Website (https://www.nextw.com.tw/ec01m/)
- EC-01M Datasheet 
- ECM-SK User Guide