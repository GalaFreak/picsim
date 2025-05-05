# PIC16F84 Simulator Documentation

This document provides comprehensive information on the PIC16F84 simulator, explaining both the backend architecture and the frontend user interface.

## Table of Contents

- [System Overview](#system-overview)
- [Backend Documentation](#backend-documentation)
- [Frontend Documentation](#frontend-documentation)
- [Test Programs](#test-programs)
- [Usage Guide](#usage-guide)

## System Overview

The PIC16F84 simulator is a comprehensive development and educational tool designed to simulate the behavior of the PIC16F84 microcontroller. It consists of two main components:

1. **Backend (picsim_backend.py)**: Implements the core simulation logic, including instruction execution, memory management, and peripheral simulation.
2. **Frontend (picsim_frontend_pyqt.py)**: Provides a graphical user interface using PyQt5, allowing users to interact with the simulator, visualize program execution, and manipulate I/O.

## Backend Documentation

### Architecture

The backend simulator implements a cycle-accurate simulation of the PIC16F84 microcontroller, including:

- 14-bit instruction execution engine
- Memory model with banking
- Special Function Registers (SFRs)
- I/O ports and peripherals
- Timer and interrupt handling
- EEPROM simulation

### Key Components

#### Memory Model

- **Program Memory**: 1K words (14-bit)
- **RAM**: 68 bytes general purpose registers, 16 special function registers
- **Banking**: Supports Bank 0 and Bank 1 switching via STATUS.RP0 bit
- **EEPROM**: 64 bytes of data EEPROM

#### Instruction Execution

The instruction execution follows the PIC16F84 datasheet, implementing:

- Byte-oriented file register operations
- Bit-oriented file register operations
- Literal and control operations
- Jump, call, and skip operations

#### Peripheral Simulation

- **Timer0**: With prescaler and internal/external clock source options
- **I/O Ports**: PORTA (5 bits) and PORTB (8 bits) with configurable direction
- **Interrupts**: External INT pin, Timer0 overflow, RB port change, EEPROM write complete
- **EEPROM**: Read and write operations with proper sequence checking

### Key Classes and Methods

#### PicSimulator

Main simulation engine with the following important methods:

- `decode_execute(opcode)`: Decodes and executes a single instruction
- `get_ram(address)` / `set_ram(address, value)`: Memory access with banking support
- `update_timer0(cycles)`: Updates Timer0 based on configuration
- `check_interrupts()`: Checks and processes interrupt conditions
- `reset(por)`: Resets the simulator (power-on or MCLR)
- `toggle_porta_pin(pin_index)` / `toggle_portb_pin(pin_index)`: Simulates external input changes

## Frontend Documentation

### GUI Layout

The GUI is divided into several sections:

1. **Control Panel**: Program loading, execution control, and frequency settings
2. **Runtime Display**: Cycle count and execution time information
3. **Special Function Registers (SFRs)**: Visualization and editing of SFRs
4. **Stack Display**: Stack contents visualization
5. **I/O Ports Panel**: Interactive visualization of PORTA and PORTB
6. **Code Listing**: Program view with current execution position and breakpoint support
7. **General Purpose Registers**: View and edit GPRs

### Main Components

#### Control Panel
- Load LST file, Test Program 14, and Test Program 15 buttons
- Reset, Step, Run, and Stop buttons
- Frequency control with slider and text input

#### Code Editor
- Displays LST file with line numbers
- Supports breakpoint toggling
- Highlights current execution line

#### I/O Visualization
- Interactive port pins that can be toggled for inputs
- Visual indicators for pin states (input/output, high/low)
- TRIS configuration display
- Color-coded pin states (inputs/outputs)

### Visualization Features

- **Stack**: Display of 8-level hardware stack with TOS (Top of Stack) highlighting
- **SFRs**: Real-time updating display of all Special Function Registers
- **Port Pins**: Visual representation of I/O pin states with tooltips showing detailed information
- **Execution Highlighting**: Current instruction is highlighted in the code listing

## Test Programs

### TPicSim14 - LED Sequence Generator

This test program demonstrates basic I/O manipulation by implementing a "Leuchtband" (LED sequence) pattern on PORTB. The direction of the sequence is determined by the state of RA0.

**Key features demonstrated:**
- Port I/O configuration and manipulation
- Use of carry flag for sequence control
- Conditional branching based on input
- Register rotation operations (RRF/RLF)

### TPicSim15 - Port Latch Test

This program tests the correct functioning of the port latch mechanism in the PIC microcontroller. It demonstrates how the internal latch value persists even when a pin is configured as an input, and how that value appears on the pin when the pin is reconfigured as an output.

**Key features demonstrated:**
- Port configuration (TRIS registers)
- Port latch behavior
- Pin state persistence during I/O direction changes

## Usage Guide

### Loading and Running Programs

1. **Load a Program**: 
   - Click "Load LST" to select a custom program file
   - Click "Load Test 14" or "Load Test 15" to load one of the built-in test programs

2. **Execution Control**:
   - Click "Run" to start continuous execution
   - Click "Step" for single-instruction execution
   - Click "Stop" to pause execution
   - Click "Reset" to reset the microcontroller

3. **Setting Breakpoints**:
   - Click in the line number margin to toggle breakpoints
   - Execution will automatically pause when a breakpoint is hit

### I/O Manipulation

1. **Configuring I/O Direction**:
   - Modify TRISA/TRISB registers (1=input, 0=output)
   - Direction is displayed in the I/O panel as "i" (input) or "o" (output)

2. **Changing Input Values**:
   - Click on pin cells in the "PIN" row of the I/O tables
   - Only pins configured as inputs can be toggled
   - Pin values are color-coded (blue=high, red=low for inputs)

3. **Observing Output Values**:
   - Output pins show the value of the corresponding PORT register
   - Green indicates high output, dark gray indicates low output
   - Special case: RA4 shows "Z" when high (open-drain)

### Frequency Control

1. Set the oscillator frequency using either:
   - The text input field (0.1 MHz to 16.0 MHz)
   - The slider control
   
2. The execution time display ("Laufzeit") will update according to the set frequency

## Memory and Register Editing

1. **Modifying Registers**:
   - Click on any editable register field and enter a new value
   - Values can be entered in hexadecimal (with 0x prefix) or decimal
   - Press Enter to confirm the change

2. **GPR Editing**:
   - All General Purpose Register values can be modified in a similar way
   - Changes take effect immediately
