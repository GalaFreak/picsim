# coding: utf-8
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
from tkinter import messagebox
from tkinter import scrolledtext
import re
import time

# Import our backend simulator
from picsim_backend import *

class PicSimulatorGUI:
    def __init__(self, master):
        self.master = master
        master.title("PIC16F84 Simulator")
        master.geometry("1200x900")

        # Create backend simulator
        self.simulator = PicSimulator()
        
        # Use shorthand references for frequently accessed simulator attributes
        self.prog_mem = self.simulator.prog_mem
        self.ram = self.simulator.ram
        self.breakpoints = set()  # Keep breakpoints in GUI since they're related to UI
        self.step_mode = False
        self.last_pc = 0
        
        # --- GUI Layout ---
        # Main Paned Window
        self.main_pane = ttk.PanedWindow(master, orient=tk.HORIZONTAL)
        self.main_pane.pack(fill=tk.BOTH, expand=True)

        # Left Pane (Controls, Registers, Stack)
        self.left_frame = ttk.Frame(self.main_pane, width=500)
        self.main_pane.add(self.left_frame, weight=1)

        # Right Pane (Code, GPRs)
        self.right_pane = ttk.PanedWindow(self.main_pane, orient=tk.VERTICAL)
        self.main_pane.add(self.right_pane, weight=2)  # Reduced weight to give left pane more space

        # --- Left Frame Content ---
        # Control Buttons
        self.control_frame = ttk.LabelFrame(self.left_frame, text="Controls")
        self.control_frame.pack(pady=5, padx=5, fill=tk.X)

        self.load_button = ttk.Button(self.control_frame, text="Load LST", command=self.load_lst_file)
        self.load_button.grid(row=0, column=0, padx=5, pady=5)
        self.reset_button = ttk.Button(self.control_frame, text="Reset", command=lambda: self.reset(por=False))
        self.reset_button.grid(row=0, column=1, padx=5, pady=5)
        self.step_button = ttk.Button(self.control_frame, text="Step", command=self.step_instruction)
        self.step_button.grid(row=0, column=2, padx=5, pady=5)
        self.run_button = ttk.Button(self.control_frame, text="Run", command=self.run_program)
        self.run_button.grid(row=0, column=3, padx=5, pady=5)
        self.stop_button = ttk.Button(self.control_frame, text="Stop", command=self.stop_program, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=4, padx=5, pady=5)

        # Frequency Setting
        self.freq_frame = ttk.Frame(self.control_frame)
        self.freq_frame.grid(row=1, column=0, columnspan=5, pady=5)
        ttk.Label(self.freq_frame, text="Quarz (MHz):").pack(side=tk.LEFT, padx=5)
        self.freq_var = tk.StringVar(value=str(self.simulator.frequency_mhz))
        self.freq_entry = ttk.Entry(self.freq_frame, textvariable=self.freq_var, width=5)
        self.freq_entry.bind("<Return>", self.update_frequency)
        self.freq_entry.pack(side=tk.LEFT, padx=5)

        # Runtime Display
        self.runtime_frame = ttk.LabelFrame(self.left_frame, text="Runtime")
        self.runtime_frame.pack(pady=5, padx=5, fill=tk.X)
        self.runtime_label_cycles = ttk.Label(self.runtime_frame, text="Cycles: 0")
        self.runtime_label_cycles.pack(anchor=tk.W, padx=5)
        self.runtime_label_us = ttk.Label(self.runtime_frame, text="Laufzeit: 0.00 us")
        self.runtime_label_us.pack(anchor=tk.W, padx=5)

        # Special Registers Display (Moved more to left and made more compact)
        self.sfr_frame = ttk.LabelFrame(self.left_frame, text="Special Function Registers (SFRs)")
        self.sfr_frame.pack(pady=5, padx=0, fill=tk.X)

        self.sfr_labels = {}
        self.sfr_values = {}
        sfr_to_display = {
            # Core
            "PC": self.simulator.pc, "W": self.simulator.w_reg, "STATUS": 0, "PCL": 0, "PCLATH": 0, "FSR": 0,
            # Peripherals / Control
            "TMR0": 0, "OPTION": 0, "INTCON": 0,
            "PORTA": 0, "TRISA": 0, "PORTB": 0, "TRISB": 0,
            "EEDATA": 0, "EEADR": 0, "EECON1": 0,
            # Flags / Derived (Read-Only in GUI)
            "RP0": 0, "Z": 0, "C": 0, "DC": 0, "TO": 0, "PD": 0,
            "GIE": 0, "T0IE": 0, "T0IF": 0, "INTE": 0, "INTF": 0,
            "RBIE": 0, "RBIF": 0, "EEIE": 0, "EEIF": 0
        }
        row, col = 0, 0
        # Order SFRs logically for display
        sfr_display_order = [
            "PC", "W", "STATUS", "PCL", "PCLATH", "FSR",
            "INTCON", "OPTION", "TMR0",
            "PORTA", "TRISA", "PORTB", "TRISB",
            "EEDATA", "EEADR", "EECON1",
            "C", "DC", "Z", "PD", "TO", "RP0",
            "GIE", "EEIE", "T0IE", "INTE", "RBIE",
            "EEIF", "T0IF", "INTF", "RBIF"
        ]

        # Adjust the SFR display to be more compact and flush left
        row, col = 0, 0
        max_cols = 8  # Increase the number of columns to make table more compact

        for name in sfr_display_order:
            if name not in sfr_to_display: continue

            lbl = ttk.Label(self.sfr_frame, text=f"{name}:", width=7)  # Fixed width for alignment
            lbl.grid(row=row, column=col, padx=2, pady=1, sticky=tk.W)
            self.sfr_labels[name] = lbl

            # Determine initial display format
            display_val = "0"
            width = 4
            is_editable = False
            if name in ["PC"]:
                display_val = "0x000"
                width = 6
                is_editable = False
            elif len(name) > 3 or name in ["W", "STATUS", "PCL", "PCLATH", "FSR", "TMR0", "OPTION", "INTCON", "PORTA", "TRISA", "PORTB", "TRISB", "EEDATA", "EEADR", "EECON1"]:
                display_val = "0x00"
                width = 5
                if name in ["W", "PCL", "PCLATH", "STATUS", "FSR", "TMR0", "OPTION", "INTCON", "PORTA", "TRISA", "PORTB", "TRISB", "EEDATA", "EEADR", "EECON1"]:
                    is_editable = True

            val_var = tk.StringVar(value=display_val)
            val_entry = ttk.Entry(self.sfr_frame, textvariable=val_var, width=width, justify=tk.RIGHT)
            val_entry.grid(row=row, column=col+1, padx=2, pady=1, sticky=tk.E)

            if is_editable:
                val_entry.bind("<Return>", lambda event, reg_name=name: self.edit_sfr(event, reg_name))
            else:
                val_entry.config(state=tk.DISABLED)

            self.sfr_values[name] = val_var

            col += 2
            if col >= max_cols:  # More columns to make table more compact
                col = 0
                row += 1

        # Create horizontal frame to hold Stack and I/O Ports side by side
        self.stack_io_frame = ttk.Frame(self.left_frame)
        self.stack_io_frame.pack(pady=5, padx=5, fill=tk.X)

        # Stack Display (Simplified - similar to W register display format)
        self.stack_frame = ttk.LabelFrame(self.stack_io_frame, text="Stack")
        self.stack_frame.pack(side=tk.LEFT, pady=5, padx=5, fill=tk.Y)
        
        # Create a frame for stack values
        self.stack_values_frame = ttk.Frame(self.stack_frame)
        self.stack_values_frame.pack(pady=5, padx=5)
        
        # Create entry-like labels for stack display (similar to W register)
        self.stack_values = {}
        for i in range(8):
            frame = ttk.Frame(self.stack_values_frame)
            frame.pack(fill=tk.X, pady=1)
            
            # Index label
            index_label = ttk.Label(frame, text=f"{i}:", width=2)
            index_label.pack(side=tk.LEFT)
            
            # Value display (similar to W register display)
            val_var = tk.StringVar(value="0x0000")
            val_entry = ttk.Entry(frame, textvariable=val_var, width=7, justify=tk.RIGHT, state=tk.DISABLED)
            val_entry.pack(side=tk.LEFT, padx=2)
            
            self.stack_values[i] = val_var

        # I/O Pin Stimulation (Moved to be next to stack)
        self.io_frame = ttk.LabelFrame(self.stack_io_frame, text="I/O Ports")
        self.io_frame.pack(side=tk.RIGHT, pady=5, padx=5, fill=tk.BOTH, expand=True)

        self.port_a_frame = ttk.LabelFrame(self.io_frame, text="PORTA / TRISA")
        self.port_a_frame.pack(pady=5, padx=5, fill=tk.X, anchor='n')
        self.port_b_frame = ttk.LabelFrame(self.io_frame, text="PORTB / TRISB")
        self.port_b_frame.pack(pady=5, padx=5, fill=tk.X, anchor='n')

        self.porta_pin_buttons = {}
        self.trisa_pin_labels = {}
        for i in range(5):  # RA0-RA4
            pin_name = f"RA{i}"
            frame = ttk.Frame(self.port_a_frame)
            frame.pack(side=tk.LEFT, padx=3)
            # TRIS Label
            lbl = ttk.Label(frame, text="I", anchor=tk.CENTER, width=4)  # Default Input
            lbl.pack()
            self.trisa_pin_labels[i] = lbl
            # Pin Button (Toggle Input)
            btn = tk.Button(frame, text=pin_name, width=4, relief=tk.RAISED, bg="light grey", command=lambda p=i: self.toggle_porta_pin(p))
            btn.pack()
            self.porta_pin_buttons[i] = btn

        self.portb_pin_buttons = {}
        self.trisb_pin_labels = {}
        for i in range(8):  # RB0-RB7
            pin_name = f"RB{i}"
            frame = ttk.Frame(self.port_b_frame)
            frame.pack(side=tk.LEFT, padx=3)
            # TRIS Label
            lbl = ttk.Label(frame, text="I", anchor=tk.CENTER, width=4)  # Default Input
            lbl.pack()
            self.trisb_pin_labels[i] = lbl
            # Pin Button (Toggle Input)
            btn = tk.Button(frame, text=pin_name, width=4, relief=tk.RAISED, bg="light grey", command=lambda p=i: self.toggle_portb_pin(p))
            btn.pack()
            self.portb_pin_buttons[i] = btn

        # --- Right Pane Content ---
        # Code Window with Breakpoint Buttons
        self.code_frame = ttk.LabelFrame(self.right_pane, text="Code Listing (.LST)")
        self.right_pane.add(self.code_frame, weight=2)  # More weight to code
        
        # Create a horizontal container for breakpoint buttons and code text
        self.code_container = ttk.Frame(self.code_frame)
        self.code_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create a frame for breakpoint buttons with fixed width
        self.breakpoint_frame = ttk.Frame(self.code_container, width=20)
        self.breakpoint_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        # Dictionary to store breakpoint buttons mapped to their addresses
        self.breakpoint_buttons = {}
        self.line_to_addr_map = {}  # Maps line numbers to addresses
        
        # Canvas for breakpoint buttons to enable scrolling
        self.bp_canvas = tk.Canvas(self.breakpoint_frame, width=20)
        self.bp_canvas.pack(side=tk.LEFT, fill=tk.Y)
        
        # Frame inside canvas to hold buttons
        self.bp_button_frame = ttk.Frame(self.bp_canvas)
        self.bp_canvas_window = self.bp_canvas.create_window((0, 0), window=self.bp_button_frame, anchor=tk.NW)
        
        # Create code text widget
        self.code_text = scrolledtext.ScrolledText(self.code_container, wrap=tk.NONE, width=80, height=20, font=("Courier New", 10))
        self.code_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.code_text.tag_configure("highlight", background="yellow")
        self.code_text.tag_configure("breakpoint", background="red", foreground="white")
        self.code_text.config(state=tk.DISABLED)  # Read-only initially
        
        # Get the ScrolledText's scrollbar
        self.code_text_yscroll = self.code_text.vbar
        
        # Sync scrolling between code text and breakpoint buttons
        self.code_text_yscroll.configure(command=self.sync_scroll)
        
        # GPR Display (Made wider by taking full width of right pane, not sharing with I/O)
        self.gpr_frame = ttk.LabelFrame(self.right_pane, text="General Purpose Registers (GPRs - Bank 0 View)")
        self.right_pane.add(self.gpr_frame, weight=1)  # Less weight than code

        self.gpr_canvas = tk.Canvas(self.gpr_frame)
        self.gpr_scrollbar = ttk.Scrollbar(self.gpr_frame, orient="vertical", command=self.gpr_canvas.yview)
        self.gpr_scrollable_frame = ttk.Frame(self.gpr_canvas)

        self.gpr_scrollable_frame.bind(
            "<Configure>",
            lambda e: self.gpr_canvas.configure(
                scrollregion=self.gpr_canvas.bbox("all")
            )
        )

        self.gpr_canvas.create_window((0, 0), window=self.gpr_scrollable_frame, anchor="nw")
        self.gpr_canvas.configure(yscrollcommand=self.gpr_scrollbar.set)

        self.gpr_canvas.pack(side="left", fill="both", expand=True)
        self.gpr_scrollbar.pack(side="right", fill="y")

        self.gpr_labels = {}
        self.gpr_values = {}
        # PIC16F84: 68 bytes GPR -> 0x0C to 0x4F (Bank 0) = 68 locations
        num_gpr_rows = (GPR_BANK0_END - GPR_BANK0_START + 1)
        gpr_cols = 10  # Increased number of columns to make display wider
        gpr_rows_per_col = (num_gpr_rows + gpr_cols - 1) // gpr_cols

        for i in range(num_gpr_rows):
            addr = GPR_BANK0_START + i
            addr_hex = f"{addr:02X}"

            col_num = (i // gpr_rows_per_col) * 2
            row_num = i % gpr_rows_per_col

            lbl = ttk.Label(self.gpr_scrollable_frame, text=f"{addr_hex}:")
            lbl.grid(row=row_num, column=col_num, padx=2, pady=1, sticky=tk.W)
            self.gpr_labels[addr] = lbl

            val_var = tk.StringVar(value="0x00")
            val_entry = ttk.Entry(self.gpr_scrollable_frame, textvariable=val_var, width=5, justify=tk.RIGHT)
            val_entry.grid(row=row_num, column=col_num+1, padx=2, pady=1)
            val_entry.bind("<Return>", lambda event, reg_addr=addr: self.edit_gpr(event, reg_addr))
            self.gpr_values[addr] = val_var

        # Call reset AFTER all GUI elements are created
        self.reset(por=True)  # Power-on Reset

        # Initial GUI Update
        self.update_gui()

    # --- GUI Update Methods ---
    def update_gui(self):
        """Updates all GUI elements with current simulator state."""
        # Update infrequently changing things first
        self.update_gui_stack()
        self.update_gui_runtime()
        # Update things that change often
        self.update_gui_sfr_gpr()
        self.update_io_pins()

    def update_gui_sfr_gpr(self):
        """Updates the SFR and GPR displays."""
        # Update SFRs (including derived flags)
        status = self.ram[SFR_STATUS_ADDR]
        intcon = self.ram[SFR_INTCON_ADDR]
        porta_latch = self.ram[SFR_PORTA_ADDR]
        portb_latch = self.ram[SFR_PORTB_ADDR]
        
        # Read bank 1 regs only if RP0 is set
        rp0 = (status >> STATUS_RP0) & 1
        option = self.ram[SFR_OPTION_REG_ADDR] if rp0 == 1 else 0  # Read 0 if wrong bank
        trisa = self.ram[SFR_TRISA_ADDR] if rp0 == 1 else 0xFF  # Read as inputs if wrong bank
        trisb = self.ram[SFR_TRISB_ADDR] if rp0 == 1 else 0xFF
        eecon1 = self.ram[SFR_EECON1_ADDR] if rp0 == 1 else 0

        sfr_map = {
            "PC": self.simulator.pc, "W": self.simulator.w_reg, "STATUS": status, "PCL": self.ram[SFR_PCL_ADDR],
            "PCLATH": self.ram[SFR_PCLATH_ADDR], "FSR": self.ram[SFR_FSR_ADDR],
            "INTCON": intcon, "OPTION": option, "TMR0": self.ram[SFR_TMR0_ADDR],
            "PORTA": porta_latch, "TRISA": trisa, "PORTB": portb_latch, "TRISB": trisb,
            "EEDATA": self.ram[SFR_EEDATA_ADDR], "EEADR": self.ram[SFR_EEADR_ADDR],
            "EECON1": eecon1,
            # Flags / Derived
            "C": (status >> STATUS_C) & 1, "DC": (status >> STATUS_DC) & 1,
            "Z": (status >> STATUS_Z) & 1, "PD": (status >> STATUS_PD) & 1,
            "TO": (status >> STATUS_TO) & 1, "RP0": rp0,
            "GIE": (intcon >> INTCON_GIE) & 1, "EEIE": (intcon >> INTCON_EEIE) & 1,
            "T0IE": (intcon >> INTCON_T0IE) & 1, "INTE": (intcon >> INTCON_INTE) & 1,
            "RBIE": (intcon >> INTCON_RBIE) & 1,
            "EEIF": (eecon1 >> EECON1_EEIF) & 1,
            "T0IF": (intcon >> INTCON_T0IF) & 1, "INTF": (intcon >> INTCON_INTF) & 1,
            "RBIF": (intcon >> INTCON_RBIF) & 1
        }

        for name, val_var in self.sfr_values.items():
            if name in sfr_map:
                current_val = sfr_map[name]
                if name in ["PC"]:
                    val_var.set(f"0x{current_val:03X}")
                # Check if name corresponds to a register that should be hex
                elif len(name) > 3 or name in ["W", "STATUS", "PCL", "PCLATH", "FSR", "INTCON", "OPTION", "TMR0", "PORTA", "TRISA", "PORTB", "TRISB", "EEDATA", "EEADR", "EECON1"]:
                    val_var.set(f"0x{current_val:02X}")
                else:  # Assume single bit flags display as decimal
                    val_var.set(str(current_val))

        # Update GPRs (Bank 0 view: 0x0C - 0x4F)
        for addr, val_var in self.gpr_values.items():
            val_var.set(f"0x{self.ram[addr]:02X}")

    def update_gui_stack(self):
        """Updates the stack display."""
        # Determine the index of the TOS (element BEFORE the next push)
        tos_index = (self.simulator.stack_ptr - 1 + 8) % 8
        
        # Update each stack entry value
        for i in range(8):
            addr = self.simulator.stack[i]
            # Update the value display
            if i == tos_index:
                # Add TOS indicator by changing the format
                self.stack_values[i].set(f"0x{addr:04X}*")
            else:
                self.stack_values[i].set(f"0x{addr:04X}")

    def update_gui_runtime(self):
        """Updates runtime counters."""
        self.runtime_label_cycles.config(text=f"Cycles: {self.simulator.runtime_cycles}")
        if self.simulator.frequency_mhz > 0:
            self.simulator.laufzeit_us = self.simulator.runtime_cycles * (4.0 / self.simulator.frequency_mhz)
            self.runtime_label_us.config(text=f"Laufzeit: {self.simulator.laufzeit_us:.2f} us")
        else:
            self.runtime_label_us.config(text="Laufzeit: N/A (Freq=0)")

    def update_io_pins(self):
        """Updates the I/O pin button appearances and TRIS labels."""
        porta_latch = self.ram[SFR_PORTA_ADDR]
        portb_latch = self.ram[SFR_PORTB_ADDR]
        rp0 = self.simulator.get_status_bit(STATUS_RP0)
        trisa = self.ram[SFR_TRISA_ADDR] if rp0 == 1 else 0xFF
        trisb = self.ram[SFR_TRISB_ADDR] if rp0 == 1 else 0xFF
        option = self.ram[SFR_OPTION_REG_ADDR] if rp0 == 1 else 0xFF
        rbpu_enabled = ((option >> OPTION_RBPU) & 1) == 0

        # PORTA
        for i in range(5):
            is_input = (trisa >> i) & 1
            pin_stimulus = (self.simulator.porta_pins >> i) & 1  # External level if input
            latch_level = (porta_latch >> i) & 1
            pin_name = f"RA{i}"
            button = self.porta_pin_buttons[i]
            label = self.trisa_pin_labels[i]

            if is_input:
                label.config(text=" I ")
                button.config(text=pin_name, bg="light blue" if pin_stimulus else "white smoke", relief=tk.RAISED, state=tk.NORMAL)
            else:  # Output
                label.config(text=" O ")
                output_level = latch_level
                # RA4 is open drain
                if i == 4:
                    pin_state_text = "L" if output_level == 0 else "Hi-Z"
                    bg_color = "gray50" if output_level == 0 else "yellow"
                    relief_state = tk.SUNKEN if output_level == 0 else tk.RAISED
                    button.config(text=f"{pin_name}\n{pin_state_text}", bg=bg_color, relief=relief_state, state=tk.DISABLED)
                else:  # RA0-RA3 are CMOS
                    pin_state_text = "H" if output_level else "L"
                    bg_color = "red" if output_level else "gray50"
                    button.config(text=f"{pin_name}\n{pin_state_text}", bg=bg_color, relief=tk.SUNKEN, state=tk.DISABLED)

        # PORTB
        portb_mismatch_found = False
        for i in range(8):
            is_input = (trisb >> i) & 1
            pin_stimulus = (self.simulator.portb_pins >> i) & 1  # External level if input
            latch_level = (portb_latch >> i) & 1
            pullup_active = is_input and rbpu_enabled
            pin_name = f"RB{i}"
            button = self.portb_pin_buttons[i]
            label = self.trisb_pin_labels[i]

            if is_input:
                label.config(text=" I" + ("(P)" if pullup_active else " "))
                # Button shows stimulated level
                button.config(text=pin_name, bg="light blue" if pin_stimulus else "white smoke", relief=tk.RAISED, state=tk.NORMAL)

                # Check for RB Port Change Interrupt (RB4-RB7)
                if 4 <= i <= 7:
                    # Compare current *input pin stimulus* with the value latched *on the last read/write* of PORTB
                    old_latched_level = (self.simulator.portb_latch_on_read >> i) & 1
                    if pin_stimulus != old_latched_level:
                        portb_mismatch_found = True
                        # Setting flag handled below after checking all pins
            else:  # Output
                label.config(text=" O ")
                output_level = latch_level
                pin_state_text = "H" if output_level else "L"
                bg_color = "red" if output_level else "gray50"
                button.config(text=f"{pin_name}\n{pin_state_text}", bg=bg_color, relief=tk.SUNKEN, state=tk.DISABLED)

        # Set RBIF if any mismatch was found on RB4-RB7 (and they are inputs)
        if portb_mismatch_found:
            if not self.simulator.get_intcon_bit(INTCON_RBIF):  # Set only if not already set
                self.simulator.set_intcon_bit(INTCON_RBIF)
                print("RB Port Change detected (Input != LatchOnRead), RBIF set.")

    def highlight_current_line(self):
        """Highlights the line corresponding to the current PC in the code view."""
        # Remove previous highlight
        self.code_text.tag_remove("highlight", "1.0", tk.END)

        # Find the text widget line index for the current PC using the map
        if self.simulator.pc in self.simulator.line_addr_map:
            line_start = self.simulator.line_addr_map[self.simulator.pc]
            line_end = f"{line_start} lineend"
            try:
                self.code_text.tag_add("highlight", line_start, line_end)
                # Scroll to the highlighted line only if not stepping manually
                if not self.step_mode:  # Only auto-scroll when running
                    self.code_text.see(f"{line_start} +2 lines")  # Scroll a bit past the line
                else:
                    self.code_text.see(line_start)  # Scroll exactly to line on step
            except tk.TclError:
                print(f"Error highlighting or scrolling to line {line_start}")  # Catch potential Tcl errors

    # --- Control Methods ---
    def load_lst_file(self):
        filepath = filedialog.askopenfilename(
            title="Open LST File",
            filetypes=(("LST files", "*.LST *.lst"), ("Text files", "*.txt"), ("All files", "*.*"))
        )
        if not filepath:
            return

        try:
            self.prog_mem = [0x3FFF] * PROG_MEM_SIZE  # NOP
            self.code_text.config(state=tk.NORMAL)
            self.code_text.delete(1.0, tk.END)
            self.breakpoints.clear()
            self.simulator.line_addr_map = {}  # Maps address to text widget line start "line.col"
            max_addr = -1

            # Clear existing breakpoint buttons
            for widget in self.bp_button_frame.winfo_children():
                widget.destroy()
            self.breakpoint_buttons.clear()

            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                line_idx = 1
                for line in f:
                    # Insert raw line first
                    line_start_index = f"{line_idx}.0"
                    self.code_text.insert(tk.END, line)
                    
                    # Create breakpoint button for this line
                    bp_button = tk.Button(
                        self.bp_button_frame,
                        width=1,
                        height=1,
                        bg="SystemButtonFace",  # Default background
                        relief=tk.RAISED,        # Default not pressed
                        bd=1                     # Small border
                    )
                    bp_button.pack(fill=tk.X, pady=0)
                    
                    # Parse line for address and opcode
                    match = re.match(r"^\s*([0-9A-Fa-f]{4})\s+([0-9A-Fa-f]{4})", line)
                    if match:
                        addr = int(match.group(1), 16)
                        opcode = int(match.group(2), 16)

                        if addr < PROG_MEM_SIZE:
                            # Store the program instruction
                            self.prog_mem[addr] = opcode
                            self.simulator.line_addr_map[addr] = line_start_index  # Map address to line index
                            self.line_to_addr_map[line_idx] = addr        # Map line to address (for button)
                            max_addr = max(max_addr, addr)
                            
                            # Configure breakpoint button
                            bp_button.config(command=lambda a=addr, b=bp_button: self.toggle_breakpoint_button(a, b))
                            self.breakpoint_buttons[addr] = bp_button
                        else:
                            print(f"Warning: Address {addr:04X} out of range in LST file.")

                    line_idx += 1

            # Configure the breakpoint canvas for proper scrolling
            self.bp_button_frame.update_idletasks()
            self.bp_canvas.config(scrollregion=self.bp_canvas.bbox("all"))
            
            # Adjust the code container to ensure proper alignment
            self.code_container.update_idletasks()
            
            self.code_text.config(state=tk.DISABLED)
            if max_addr == -1:
                messagebox.showwarning("Load LST", "No valid code lines found in LST file.")
            else:
                messagebox.showinfo("Load LST", f"LST file loaded successfully. Max address: 0x{max_addr:03X}")

            self.reset(por=True)  # Reset processor after loading

        except Exception as e:
            messagebox.showerror("Load LST Error", f"Failed to load or parse LST file:\n{e}")
            self.code_text.config(state=tk.DISABLED)

    def reset(self, por=True):
        """Resets the simulator state."""
        self.simulator.reset(por)
        self.simulator.running = False
        
        # Update GUI button states
        self.stop_button.config(state=tk.DISABLED)
        self.run_button.config(state=tk.NORMAL)
        self.step_button.config(state=tk.NORMAL)

        self.update_gui()
        self.highlight_current_line()

    def step_instruction(self):
        """Executes a single instruction cycle."""
        if self.simulator.running: return  # Don't step if running continuously

        try:
            self.step_mode = True
            self.last_pc = self.simulator.pc
            if self.simulator.pc >= PROG_MEM_SIZE:
                messagebox.showwarning("Execution Halted", f"Program Counter (0x{self.simulator.pc:03X}) out of bounds.")
                return

            opcode = self.prog_mem[self.simulator.pc]
            # Check for NOP / unimplemented memory before decoding
            if (opcode == 0x3FFF or opcode == 0x0000):  # Default value or potential erased state / NOP
                print(f"PC=0x{self.simulator.pc:03X}: NOP (0x{opcode:04X})")
                cycles_taken = 1
                self.simulator.pc = (self.simulator.pc + 1) & 0x1FFF  # Increment PC
            else:
                cycles_taken = self.simulator.decode_execute(opcode)  # This will increment PC if needed

            self.simulator.runtime_cycles += cycles_taken

            # Simulate peripheral updates per cycle (pass cycles_taken)
            self.simulator.update_timer0(cycles_taken)
            self.simulator.check_interrupts()  # Check after instruction execution

            self.update_gui()
            self.highlight_current_line()

        except ValueError as e:  # Catch specific opcode errors
            messagebox.showerror("Instruction Error", f"Error executing instruction at PC=0x{self.last_pc:03X} (Opcode: 0x{self.prog_mem[self.last_pc]:04X}):\n{e}")
            self.stop_program()  # Halt on error
        except Exception as e:
            messagebox.showerror("Runtime Error", f"Unexpected error at PC=0x{self.last_pc:03X}:\n{e}")
            self.stop_program()  # Halt on error
        finally:
            self.step_mode = False

    def run_program(self):
        """Runs the program continuously."""
        if self.simulator.running: return
        self.simulator.running = True
        self.run_button.config(state=tk.DISABLED)
        self.step_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.execute_cycle()

    def stop_program(self):
        """Stops continuous execution."""
        self.simulator.running = False
        # Ensure buttons exist before configuring
        if hasattr(self, 'run_button'): self.run_button.config(state=tk.NORMAL)
        if hasattr(self, 'step_button'): self.step_button.config(state=tk.NORMAL)
        if hasattr(self, 'stop_button'): self.stop_button.config(state=tk.DISABLED)

    def execute_cycle(self):
        """Executes one cycle and schedules the next if running."""
        if not self.simulator.running:
            self.stop_program()  # Ensure buttons are correct state
            return

        # Check for breakpoint before execution
        if self.simulator.pc in self.breakpoints:
            print(f"Breakpoint hit at address 0x{self.simulator.pc:03X}")
            messagebox.showinfo("Breakpoint Hit", f"Breakpoint hit at address 0x{self.simulator.pc:03X}")
            self.stop_program()
            self.update_gui()
            self.highlight_current_line()
            return

        try:
            self.last_pc = self.simulator.pc
            if self.simulator.pc >= PROG_MEM_SIZE:
                messagebox.showwarning("Execution Halted", f"Program Counter (0x{self.simulator.pc:03X}) out of bounds.")
                self.stop_program()
                return

            opcode = self.prog_mem[self.simulator.pc]
            cycles_taken = 0

            # Check for NOP / unimplemented memory before decoding
            if opcode == 0x3FFF or opcode == 0x0000:  # Default value or potential erased state / NOP
                print(f"PC=0x{self.simulator.pc:03X}: NOP (0x{opcode:04X})")
                cycles_taken = 1
                self.simulator.pc = (self.simulator.pc + 1) & 0x1FFF  # Increment PC
            else:
                cycles_taken = self.simulator.decode_execute(opcode)  # This will increment PC if needed

            self.simulator.runtime_cycles += cycles_taken

            # Simulate peripheral updates per cycle
            self.simulator.update_timer0(cycles_taken)
            self.simulator.check_interrupts()  # Check after instruction execution

            # --- GUI Update Scheduling ---
            # Update GUI regularly for better responsiveness
            update_interval_cycles = 10  # More frequent updates
            if self.simulator.runtime_cycles % update_interval_cycles == 0:
                self.update_gui()
                self.highlight_current_line()
                self.master.update_idletasks()  # Force processing of pending GUI events
                
                # Small delay to ensure the GUI can respond
                self.master.after(10, self.execute_cycle)  # 10ms delay between updates
            else:
                # If no GUI update, schedule next cycle
                self.master.after(1, self.execute_cycle)  # 1ms minimal delay

        except ValueError as e:  # Catch specific opcode errors
            messagebox.showerror("Instruction Error", f"Error executing instruction at PC=0x{self.last_pc:03X} (Opcode: 0x{self.prog_mem[self.last_pc]:04X}):\n{e}")
            self.stop_program()  # Halt on error
            self.update_gui()  # Show final state
            self.highlight_current_line()
        except Exception as e:
            messagebox.showerror("Runtime Error", f"Unexpected error at PC=0x{self.last_pc:03X}:\n{e}")
            self.stop_program()  # Stop on error
            self.update_gui()  # Show final state
            self.highlight_current_line()

    # --- GUI Interaction Methods ---
    def update_frequency(self, event=None):
        try:
            new_freq_str = self.freq_var.get().replace(',', '.')  # Allow comma as decimal sep
            new_freq = float(new_freq_str)
            if new_freq >= 0:
                self.simulator.frequency_mhz = new_freq
                print(f"Frequency updated to {self.simulator.frequency_mhz} MHz")
                self.update_gui_runtime()  # Recalculate laufzeit
            else:
                messagebox.showwarning("Invalid Frequency", "Frequency must be non-negative.")
                self.freq_var.set(str(self.simulator.frequency_mhz))  # Revert
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number for frequency.")
            self.freq_var.set(str(self.simulator.frequency_mhz))  # Revert
        # Lose focus from entry
        self.master.focus()

    def toggle_breakpoint(self, event):
        """Legacy method redirecting to button toggle when text is clicked"""
        # Save current view position
        current_position = self.code_text.yview()
        
        # Get line info and find corresponding button
        line_info = self.code_text.index(f"@{event.x},{event.y}")
        line_num = int(float(line_info))
        
        # Check if this line has an address
        if line_num in self.line_to_addr_map:
            addr = self.line_to_addr_map[line_num]
            if addr in self.breakpoint_buttons:
                # Simulate pressing the corresponding button
                self.toggle_breakpoint_button(addr, self.breakpoint_buttons[addr])
        
        # Restore view position
        self.code_text.yview_moveto(current_position[0])

    def toggle_breakpoint_button(self, addr, button):
        """Toggle breakpoint when button is clicked"""
        # Save current view position
        current_position = self.code_text.yview()
        
        # Toggle breakpoint
        if addr in self.breakpoints:
            # Remove breakpoint
            self.breakpoints.remove(addr)
            button.config(bg="SystemButtonFace", relief=tk.RAISED)
            # Remove breakpoint highlighting from text
            line_info = self.line_addr_map.get(addr, None)
            if line_info:
                line_end = f"{line_info} lineend"
                self.code_text.tag_remove("breakpoint", line_info, line_end)
            print(f"Breakpoint removed at 0x{addr:03X}")
        else:
            # Add breakpoint
            self.breakpoints.add(addr)
            button.config(bg="red", relief=tk.SUNKEN)
            # Add breakpoint highlighting to text
            line_info = self.line_addr_map.get(addr, None)
            if line_info:
                line_end = f"{line_info} lineend"
                self.code_text.tag_add("breakpoint", line_info, line_end)
            print(f"Breakpoint set at 0x{addr:03X}")
        
        # Restore view position
        self.code_text.yview_moveto(current_position[0])

    def sync_scroll(self, *args):
        """Synchronize scrolling between code text and breakpoint buttons"""
        self.code_text.yview(*args)  # Standard ScrolledText behavior
        self.bp_canvas.yview(*args)  # Also move the breakpoint canvas

    def toggle_porta_pin(self, pin_index):
        """Toggles the simulated input level of a PORTA pin."""
        # Only toggle if pin is currently set as input
        if self.simulator.get_tris_a_bit(pin_index):
            self.simulator.porta_pins ^= (1 << pin_index)  # Flip the bit
            pin_level = (self.simulator.porta_pins >> pin_index) & 1
            print(f"Toggled RA{pin_index} input stimulus to {pin_level}")
            self.update_io_pins()
            # Toggling RA4 might affect TMR0 if in external clock mode - handled in update_timer0

    def toggle_portb_pin(self, pin_index):
        """Toggles the simulated input level of a PORTB pin."""
        # Only toggle if pin is currently set as input
        if self.simulator.get_tris_b_bit(pin_index):
            # Store previous level for edge detection
            prev_level = (self.simulator.portb_pins >> pin_index) & 1
            # Flip the bit
            self.simulator.portb_pins ^= (1 << pin_index)
            current_level = (self.simulator.portb_pins >> pin_index) & 1
            print(f"Toggled RB{pin_index} input stimulus to {current_level}")

            # Check for RB0/INT edge immediately after toggle
            if pin_index == 0:
                option = self.ram[SFR_OPTION_REG_ADDR] if self.simulator.get_status_bit(STATUS_RP0) == 1 else 0xFF
                intedg = (option >> OPTION_INTEDG) & 1  # 1=Rising, 0=Falling
                is_rising_edge = prev_level == 0 and current_level == 1
                is_falling_edge = prev_level == 1 and current_level == 0

                if (intedg == 1 and is_rising_edge) or (intedg == 0 and is_falling_edge):
                    if not self.simulator.get_intcon_bit(INTCON_INTF):  # Set only if not already set
                        self.simulator.set_intcon_bit(INTCON_INTF)
                        print("INT/RB0 edge detected, INTF set.")

            # Update GUI (this will also handle RB Port Change check)
            self.update_io_pins()

    def edit_sfr(self, event, reg_name):
        widget = event.widget
        current_val_str = widget.get()
        try:
            new_value_str = widget.get()
            new_value = int(new_value_str, 0)  # Auto-detect base (0x hex, else dec)

            # --- Perform Write based on Register Name ---
            address_to_write = -1
            is_bank_1_reg = False

            # Map name to address, considering potential bank
            if reg_name == "W":
                self.simulator.w_reg = new_value & 0xFF
            elif reg_name == "PCL": address_to_write = SFR_PCL_ADDR
            elif reg_name == "PCLATH": address_to_write = SFR_PCLATH_ADDR
            elif reg_name == "STATUS": address_to_write = SFR_STATUS_ADDR
            elif reg_name == "FSR": address_to_write = SFR_FSR_ADDR
            elif reg_name == "INTCON": address_to_write = SFR_INTCON_ADDR
            elif reg_name == "TMR0": address_to_write = SFR_TMR0_ADDR  # Bank 0
            elif reg_name == "PORTA": address_to_write = SFR_PORTA_ADDR  # Bank 0
            elif reg_name == "PORTB": address_to_write = SFR_PORTB_ADDR  # Bank 0
            elif reg_name == "EEDATA": address_to_write = SFR_EEDATA_ADDR  # Bank 0
            elif reg_name == "EEADR": address_to_write = SFR_EEADR_ADDR  # Bank 0
            # Bank 1 Registers
            elif reg_name == "OPTION": address_to_write = SFR_OPTION_REG_ADDR; is_bank_1_reg = True
            elif reg_name == "TRISA": address_to_write = SFR_TRISA_ADDR; is_bank_1_reg = True
            elif reg_name == "TRISB": address_to_write = SFR_TRISB_ADDR; is_bank_1_reg = True
            elif reg_name == "EECON1": address_to_write = SFR_EECON1_ADDR; is_bank_1_reg = True

            if address_to_write != -1:
                print(f"Editing SFR {reg_name} (Addr: 0x{address_to_write:02X}) to 0x{new_value:02X}")
                self.simulator.set_ram(address_to_write, new_value & 0xFF)
            elif reg_name != "W":
                print(f"Cannot edit register '{reg_name}' directly.")

            # Lose focus
            self.master.focus()
            self.update_gui()  # Refresh display immediately after edit

        except ValueError:
            messagebox.showerror("Edit Error", f"Invalid value entered for {reg_name}: '{current_val_str}'")
            self.update_gui_sfr_gpr()  # Revert display
        except Exception as e:
            messagebox.showerror("Edit Error", f"Error setting {reg_name}: {e}")
            self.update_gui_sfr_gpr()  # Revert display

    def edit_gpr(self, event, address):
        widget = event.widget
        current_val_str = widget.get()
        try:
            new_value_str = widget.get()
            new_value = int(new_value_str, 0)  # Auto-detect base
            # GPR edits always target Bank 0 addresses in our representation
            if GPR_BANK0_START <= address <= GPR_BANK0_END:
                print(f"Editing GPR 0x{address:02X} to 0x{new_value:02X}")
                self.simulator.set_ram(address, new_value & 0xFF)  # Use set_ram for consistency
                # Lose focus
                self.master.focus()
            else:
                print(f"Invalid GPR address for edit: 0x{address:02X}")

        except ValueError:
            messagebox.showerror("Edit Error", f"Invalid value entered for GPR 0x{address:02X}: '{current_val_str}'")
            self.update_gui_sfr_gpr()  # Revert display
        except Exception as e:
            messagebox.showerror("Edit Error", f"Error setting GPR 0x{address:02X}: {e}")
            self.update_gui_sfr_gpr()  # Revert display

# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    app = PicSimulatorGUI(root)
    root.mainloop()