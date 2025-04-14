# coding: utf-8
import time
import threading  # For EEPROM Write Time simulation

# Constants based on PIC16F84 datasheet
PROG_MEM_SIZE = 0x400  # 1K words
RAM_SIZE = 0x80       # 128 bytes per bank
GPR_BANK0_START = 0x0C
GPR_BANK0_END = 0x4F
GPR_BANK1_START = 0x8C  # Bank 1 GPRs start at 0x8C (mapped to 0x0C)
GPR_BANK1_END = 0xCF   # Bank 1 GPRs end at 0xCF (mapped to 0x4F) - PIC16F84 has 68 bytes total GPR
EEPROM_SIZE = 0x40     # 64 bytes

# SFR Addresses (Bank 0)
SFR_INDF_ADDR = 0x00
SFR_TMR0_ADDR = 0x01
SFR_PCL_ADDR = 0x02
SFR_STATUS_ADDR = 0x03
SFR_FSR_ADDR = 0x04
SFR_PORTA_ADDR = 0x05
SFR_PORTB_ADDR = 0x06
SFR_EEDATA_ADDR = 0x08
SFR_EEADR_ADDR = 0x09
SFR_PCLATH_ADDR = 0x0A
SFR_INTCON_ADDR = 0x0B

# SFR Addresses (Bank 1) - Use addresses >= 0x80
SFR_OPTION_REG_ADDR = 0x81
SFR_TRISA_ADDR = 0x85
SFR_TRISB_ADDR = 0x86
SFR_EECON1_ADDR = 0x88
SFR_EECON2_ADDR = 0x89  # Not a physical register

# STATUS Register Bits
STATUS_C = 0
STATUS_DC = 1
STATUS_Z = 2
STATUS_PD = 3
STATUS_TO = 4
STATUS_RP0 = 5
STATUS_RP1 = 6  # Unused in F84
STATUS_IRP = 7  # Unused in F84

# OPTION_REG Bits
OPTION_PS0 = 0
OPTION_PS1 = 1
OPTION_PS2 = 2
OPTION_PSA = 3
OPTION_T0SE = 4
OPTION_T0CS = 5
OPTION_INTEDG = 6
OPTION_RBPU = 7

# INTCON Bits
INTCON_RBIF = 0
INTCON_INTF = 1
INTCON_T0IF = 2
INTCON_RBIE = 3
INTCON_INTE = 4
INTCON_T0IE = 5
INTCON_EEIE = 6
INTCON_GIE = 7

# EECON1 Bits
EECON1_RD = 0
EECON1_WR = 1
EECON1_WREN = 2
EECON1_WRERR = 3
EECON1_EEIF = 4

class PicSimulator:
    def __init__(self):
        # Simulator State
        self.prog_mem = [0x3FFF] * PROG_MEM_SIZE
        self.ram = [0] * (RAM_SIZE * 2)  # Combined Banks 0 and 1 RAM
        self.eeprom_data = [0] * EEPROM_SIZE
        self.pc = 0
        self.w_reg = 0
        self.stack = [0] * 8
        self.stack_ptr = 0
        self.runtime_cycles = 0
        self.laufzeit_us = 0.0
        self.frequency_mhz = 4.0  # Default frequency
        self.eecon2_write_step = 0
        self.prescaler_counter = 0
        self.tmr0_sync_stage1 = 0
        self.tmr0_sync_stage2 = 0
        self.tmr0_inhibit_cycles = 0
        self.porta_pins = 0
        self.portb_pins = 0
        self.portb_latch_on_read = 0
        self.running = False
        self.step_mode = False
        self.breakpoints = set()
        self.last_pc = 0
        self.line_addr_map = {}

    # --- Memory Access Methods ---
    def get_ram(self, address):
        """Reads from RAM, handling banking via STATUS.RP0"""
        # Ensure address is within valid range for direct/indirect mapping
        address &= 0xFF  # Mask to 8 bits for comparison logic below

        rp0 = (self.ram[SFR_STATUS_ADDR] >> STATUS_RP0) & 1

        if address == SFR_INDF_ADDR:  # Indirect Addressing uses FSR value
            effective_addr = self.ram[SFR_FSR_ADDR]
            if effective_addr == 0x00:
                return 0  # Reading INDF(0) returns 0

            # Determine the actual address based on FSR and RP0 for banked SFRs
            actual_addr_in_ram_array = 0
            # GPR Range (mapped)
            if (GPR_BANK0_START <= effective_addr <= GPR_BANK0_END) or \
               (GPR_BANK1_START <= effective_addr <= GPR_BANK1_END):
                actual_addr_in_ram_array = effective_addr & 0x7F  # Map to Bank 0 GPR range
            # SFR Range: Check bank bit RP0
            elif effective_addr < GPR_BANK0_START:  # SFR addresses 0x00-0x0B
                # Shared SFRs?
                if effective_addr in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                # Bank specific SFRs accessed via FSR<0x0C
                elif rp0 == 0 and effective_addr in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                elif rp0 == 1 and effective_addr in [0x01, 0x05, 0x06, 0x08, 0x09]:
                    actual_addr_in_ram_array = effective_addr | 0x80  # Access Bank 1 address space in array
                else:
                    return 0  # Accessing non-existent/wrong-bank SFR via indirect low address
            else:
                return 0  # Invalid FSR value

            # Ensure address is within bounds of our RAM array representation
            if 0 <= actual_addr_in_ram_array < len(self.ram):
                # Handle port reads
                return self.ram[actual_addr_in_ram_array]
            else:
                return 0  # Should not happen if logic above is correct

        else:  # Direct Addressing uses the instruction's 'f' value
            actual_addr_in_ram_array = 0
            # Determine address based on 'f' and RP0
            if address < GPR_BANK0_START:  # SFR 0x00 - 0x0B
                # Shared SFRs
                if address in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address
                # Bank 0 specific SFRs
                elif rp0 == 0 and address in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = address
                # Bank 1 specific SFRs
                elif rp0 == 1 and address in [0x01, 0x05, 0x06, 0x08, 0x09]:
                    actual_addr_in_ram_array = address | 0x80  # Access Bank 1 address
                else:
                    # Trying to access a bank-specific SFR when in the wrong bank
                    return 0  # Reads as 0
            elif GPR_BANK0_START <= address <= GPR_BANK0_END:  # Bank 0 GPR range
                actual_addr_in_ram_array = address
            elif address >= 0x80:  # Accessing address >= 0x80 explicitly
                # Bank 1 GPRs map to Bank 0 GPRs
                if GPR_BANK1_START <= address <= GPR_BANK1_END:
                    actual_addr_in_ram_array = address & 0x7F  # Map to Bank 0
                # Bank 1 SFRs accessed via direct Bank 1 address
                elif rp0 == 1 and address in [SFR_OPTION_REG_ADDR, SFR_TRISA_ADDR, SFR_TRISB_ADDR, SFR_EECON1_ADDR, SFR_EECON2_ADDR]:
                    actual_addr_in_ram_array = address
                # Shared SFRs accessed via their Bank 1 address alias
                elif address & 0x7F in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address & 0x7F
                else:
                    # Accessing non-existent Bank 1 address or wrong bank
                    return 0
            else:
                # Address between 0x50 and 0x7F (unused Bank 0 GPR space for F84)
                return 0

            # Read PORTA/B special case: Reads pins, not latch
            if actual_addr_in_ram_array == SFR_PORTA_ADDR:
                trisa = self.ram[SFR_TRISA_ADDR] if rp0 == 1 else 0xFF
                latch_val = self.ram[SFR_PORTA_ADDR]
                pin_val = self.porta_pins
                read_val = 0
                for i in range(5):
                    if (trisa >> i) & 1:  # If pin is input
                        read_val |= (pin_val & (1 << i))  # Read external pin state
                    else:  # If pin is output
                        read_val |= (latch_val & (1 << i))  # Read latch state
                # Mask upper bits as they are unimplemented
                return read_val & 0x1F
            elif actual_addr_in_ram_array == SFR_PORTB_ADDR:
                trisb = self.ram[SFR_TRISB_ADDR] if rp0 == 1 else 0xFF
                latch_val = self.ram[SFR_PORTB_ADDR]
                pin_val = self.portb_pins
                read_val = 0
                for i in range(8):
                    if (trisb >> i) & 1:  # Input
                        read_val |= (pin_val & (1 << i))
                    else:  # Output
                        read_val |= (latch_val & (1 << i))
                # Reading PORTB latches the value for interrupt-on-change comparison
                self.portb_latch_on_read = read_val
                return read_val
            elif 0 <= actual_addr_in_ram_array < len(self.ram):
                return self.ram[actual_addr_in_ram_array]
            else:
                return 0

    def set_ram(self, address, value):
        """Writes to RAM, handling banking via STATUS.RP0"""
        value &= 0xFF  # Ensure value is 8-bit
        address &= 0xFF  # Mask address for comparison logic

        rp0 = (self.ram[SFR_STATUS_ADDR] >> STATUS_RP0) & 1

        if address == SFR_INDF_ADDR:  # Indirect Addressing uses FSR value
            effective_addr = self.ram[SFR_FSR_ADDR]
            if effective_addr == 0x00:
                return  # Writing to INDF(0) is NOP

            # Determine the actual address based on FSR and RP0 for banked SFRs
            actual_addr_in_ram_array = -1  # Use -1 to indicate invalid initially
            # GPR Range (mapped)
            if (GPR_BANK0_START <= effective_addr <= GPR_BANK0_END) or \
               (GPR_BANK1_START <= effective_addr <= GPR_BANK1_END):
                actual_addr_in_ram_array = effective_addr & 0x7F  # Map to Bank 0 GPR range
            # SFR Range
            elif effective_addr < GPR_BANK0_START:  # SFR addresses 0x00-0x0B
                # Shared SFRs
                if effective_addr in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                # Bank specific SFRs
                elif rp0 == 0 and effective_addr in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                elif rp0 == 1 and effective_addr in [0x01, 0x05, 0x06, 0x08, 0x09]:
                    actual_addr_in_ram_array = effective_addr | 0x80  # Access Bank 1 address

            # Perform write if address is valid
            if 0 <= actual_addr_in_ram_array < len(self.ram):
                # Special handling for STATUS write
                if actual_addr_in_ram_array == SFR_STATUS_ADDR:
                    # Preserve read-only/hardware-controlled bits TO, PD
                    old_status = self.ram[SFR_STATUS_ADDR]
                    write_mask = ~((1 << STATUS_TO) | (1 << STATUS_PD))
                    new_status = (old_status & ~write_mask) | (value & write_mask)
                    self.ram[SFR_STATUS_ADDR] = new_status & 0xFF
                else:
                    self.ram[actual_addr_in_ram_array] = value
                # Call handlers for side effects
                self.call_sfr_write_handler(actual_addr_in_ram_array)

        else:  # Direct Addressing uses the instruction's 'f' value
            actual_addr_in_ram_array = -1
            # Determine address based on 'f' and RP0
            if address < GPR_BANK0_START:  # SFR 0x00 - 0x0B
                # Shared SFRs
                if address in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address
                # Bank 0 specific SFRs
                elif rp0 == 0 and address in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = address
                # else: Accessing Bank 1 SFR via Bank 0 address - NOP
            elif GPR_BANK0_START <= address <= GPR_BANK0_END:  # Bank 0 GPR range
                actual_addr_in_ram_array = address
            elif address >= 0x80:  # Accessing address >= 0x80 explicitly
                # Bank 1 GPRs map to Bank 0 GPRs
                if GPR_BANK1_START <= address <= GPR_BANK1_END:
                    actual_addr_in_ram_array = address & 0x7F  # Map to Bank 0
                # Bank 1 specific SFRs accessed via direct Bank 1 address
                elif rp0 == 1 and address in [SFR_OPTION_REG_ADDR, SFR_TRISA_ADDR, SFR_TRISB_ADDR, SFR_EECON1_ADDR, SFR_EECON2_ADDR]:
                    actual_addr_in_ram_array = address
                # Shared SFRs accessed via their Bank 1 address alias
                elif address & 0x7F in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address & 0x7F

            # Perform write if address is valid
            if 0 <= actual_addr_in_ram_array < len(self.ram):
                if actual_addr_in_ram_array == SFR_STATUS_ADDR:
                    old_status = self.ram[SFR_STATUS_ADDR]
                    write_mask = ~((1 << STATUS_TO) | (1 << STATUS_PD))
                    new_status = (old_status & ~write_mask) | (value & write_mask)
                    self.ram[SFR_STATUS_ADDR] = new_status & 0xFF
                else:
                    self.ram[actual_addr_in_ram_array] = value
                # Call handlers for side effects
                self.call_sfr_write_handler(actual_addr_in_ram_array)

    def call_sfr_write_handler(self, address):
        """Calls the appropriate handler after an SFR write."""
        # Map Bank 1 aliases to their base address for handlers
        base_addr = address & 0x7F
        bank1_addr = address if address >= 0x80 else -1  # Use actual Bank 1 address if applicable

        if base_addr == SFR_PORTA_ADDR: 
            self.handle_porta_write()
        elif base_addr == SFR_PORTB_ADDR: 
            self.handle_portb_write()
        elif bank1_addr == SFR_TRISA_ADDR: 
            self.handle_trisa_write()
        elif bank1_addr == SFR_TRISB_ADDR: 
            self.handle_trisb_write()
        elif bank1_addr == SFR_OPTION_REG_ADDR: 
            self.handle_option_write()
        elif base_addr == SFR_PCL_ADDR: 
            self.handle_pcl_write(is_computed_goto=False)
        elif base_addr == SFR_TMR0_ADDR: 
            self.handle_tmr0_write()
        elif bank1_addr == SFR_EECON1_ADDR: 
            self.handle_eecon1_write()
        elif bank1_addr == SFR_EECON2_ADDR: 
            self.handle_eecon2_write()

    def get_status_bit(self, bit_pos):
        return (self.ram[SFR_STATUS_ADDR] >> bit_pos) & 1

    def set_status_bit_from_op(self, bit_pos):
        """Sets a status bit as a result of an operation (e.g., BSF). Respects read-only bits."""
        if bit_pos == STATUS_RP0:
            self.ram[SFR_STATUS_ADDR] |= (1 << bit_pos)
        elif bit_pos in [STATUS_C, STATUS_DC, STATUS_Z]:
            self.ram[SFR_STATUS_ADDR] |= (1 << bit_pos)

    def clear_status_bit_from_op(self, bit_pos):
        """Clears a status bit as a result of an operation (e.g., BCF). Respects read-only bits."""
        if bit_pos == STATUS_RP0:
            self.ram[SFR_STATUS_ADDR] &= ~(1 << bit_pos)
        elif bit_pos in [STATUS_C, STATUS_DC, STATUS_Z]:
            self.ram[SFR_STATUS_ADDR] &= ~(1 << bit_pos)

    def update_status_flags(self, carry=None, digit_carry=None, zero=None):
        """Updates C, DC, Z flags based on operation result. None means flag is unaffected."""
        status = self.ram[SFR_STATUS_ADDR]
        if carry is not None:
            if carry: status |= (1 << STATUS_C)
            else: status &= ~(1 << STATUS_C)
        if digit_carry is not None:
            if digit_carry: status |= (1 << STATUS_DC)
            else: status &= ~(1 << STATUS_DC)
        if zero is not None:
            if zero: status |= (1 << STATUS_Z)
            else: status &= ~(1 << STATUS_Z)
        self.ram[SFR_STATUS_ADDR] = status

    def get_intcon_bit(self, bit_pos):
        return (self.ram[SFR_INTCON_ADDR] >> bit_pos) & 1

    def set_intcon_bit(self, bit_pos):
        """Sets INTCON bit, used for flags by peripherals or enables by BSF."""
        self.ram[SFR_INTCON_ADDR] |= (1 << bit_pos)

    def clear_intcon_bit(self, bit_pos):
        """Clears INTCON bit, used for flags by software (BCF) or enables by BCF."""
        self.ram[SFR_INTCON_ADDR] &= ~(1 << bit_pos)

    def get_option_bit(self, bit_pos):
        if self.get_status_bit(STATUS_RP0) == 1:
            return (self.ram[SFR_OPTION_REG_ADDR] >> bit_pos) & 1
        else:
            return 0

    def get_tris_a_bit(self, bit_pos):
        if bit_pos >= 5: return 1  # TRISA<7:5> read as 1 (unimplemented)
        if self.get_status_bit(STATUS_RP0) == 1:
            return (self.ram[SFR_TRISA_ADDR] >> bit_pos) & 1
        else:
            return 1

    def get_tris_b_bit(self, bit_pos):
        if self.get_status_bit(STATUS_RP0) == 1:
            return (self.ram[SFR_TRISB_ADDR] >> bit_pos) & 1
        else:
            return 1

    def get_eecon1_bit(self, bit_pos):
        if self.get_status_bit(STATUS_RP0) == 1:
            return (self.ram[SFR_EECON1_ADDR] >> bit_pos) & 1
        else:
            return 0

    def set_eecon1_bit(self, bit_pos):
        if self.get_status_bit(STATUS_RP0) == 1:
            self.ram[SFR_EECON1_ADDR] |= (1 << bit_pos)
            if bit_pos == EECON1_RD or bit_pos == EECON1_WR:
                self.handle_eecon1_write()

    def clear_eecon1_bit(self, bit_pos):
        if self.get_status_bit(STATUS_RP0) == 1:
            if bit_pos == EECON1_EEIF or bit_pos == EECON1_WREN:
                self.ram[SFR_EECON1_ADDR] &= ~(1 << bit_pos)

    # --- Register Write Handlers (for side effects) ---
    def handle_porta_write(self):
        pass  # To be overridden or extended in GUI class

    def handle_portb_write(self):
        self.portb_latch_on_read = self.ram[SFR_PORTB_ADDR]

    def handle_trisa_write(self):
        pass  # To be overridden or extended in GUI class

    def handle_trisb_write(self):
        pass  # To be overridden or extended in GUI class

    def handle_option_write(self):
        self.prescaler_counter = 0  # Simplification: Clear prescaler on OPTION write

    def handle_pcl_write(self, is_computed_goto=False):
        """Handles writing to PCL."""
        pcl = self.ram[SFR_PCL_ADDR]
        pclath_val = self.ram[SFR_PCLATH_ADDR]
        
        if is_computed_goto:
            # For ADDWF PCL,F and similar instructions:
            high_bits = (pclath_val & 0x18) << 5  # PCLATH<4:3> -> PC<12:11>
            mid_bits = (pclath_val & 0x07) << 8   # PCLATH<2:0> -> PC<10:8>
            self.pc = high_bits | mid_bits | pcl
            print(f"Computed GOTO: New PC = 0x{self.pc:03X} (PCL={pcl:02X}, PCLATH={pclath_val:02X})")
        else:
            # For direct writes (MOVWF PCL):
            self.pc = ((pclath_val & 0x1F) << 8) | pcl
            print(f"PCL Direct Write: New PC = 0x{self.pc:03X} (PCL={pcl:02X}, PCLATH={pclath_val:02X})")
        
        self.pc &= 0x1FFF  # Ensure PC is within 13 bits

    def handle_tmr0_write(self):
        self.tmr0_inhibit_cycles = 2
        if self.get_option_bit(OPTION_PSA) == 0:
            self.prescaler_counter = 0
            print("TMR0 Write: Prescaler cleared.")

    def handle_eecon1_write(self):
        eecon1 = self.ram[SFR_EECON1_ADDR]
        
        if (eecon1 & (1 << EECON1_RD)):
            print("EECON1 Write: RD bit set, initiating EEPROM read.")
            self.execute_eeprom_read()
            self.ram[SFR_EECON1_ADDR] &= ~(1 << EECON1_RD)

        if (eecon1 & (1 << EECON1_WR)):
            print(f"EECON1 Write: WR bit set. WREN={(eecon1 >> EECON1_WREN) & 1}, SeqStep={self.eecon2_write_step}")
            if (eecon1 & (1 << EECON1_WREN)) and self.eecon2_write_step == 2:
                self.execute_eeprom_write()
                self.ram[SFR_EECON1_ADDR] &= ~(1 << EECON1_WR)
                self.ram[SFR_EECON1_ADDR] |= (1 << EECON1_EEIF)
                print("EECON1 Write: Write complete, EEIF set.")
                self.check_interrupts()
            else:
                self.ram[SFR_EECON1_ADDR] &= ~(1 << EECON1_WR)
                print("EECON1 Write: Invalid sequence or WREN=0. Write aborted.")
            self.eecon2_write_step = 0

    def handle_eecon2_write(self):
        val = self.ram[SFR_EECON2_ADDR]
        if self.eecon2_write_step == 0 and val == 0x55:
            self.eecon2_write_step = 1
            print("EECON2 Write: Step 1 (0x55) received.")
        elif self.eecon2_write_step == 1 and val == 0xAA:
            self.eecon2_write_step = 2
            print("EECON2 Write: Step 2 (0xAA) received.")
        else:
            self.eecon2_write_step = 0
            print(f"EECON2 Write: Invalid sequence value 0x{val:02X} received.")
        self.ram[SFR_EECON2_ADDR] = 0

    def execute_eeprom_read(self):
        addr = self.ram[SFR_EEADR_ADDR] & (EEPROM_SIZE - 1)
        self.ram[SFR_EEDATA_ADDR] = self.eeprom_data[addr]
        print(f"EEPROM Read: Addr=0x{addr:02X} -> Data=0x{self.ram[SFR_EEDATA_ADDR]:02X}")

    def execute_eeprom_write(self):
        addr = self.ram[SFR_EEADR_ADDR] & (EEPROM_SIZE - 1)
        data = self.ram[SFR_EEDATA_ADDR]
        self.eeprom_data[addr] = data
        print(f"EEPROM Write: Addr=0x{addr:02X} Data=0x{data:02X}")

    def update_timer0(self, cycles_elapsed=1):
        """Updates TMR0 based on configuration."""
        option = self.ram[SFR_OPTION_REG_ADDR] if self.get_status_bit(STATUS_RP0) == 1 else 0xFF
        psa = (option >> OPTION_PSA) & 1
        t0cs = (option >> OPTION_T0CS) & 1

        prescaler_mask = 0b111
        prescaler_bits = option & prescaler_mask
        tmr0_prescale = 1
        if psa == 0:  # Prescaler assigned to TMR0
            tmr0_prescale = 2 << prescaler_bits  # 2, 4, 8, ..., 256

        tmr0_val_before = self.ram[SFR_TMR0_ADDR]

        for _ in range(cycles_elapsed):
            if self.tmr0_inhibit_cycles > 0:
                self.tmr0_inhibit_cycles -= 1
                continue

            clock_edge_occurred = False
            if t0cs == 0:  # Internal clock (Fosc/4)
                clock_edge_occurred = True
            else:  # External clock (RA4/TOCKI pin)
                clock_edge_occurred = True  # Simplified simulation

            if clock_edge_occurred:
                self.prescaler_counter += 1
                if self.prescaler_counter >= tmr0_prescale:
                    self.prescaler_counter = 0
                    tmr0_val = self.ram[SFR_TMR0_ADDR]
                    tmr0_val = (tmr0_val + 1) & 0xFF
                    self.ram[SFR_TMR0_ADDR] = tmr0_val
                    if tmr0_val == 0 and tmr0_val_before == 0xFF:
                        self.set_intcon_bit(INTCON_T0IF)
                        print("TMR0 Overflow, T0IF set.")

    def check_interrupts(self):
        """Checks if any enabled interrupt has occurred and triggers ISR jump if GIE is set."""
        intcon = self.ram[SFR_INTCON_ADDR]
        gie = (intcon >> INTCON_GIE) & 1

        if not gie:
            return  # Interrupts globally disabled

        interrupt_pending = False
        vector_addr = 0x0004

        # Check interrupt sources
        if (intcon >> INTCON_INTE) & 1 and (intcon >> INTCON_INTF) & 1:
            interrupt_pending = True
            print("INT Interrupt Pending (INTE=1, INTF=1)")

        if (intcon >> INTCON_T0IE) & 1 and (intcon >> INTCON_T0IF) & 1:
            interrupt_pending = True
            print("TMR0 Interrupt Pending (T0IE=1, T0IF=1)")

        if (intcon >> INTCON_RBIE) & 1 and (intcon >> INTCON_RBIF) & 1:
            interrupt_pending = True
            print("RB Port Change Interrupt Pending (RBIE=1, RBIF=1)")

        if self.get_status_bit(STATUS_RP0) == 1:
            eecon1 = self.ram[SFR_EECON1_ADDR]
            if (intcon >> INTCON_EEIE) & 1 and (eecon1 >> EECON1_EEIF) & 1:
                interrupt_pending = True
                print("EEPROM Write Complete Interrupt Pending (EEIE=1, EEIF=1)")

        if interrupt_pending:
            print(f"Interrupt occurred! GIE was set. Jumping to ISR (0x{vector_addr:04X})")
            self.ram[SFR_INTCON_ADDR] &= ~(1 << INTCON_GIE)
            self.push_stack(self.pc)
            self.pc = vector_addr
            self.runtime_cycles += 2

    def push_stack(self, address):
        self.stack[self.stack_ptr] = address & 0x1FFF
        print(f"Stack Push: SP={self.stack_ptr} Addr=0x{self.stack[self.stack_ptr]:04X}")
        self.stack_ptr = (self.stack_ptr + 1) % 8

    def pop_stack(self):
        self.stack_ptr = (self.stack_ptr - 1 + 8) % 8
        addr = self.stack[self.stack_ptr]
        print(f"Stack Pop: SP={self.stack_ptr} Addr=0x{addr:04X}")
        return addr

    def decode_execute(self, opcode):
        """Decodes and executes a 14-bit opcode. Returns cycles taken."""
        cycles = 1  # Most instructions are 1 cycle
        pc_increment = True  # Most instructions increment PC by 1 after execution

        # Extract common fields
        f_addr = opcode & 0x007F  # File register address (7 bits)
        dest_d = (opcode >> 7) & 1  # Destination bit (0=W, 1=f)
        bit_b = (opcode >> 7) & 7  # Bit address (3 bits)
        literal_k8 = opcode & 0x00FF  # 8-bit literal
        literal_k11 = opcode & 0x07FF  # 11-bit literal

        # --- Decode based on opcode patterns (datasheet Table 9-2) ---

        # NOP (Can be encoded multiple ways, 0x0000 is common)
        if opcode == 0x0000 or (opcode & 0b11111110011111) == 0:  # Matches 00 0000 0xx0 0000
            print(f"PC=0x{self.pc:03X}: NOP")
            pass  # Do nothing, PC increments later

        # Byte-oriented file register operations (00 xxxx ....)
        elif (opcode >> 11) == 0b000:
            op_sub = (opcode >> 8) & 0b1111  # Bits 11:8 determine specific instruction

            if opcode == 0b00000001100100:  # CLRWDT
                print(f"PC=0x{self.pc:03X}: CLRWDT")
                # 00h -> WDT, 0 -> WDT prescaler, 1 -> TO, 1 -> PD
                # TODO: Implement WDT state and prescaler clearing
                status = self.ram[SFR_STATUS_ADDR]
                status |= (1 << STATUS_TO) | (1 << STATUS_PD)
                self.ram[SFR_STATUS_ADDR] = status
            elif opcode == 0b00000001100011:  # SLEEP
                print(f"PC=0x{self.pc:03X}: SLEEP")
                # 00h -> WDT, 0 -> WDT prescaler, 1 -> TO, 0 -> PD
                # TODO: Implement WDT clearing, oscillator stop simulation
                status = self.ram[SFR_STATUS_ADDR]
                status |= (1 << STATUS_TO)
                status &= ~(1 << STATUS_PD)
                self.ram[SFR_STATUS_ADDR] = status
                self.running = False  # Halt simulation for SLEEP
                print("SLEEP instruction executed. Halting.")
                pc_increment = True  # PC should increment then halt
            elif op_sub == 0b0000 and (opcode & 0x80):  # MOVWF f (00 0000 1fff ffff)
                print(f"PC=0x{self.pc:03X}: MOVWF 0x{f_addr:02X}")
                self.set_ram(f_addr, self.w_reg)
            elif op_sub == 0b0001 and not (opcode & 0x80):  # CLRW (00 0001 0xxx xxxx)
                print(f"PC=0x{self.pc:03X}: CLRW")
                self.w_reg = 0
                self.update_status_flags(zero=True)
            elif op_sub == 0b0001 and (opcode & 0x80):  # CLRF f (00 0001 1fff ffff)
                print(f"PC=0x{self.pc:03X}: CLRF 0x{f_addr:02X}")
                # CLRF affects Z flag, but does not read the register first
                self.set_ram(f_addr, 0)
                self.update_status_flags(zero=True)
            elif op_sub == 0b0010:  # SUBWF f,d (00 0010 dfff ffff)
                print(f"PC=0x{self.pc:03X}: SUBWF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result_16 = val_f - self.w_reg  # Perform subtraction
                result_8 = result_16 & 0xFF
                # Flags: C=NOT borrow, DC=NOT digit borrow, Z
                carry = result_16 >= 0  # C=1 if result positive or zero (no borrow)
                digit_carry = (val_f & 0x0F) - (self.w_reg & 0x0F) >= 0  # DC=1 if no borrow from bit 4
                zero = result_8 == 0
                self.update_status_flags(carry=carry, digit_carry=digit_carry, zero=zero)
                if dest_d == 0: self.w_reg = result_8
                else: self.set_ram(f_addr, result_8)
            elif op_sub == 0b0011:  # DECF f,d (00 0011 dfff ffff)
                print(f"PC=0x{self.pc:03X}: DECF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result = (val_f - 1) & 0xFF
                zero = result == 0
                self.update_status_flags(zero=zero)  # Only Z affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b0100:  # IORWF f,d (00 0100 dfff ffff)
                print(f"PC=0x{self.pc:03X}: IORWF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result = (val_f | self.w_reg) & 0xFF
                zero = result == 0
                self.update_status_flags(zero=zero)  # Only Z affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b0101:  # ANDWF f,d (00 0101 dfff ffff)
                print(f"PC=0x{self.pc:03X}: ANDWF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result = (val_f & self.w_reg) & 0xFF
                zero = result == 0
                self.update_status_flags(zero=zero)  # Only Z affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b0110:  # XORWF f,d (00 0110 dfff ffff)
                print(f"PC=0x{self.pc:03X}: XORWF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result = (val_f ^ self.w_reg) & 0xFF
                zero = result == 0
                self.update_status_flags(zero=zero)  # Only Z affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b0111:  # ADDWF f,d (00 0111 dfff ffff)
                print(f"PC=0x{self.pc:03X}: ADDWF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result_16 = val_f + self.w_reg
                result_8 = result_16 & 0xFF
                # Flags: C, DC, Z
                carry = (result_16 >> 8) & 1
                digit_carry = ((val_f & 0x0F) + (self.w_reg & 0x0F)) > 0x0F
                zero = result_8 == 0
                self.update_status_flags(carry=carry, digit_carry=digit_carry, zero=zero)
                
                # Special case: ADDWF PCL, F - handle as "Computed GOTO"
                is_computed_goto = (f_addr == SFR_PCL_ADDR and dest_d == 1)
                
                if dest_d == 0: 
                    self.w_reg = result_8
                else: 
                    self.ram[f_addr] = result_8
                    if is_computed_goto:
                        # Handle computed GOTO directly
                        self.handle_pcl_write(is_computed_goto=True)
                        cycles = 2  # ADDWF PCL takes 2 cycles
                        pc_increment = False  # PC already set by handle_pcl_write
                    elif f_addr != SFR_PCL_ADDR:  
                        # Only call handler for non-PCL registers
                        self.call_sfr_write_handler(f_addr)
            elif op_sub == 0b1000:  # MOVF f,d (00 1000 dfff ffff)
                print(f"PC=0x{self.pc:03X}: MOVF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                zero = val_f == 0
                self.update_status_flags(zero=zero)  # Only Z affected
                if dest_d == 0: 
                    self.w_reg = val_f
                else:
                    self.set_ram(f_addr, val_f)  # d=1: Write value back to f (even if unchanged)
            elif op_sub == 0b1001:  # COMF f,d (00 1001 dfff ffff)
                print(f"PC=0x{self.pc:03X}: COMF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result = (~val_f) & 0xFF
                zero = result == 0
                self.update_status_flags(zero=zero)  # Only Z affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b1010:  # INCF f,d (00 1010 dfff ffff)
                print(f"PC=0x{self.pc:03X}: INCF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result = (val_f + 1) & 0xFF
                zero = result == 0
                self.update_status_flags(zero=zero)  # Only Z affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b1011:  # DECFSZ f,d (00 1011 dfff ffff)
                val_f = self.get_ram(f_addr)
                result = (val_f - 1) & 0xFF
                skip = result == 0
                print(f"PC=0x{self.pc:03X}: DECFSZ 0x{f_addr:02X}, {'F' if dest_d else 'W'}{' (Skip)' if skip else ''}")
                # Z flag is NOT affected by DECFSZ/INCFSZ
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
                if skip:
                    pc_increment = False  # Will add 2 to PC manually
                    self.pc = (self.pc + 2) & 0x1FFF
                    cycles = 2  # Takes 2 cycles if skip
            elif op_sub == 0b1100:  # RRF f,d (00 1100 dfff ffff)
                print(f"PC=0x{self.pc:03X}: RRF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                old_carry = self.get_status_bit(STATUS_C)
                new_carry = val_f & 1
                result = (old_carry << 7) | (val_f >> 1)
                self.update_status_flags(carry=new_carry)  # Only C affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b1101:  # RLF f,d (00 1101 dfff ffff)
                print(f"PC=0x{self.pc:03X}: RLF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                old_carry = self.get_status_bit(STATUS_C)
                new_carry = (val_f >> 7) & 1
                result = ((val_f << 1) | old_carry) & 0xFF
                self.update_status_flags(carry=new_carry)  # Only C affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b1110:  # SWAPF f,d (00 1110 dfff ffff)
                print(f"PC=0x{self.pc:03X}: SWAPF 0x{f_addr:02X}, {'F' if dest_d else 'W'}")
                val_f = self.get_ram(f_addr)
                result = ((val_f & 0x0F) << 4) | ((val_f & 0xF0) >> 4)
                # Flags not affected
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
            elif op_sub == 0b1111:  # INCFSZ f,d (00 1111 dfff ffff)
                val_f = self.get_ram(f_addr)
                result = (val_f + 1) & 0xFF
                skip = result == 0
                print(f"PC=0x{self.pc:03X}: INCFSZ 0x{f_addr:02X}, {'F' if dest_d else 'W'}{' (Skip)' if skip else ''}")
                # Z flag is NOT affected by DECFSZ/INCFSZ
                if dest_d == 0: self.w_reg = result
                else: self.set_ram(f_addr, result)
                if skip:
                    pc_increment = False  # Will add 2 to PC manually
                    self.pc = (self.pc + 2) & 0x1FFF
                    cycles = 2  # Takes 2 cycles if skip

            # --- Special Instructions in 00xxxx group ---
            # Check specific opcodes after handling sub-groups
            elif opcode == 0b00000000001000:  # RETURN
                print(f"PC=0x{self.pc:03X}: RETURN")
                self.pc = self.pop_stack()
                cycles = 2
                pc_increment = False  # PC is set by pop
            elif opcode == 0b00000000001001:  # RETFIE
                print(f"PC=0x{self.pc:03X}: RETFIE")
                self.pc = self.pop_stack()
                self.set_intcon_bit(INTCON_GIE)  # Enable global interrupts
                cycles = 2
                pc_increment = False  # PC is set by pop
            elif (opcode & 0b11111111111000) == 0b00000001100000:  # TRIS f (00 0000 0110 0fff)
                tris_reg_num = opcode & 0b111  # 5, 6, or 7
                print(f"PC=0x{self.pc:03X}: TRIS {tris_reg_num}")
                if tris_reg_num == 5:  # TRISA
                    self.set_ram(SFR_TRISA_ADDR, self.w_reg)
                elif tris_reg_num == 6:  # TRISB
                    self.set_ram(SFR_TRISB_ADDR, self.w_reg)
                # Ignore TRISC for F84
            elif opcode == 0b00000001100010:  # OPTION (00 0000 0110 0010)
                print(f"PC=0x{self.pc:03X}: OPTION")
                self.set_ram(SFR_OPTION_REG_ADDR, self.w_reg)
            # If none of the above matched within 00xxxx group, it's likely an error or unhandled NOP variant
            else:
                print(f"PC=0x{self.pc:03X}: Unhandled Opcode in 00xxxx range: {opcode:04X}")
                # Treat as NOP?
                pass


        # Bit-oriented file register operations (01 xxxx ....)
        elif (opcode >> 10) == 0b0100:  # BCF f,b (01 00bb bfff ffff)
            print(f"PC=0x{self.pc:03X}: BCF 0x{f_addr:02X}, {bit_b}")
            # Read-modify-write
            current_val = self.get_ram(f_addr)
            new_val = current_val & ~(1 << bit_b)
            # Special handling for clearing STATUS/INTCON flags/enables if needed
            if f_addr == SFR_STATUS_ADDR or (f_addr == (SFR_STATUS_ADDR | 0x80)):
                self.clear_status_bit_from_op(bit_b)  # Handles RP0, ignores TO/PD
            elif f_addr == SFR_INTCON_ADDR or (f_addr == (SFR_INTCON_ADDR | 0x80)):
                self.clear_intcon_bit(bit_b)  # Handles flags vs enables
            elif f_addr == SFR_EECON1_ADDR:  # Only addressable in Bank 1
                self.clear_eecon1_bit(bit_b)
            else:
                self.set_ram(f_addr, new_val)  # Normal write
        elif (opcode >> 10) == 0b0101:  # BSF f,b (01 01bb bfff ffff)
            print(f"PC=0x{self.pc:03X}: BSF 0x{f_addr:02X}, {bit_b}")
            # Read-modify-write
            current_val = self.get_ram(f_addr)
            new_val = current_val | (1 << bit_b)
            # Special handling for setting STATUS/INTCON flags/enables if needed
            if f_addr == SFR_STATUS_ADDR or (f_addr == (SFR_STATUS_ADDR | 0x80)):
                self.set_status_bit_from_op(bit_b)  # Handles RP0, ignores TO/PD
            elif f_addr == SFR_INTCON_ADDR or (f_addr == (SFR_INTCON_ADDR | 0x80)):
                self.set_intcon_bit(bit_b)  # Sets flag or enable
            elif f_addr == SFR_EECON1_ADDR:
                self.set_eecon1_bit(bit_b)  # Handles RD/WR side effects
            else:
                self.set_ram(f_addr, new_val)  # Normal write
        elif (opcode >> 10) == 0b0110:  # BTFSC f,b (01 10bb bfff ffff)
            val_f = self.get_ram(f_addr)
            bit_is_clear = (val_f & (1 << bit_b)) == 0
            print(f"PC=0x{self.pc:03X}: BTFSC 0x{f_addr:02X}, {bit_b}{' (Skip)' if bit_is_clear else ''}")
            if bit_is_clear:
                pc_increment = False  # Will add 2 to PC manually
                self.pc = (self.pc + 2) & 0x1FFF
                cycles = 2
        elif (opcode >> 10) == 0b0111:  # BTFSS f,b (01 11bb bfff ffff)
            val_f = self.get_ram(f_addr)
            bit_is_set = (val_f & (1 << bit_b)) != 0
            print(f"PC=0x{self.pc:03X}: BTFSS 0x{f_addr:02X}, {bit_b}{' (Skip)' if bit_is_set else ''}")
            if bit_is_set:
                pc_increment = False  # Will add 2 to PC manually
                self.pc = (self.pc + 2) & 0x1FFF
                cycles = 2

        # CALL and GOTO (10 xxxx ....)
        elif (opcode >> 11) == 0b100:  # CALL k (10 0kkk kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: CALL 0x{literal_k11:03X}")
            self.push_stack(self.pc + 1)  # Push return address
            pclath_high = self.ram[SFR_PCLATH_ADDR]
            # CALL uses PCLATH<4:3> for PC<12:11>
            target_pc = ((pclath_high & 0b00011000) << (8-3)) | literal_k11
            self.pc = target_pc & 0x1FFF  # Ensure 13-bit PC
            cycles = 2
            pc_increment = False  # PC is set by instruction
        elif (opcode >> 11) == 0b101:  # GOTO k (10 1kkk kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: GOTO 0x{literal_k11:03X}")
            pclath_high = self.ram[SFR_PCLATH_ADDR]
            # GOTO uses PCLATH<4:3> for PC<12:11>
            target_pc = ((pclath_high & 0b00011000) << (8-3)) | literal_k11
            self.pc = target_pc & 0x1FFF  # Ensure 13-bit PC
            cycles = 2
            pc_increment = False  # PC is set by instruction

        # Literal and control operations (11 xxxx ....)
        elif (opcode >> 10) == 0b1100:  # MOVLW k (11 00xx kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: MOVLW 0x{literal_k8:02X}")
            self.w_reg = literal_k8
            # Flags not affected
        elif (opcode >> 10) == 0b1101:  # RETLW k (11 01xx kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: RETLW 0x{literal_k8:02X}")
            self.w_reg = literal_k8
            self.pc = self.pop_stack()
            cycles = 2
            pc_increment = False  # PC is set by pop
        elif (opcode >> 8) == 0b111000:  # IORLW k (11 1000 kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: IORLW 0x{literal_k8:02X}")
            self.w_reg = (self.w_reg | literal_k8) & 0xFF
            self.update_status_flags(zero=(self.w_reg == 0))  # Only Z affected
        elif (opcode >> 8) == 0b111001:  # ANDLW k (11 1001 kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: ANDLW 0x{literal_k8:02X}")
            self.w_reg = (self.w_reg & literal_k8) & 0xFF
            self.update_status_flags(zero=(self.w_reg == 0))  # Only Z affected
        elif (opcode >> 8) == 0b111010:  # XORLW k (11 1010 kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: XORLW 0x{literal_k8:02X}")
            self.w_reg = (self.w_reg ^ literal_k8) & 0xFF
            self.update_status_flags(zero=(self.w_reg == 0))  # Only Z affected
        elif (opcode >> 9) == 0b11110:  # SUBLW k (11 110x kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: SUBLW 0x{literal_k8:02X}")
            result_16 = literal_k8 - self.w_reg
            result_8 = result_16 & 0xFF
            # Flags: C=NOT borrow, DC=NOT digit borrow, Z
            carry = result_16 >= 0
            digit_carry = (literal_k8 & 0x0F) - (self.w_reg & 0x0F) >= 0
            zero = result_8 == 0
            self.update_status_flags(carry=carry, digit_carry=digit_carry, zero=zero)
            self.w_reg = result_8
        elif (opcode >> 9) == 0b11111:  # ADDLW k (11 111x kkkk kkkk)
            print(f"PC=0x{self.pc:03X}: ADDLW 0x{literal_k8:02X}")
            result_16 = literal_k8 + self.w_reg
            result_8 = result_16 & 0xFF
            # Flags: C, DC, Z
            carry = (result_16 >> 8) & 1
            digit_carry = ((literal_k8 & 0x0F) + (self.w_reg & 0x0F)) > 0x0F
            zero = result_8 == 0
            self.update_status_flags(carry=carry, digit_carry=digit_carry, zero=zero)
            self.w_reg = result_8

        # Fallback for completely unknown opcodes
        else:
            print(f"PC=0x{self.pc:03X}: Unknown opcode: {opcode:04X}")
            raise ValueError(f"Unknown opcode 0x{opcode:04X}")

        # Increment PC for next instruction (if not already handled by branch/skip/call/return)
        if pc_increment:
            self.pc = (self.pc + 1) & 0x1FFF  # Mask to 13 bits

        return cycles

    def update_bank_dependent_regs(self):
        """Update any state that depends on the current bank."""
        # This function is mainly for frontend use, but included here for completeness
        pass

    def reset(self, por=True):
        """Resets the simulator state."""
        print("--- RESET ---", "POR" if por else "MCLR")
        self.pc = 0
        self.w_reg = 0
        self.stack = [0] * 8
        self.stack_ptr = 0
        self.runtime_cycles = 0
        self.laufzeit_us = 0.0
        self.eecon2_write_step = 0
        self.prescaler_counter = 0
        self.tmr0_inhibit_cycles = 0
        self.porta_pins = 0
        self.portb_pins = 0
        self.portb_latch_on_read = 0

        # Clear GPRs
        for i in range(GPR_BANK0_START, GPR_BANK0_END + 1):
            self.ram[i] = 0x00

        # Initialize SFRs to reset states
        if por:
            self.ram[SFR_PCL_ADDR] = 0x00
            self.ram[SFR_STATUS_ADDR] = 0b00011000  # TO=1, PD=1, others 0
            self.ram[SFR_FSR_ADDR] = 0x00
            self.ram[SFR_PORTA_ADDR] = 0x00
            self.ram[SFR_PORTB_ADDR] = 0x00
            self.ram[SFR_EEDATA_ADDR] = 0x00
            self.ram[SFR_EEADR_ADDR] = 0x00
            self.ram[SFR_PCLATH_ADDR] = 0x00
            self.ram[SFR_INTCON_ADDR] = 0x00

            # Bank 1 SFRs
            self.ram[SFR_OPTION_REG_ADDR] = 0xFF
            self.ram[SFR_TRISA_ADDR] = 0b00011111
            self.ram[SFR_TRISB_ADDR] = 0xFF
            self.ram[SFR_EECON1_ADDR] = 0b00000000
            self.ram[SFR_EECON2_ADDR] = 0

            self.ram[SFR_TMR0_ADDR] = 0x00
        else:
            # MCLR Reset
            self.ram[SFR_PCL_ADDR] = 0x00
            self.ram[SFR_PCLATH_ADDR] &= 0b11100000
            self.ram[SFR_INTCON_ADDR] &= 0b00000111
            self.ram[SFR_EECON1_ADDR] &= ~((1 << EECON1_WREN) | (1 << EECON1_WR) | (1 << EECON1_RD))

        self.running = False
