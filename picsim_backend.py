# coding: utf-8

# Constants based on PIC16F84 datasheet
PROG_MEM_SIZE = 0x400  # 1K words
RAM_SIZE = 0x80       # 128 bytes per bank
GPR_BANK0_START = 0x0C  # First GPR in Bank 0
GPR_BANK0_END = 0x4F    # Last GPR in Bank 0
GPR_BANK1_START = 0x8C  # First GPR in Bank 1 (maps to same physical memory as 0x0C)
GPR_BANK1_END = 0xCF    # Last GPR in Bank 1 (maps to same physical memory as 0x4F)
EEPROM_SIZE = 0x40      # 64 bytes

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
    def __init__(self, *args, **kwargs):
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
        self.frequency_mhz = 4.0  # Default frequency (4 MHz)
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
        
        # Watchdog Timer state
        self.wdt_counter = 0
        self.wdt_enabled = True  # WDT is enabled by default on power-up
        self.wdt_timeout = 18000  # Exact WDT timeout in cycles at 1MHz
        self.on_wdt_timeout = None  # Callback for WDT timeout notification
        self.sleep_mode = False  # Track if processor is in sleep mode
        self.wdt_last_reset_time = 0  # Track when WDT was last reset
        self.wdt_elapsed_us = 0.0
        self.wdt_timeout_us = 18000.0  # 18 ms in microseconds

    # --- Memory Access Methods ---
    def get_ram(self, address):
        """Reads from RAM, handling banking via STATUS.RP0"""
        # Ensure address is within valid range for direct/indirect mapping
        address &= 0xFF  # Mask to 8 bits for comparison logic below

        rp0 = (self.ram[SFR_STATUS_ADDR] >> STATUS_RP0) & 1
        actual_addr_in_ram_array = -1 # Use -1 for invalid/unmapped initially

        if address == SFR_INDF_ADDR:  # Indirect Addressing uses FSR value
            effective_addr = self.ram[SFR_FSR_ADDR]
            if effective_addr == 0x00:
                return 0  # Reading INDF(0) returns 0

            # Determine the actual address based on FSR
            # GPR Range (mapped)
            if (GPR_BANK0_START <= effective_addr <= GPR_BANK0_END) or \
               (GPR_BANK1_START <= effective_addr <= GPR_BANK1_END):
                actual_addr_in_ram_array = effective_addr & 0x7F  # Map to Bank 0 GPR range (0x0C-0x4F)
            # SFR Range
            elif effective_addr < GPR_BANK0_START:  # SFR addresses 0x00-0x0B
                # Shared SFRs (accessible regardless of RP0)
                if effective_addr in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                # Bank 0 specific SFRs (accessed via low address)
                elif rp0 == 0 and effective_addr in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                # Bank 1 specific SFRs (accessed via low address alias when RP0=1)
                elif rp0 == 1 and effective_addr in [0x01, 0x05, 0x06, 0x08, 0x09]: # OPTION, TRISA, TRISB, EECON1, EECON2 aliases
                    actual_addr_in_ram_array = effective_addr | 0x80 # Map to Bank 1 address (0x81, 0x85, 0x86, 0x88, 0x89)
                # else: Accessing wrong bank via low address -> invalid
            elif effective_addr >= 0x80: # SFR addresses 0x80+
                 # Shared SFRs (accessed via high address alias)
                if effective_addr & 0x7F in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                     actual_addr_in_ram_array = effective_addr & 0x7F
                 # Bank 1 specific SFRs (accessed via high address)
                elif rp0 == 1 and effective_addr in [SFR_OPTION_REG_ADDR, SFR_TRISA_ADDR, SFR_TRISB_ADDR, SFR_EECON1_ADDR, SFR_EECON2_ADDR]:
                     actual_addr_in_ram_array = effective_addr
                # else: Accessing Bank 0 SFR via high address, or Bank 1 SFR when RP0=0 -> invalid

            # else: FSR points to unimplemented memory (e.g., 0x50-0x7F) -> invalid

        else:  # Direct Addressing uses the instruction's 'f' value
            # Determine address based on 'f' and RP0
            if address < GPR_BANK0_START:  # SFR 0x00 - 0x0B
                # Shared SFRs
                if address in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address
                # Bank 0 specific SFRs
                elif rp0 == 0 and address in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = address
                # Bank 1 specific SFRs (accessed via low address alias when RP0=1)
                elif rp0 == 1 and address in [0x01, 0x05, 0x06, 0x08, 0x09]: # OPTION, TRISA, TRISB, EECON1, EECON2 aliases
                    actual_addr_in_ram_array = address | 0x80 # Map to Bank 1 address
                # else: Accessing wrong bank via low address -> invalid
            elif GPR_BANK0_START <= address <= GPR_BANK0_END:  # Bank 0 GPR range
                actual_addr_in_ram_array = address
            elif address >= 0x80:  # Accessing address >= 0x80 explicitly
                # Bank 1 GPRs map to Bank 0 GPRs
                if GPR_BANK1_START <= address <= GPR_BANK1_END:
                    actual_addr_in_ram_array = address & 0x7F  # Map to Bank 0 (0x0C-0x4F)
                # Bank 1 specific SFRs accessed via direct Bank 1 address
                elif rp0 == 1 and address in [SFR_OPTION_REG_ADDR, SFR_TRISA_ADDR, SFR_TRISB_ADDR, SFR_EECON1_ADDR, SFR_EECON2_ADDR]:
                    actual_addr_in_ram_array = address
                # Shared SFRs accessed via their Bank 1 address alias
                elif address & 0x7F in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address & 0x7F
                # else: Accessing non-existent Bank 1 address or wrong bank -> invalid
            # else: Address between 0x50 and 0x7F (unused Bank 0 GPR space for F84) -> invalid

        # Perform read if address is valid and within RAM array bounds
        if 0 <= actual_addr_in_ram_array < len(self.ram):
            # Read PORTA/B special case: Reads pins for inputs, latch for outputs
            if actual_addr_in_ram_array == SFR_PORTA_ADDR:
                # Need TRISA which is in Bank 1
                # Read TRISA directly from its Bank 1 location
                trisa_val = self.ram[SFR_TRISA_ADDR]
                latch_val = self.ram[SFR_PORTA_ADDR]
                pin_val = self.porta_pins
                read_val = 0
                for i in range(5):
                    if (trisa_val >> i) & 1:  # If pin is input (TRISA bit = 1)
                        read_val |= (pin_val & (1 << i))  # Read external pin state
                    else:  # If pin is output (TRISA bit = 0)
                        read_val |= (latch_val & (1 << i))  # Read latch state
                # Mask upper bits as they are unimplemented
                return read_val & 0x1F
            elif actual_addr_in_ram_array == SFR_PORTB_ADDR:
                # Need TRISB which is in Bank 1
                # Read TRISB directly from its Bank 1 location
                trisb_val = self.ram[SFR_TRISB_ADDR]
                latch_val = self.ram[SFR_PORTB_ADDR] # Read current latch value
                pin_val = self.portb_pins # Read current pin stimulus
                read_val = 0
                for i in range(8):
                    if (trisb_val >> i) & 1:  # If pin is input (TRISB bit = 1)
                        read_val |= (pin_val & (1 << i)) # Read external pin state
                    else:  # If pin is output (TRISB bit = 0)
                        read_val |= (latch_val & (1 << i)) # Read latch state
                # Reading PORTB latches the value for interrupt-on-change comparison
                self.portb_latch_on_read = read_val # Latch the value *read* from the pins/latches
                return read_val
            else:
                # Normal RAM/SFR read
                return self.ram[actual_addr_in_ram_array]
        else:
            # Invalid address calculated or out of bounds
            # print(f"Warning: Read from invalid/unmapped address. Address={address:02X}, EffAddr={effective_addr if address == SFR_INDF_ADDR else address:02X}, RP0={rp0}, MappedAddr={actual_addr_in_ram_array}")
            return 0

    def set_ram(self, address, value):
        """Writes to RAM, handling banking via STATUS.RP0"""
        value &= 0xFF  # Ensure value is 8-bit
        address &= 0xFF  # Mask address for comparison logic

        rp0 = (self.ram[SFR_STATUS_ADDR] >> STATUS_RP0) & 1
        actual_addr_in_ram_array = -1 # Use -1 for invalid/unmapped initially

        if address == SFR_INDF_ADDR:  # Indirect Addressing uses FSR value
            effective_addr = self.ram[SFR_FSR_ADDR]
            if effective_addr == 0x00:
                return  # Writing to INDF(0) is NOP

            # Determine the actual address based on FSR
            # GPR Range (mapped)
            if (GPR_BANK0_START <= effective_addr <= GPR_BANK0_END) or \
               (GPR_BANK1_START <= effective_addr <= GPR_BANK1_END):
                actual_addr_in_ram_array = effective_addr & 0x7F  # Map to Bank 0 GPR range (0x0C-0x4F)
            # SFR Range
            elif effective_addr < GPR_BANK0_START:  # SFR addresses 0x00-0x0B
                # Shared SFRs (accessible regardless of RP0)
                if effective_addr in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                # Bank 0 specific SFRs (accessed via low address)
                elif rp0 == 0 and effective_addr in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = effective_addr
                # Bank 1 specific SFRs (accessed via low address alias when RP0=1)
                elif rp0 == 1 and effective_addr in [0x01, 0x05, 0x06, 0x08, 0x09]: # OPTION, TRISA, TRISB, EECON1, EECON2 aliases
                    actual_addr_in_ram_array = effective_addr | 0x80 # Map to Bank 1 address (0x81, 0x85, 0x86, 0x88, 0x89)
                # else: Accessing wrong bank via low address -> invalid
            elif effective_addr >= 0x80: # SFR addresses 0x80+
                 # Shared SFRs (accessed via high address alias)
                if effective_addr & 0x7F in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                     actual_addr_in_ram_array = effective_addr & 0x7F
                 # Bank 1 specific SFRs (accessed via high address)
                elif rp0 == 1 and effective_addr in [SFR_OPTION_REG_ADDR, SFR_TRISA_ADDR, SFR_TRISB_ADDR, SFR_EECON1_ADDR, SFR_EECON2_ADDR]:
                     actual_addr_in_ram_array = effective_addr
                # else: Accessing Bank 0 SFR via high address, or Bank 1 SFR when RP0=0 -> invalid

            # else: FSR points to unimplemented memory (e.g., 0x50-0x7F) -> invalid

        else:  # Direct Addressing uses the instruction's 'f' value
            # Determine address based on 'f' and RP0
            if address < GPR_BANK0_START:  # SFR 0x00 - 0x0B
                # Shared SFRs
                if address in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address
                # Bank 0 specific SFRs
                elif rp0 == 0 and address in [SFR_TMR0_ADDR, SFR_PORTA_ADDR, SFR_PORTB_ADDR, SFR_EEDATA_ADDR, SFR_EEADR_ADDR]:
                    actual_addr_in_ram_array = address
                # Bank 1 specific SFRs (accessed via low address alias when RP0=1)
                elif rp0 == 1 and address in [0x01, 0x05, 0x06, 0x08, 0x09]: # OPTION, TRISA, TRISB, EECON1, EECON2 aliases
                    actual_addr_in_ram_array = address | 0x80 # Map to Bank 1 address
                # else: Accessing wrong bank via low address -> invalid
            elif GPR_BANK0_START <= address <= GPR_BANK0_END:  # Bank 0 GPR range
                actual_addr_in_ram_array = address
            elif address >= 0x80:  # Accessing address >= 0x80 explicitly
                # Bank 1 GPRs map to Bank 0 GPRs
                if GPR_BANK1_START <= address <= GPR_BANK1_END:
                    actual_addr_in_ram_array = address & 0x7F  # Map to Bank 0 (0x0C-0x4F)
                # Bank 1 specific SFRs accessed via direct Bank 1 address
                elif rp0 == 1 and address in [SFR_OPTION_REG_ADDR, SFR_TRISA_ADDR, SFR_TRISB_ADDR, SFR_EECON1_ADDR, SFR_EECON2_ADDR]:
                    actual_addr_in_ram_array = address
                # Shared SFRs accessed via their Bank 1 address alias
                elif address & 0x7F in [SFR_PCL_ADDR, SFR_STATUS_ADDR, SFR_FSR_ADDR, SFR_PCLATH_ADDR, SFR_INTCON_ADDR]:
                    actual_addr_in_ram_array = address & 0x7F
                # else: Accessing non-existent Bank 1 address or wrong bank -> invalid
            # else: Address between 0x50 and 0x7F (unused Bank 0 GPR space for F84) -> invalid

        # Perform write if address is valid and within RAM array bounds
        if 0 <= actual_addr_in_ram_array < len(self.ram):
            # Special handling for STATUS write (preserve read-only bits TO, PD)
            if actual_addr_in_ram_array == SFR_STATUS_ADDR:
                old_status = self.ram[SFR_STATUS_ADDR]
                write_mask = ~((1 << STATUS_TO) | (1 << STATUS_PD))
                new_status = (old_status & ~write_mask) | (value & write_mask)
                self.ram[SFR_STATUS_ADDR] = new_status & 0xFF
            # Special handling for EECON2 write (sequence check)
            elif actual_addr_in_ram_array == SFR_EECON2_ADDR:
                self.handle_eecon2_write(value) # Pass value for sequence check
                # Don't actually write to EECON2, it's not a real register
            else:
                # Normal RAM/SFR write
                self.ram[actual_addr_in_ram_array] = value

            # Call handlers for side effects AFTER writing the value
            self.call_sfr_write_handler(actual_addr_in_ram_array)
        else:
            # Invalid address calculated or out of bounds
            # print(f"Warning: Write to invalid/unmapped address. Address={address:02X}, Value={value:02X}, EffAddr={effective_addr if address == SFR_INDF_ADDR else address:02X}, RP0={rp0}, MappedAddr={actual_addr_in_ram_array}")
            pass

    def call_sfr_write_handler(self, address):
        """Calls the appropriate handler after an SFR write."""
        # Note: address is the actual index in the self.ram array (0x00-0xFF)
        if address == SFR_PORTA_ADDR:
            self.handle_porta_write()
        elif address == SFR_PORTB_ADDR:
            self.handle_portb_write()
        elif address == SFR_TRISA_ADDR: # Bank 1 address
            self.handle_trisa_write()
        elif address == SFR_TRISB_ADDR: # Bank 1 address
            self.handle_trisb_write()
        elif address == SFR_PCL_ADDR:
            self.handle_pcl_write()
        elif address == SFR_TMR0_ADDR:
            self.handle_tmr0_write()
        elif address == SFR_OPTION_REG_ADDR: # Bank 1 address
            self.handle_option_write()
        elif address == SFR_EECON1_ADDR: # Bank 1 address
            self.handle_eecon1_write()
        # Add other handlers as needed (e.g., INTCON, PCLATH)

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
        # Read TRISA directly from its Bank 1 location
        return (self.ram[SFR_TRISA_ADDR] >> bit_pos) & 1

    def get_tris_b_bit(self, bit_pos):
        # Read TRISB directly from its Bank 1 location
        return (self.ram[SFR_TRISB_ADDR] >> bit_pos) & 1

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
        """Handles writing to PORTA - updates the latch and affects physical pins when configured as outputs."""
        porta_latch = self.ram[SFR_PORTA_ADDR] & 0x1F  # Only 5 bits available on PORTA
        # Need TRISA which is in Bank 1
        trisa = self.ram[SFR_TRISA_ADDR] # Read TRISA directly

        # For each pin configured as output, update the physical pin state to match latch
        for i in range(5):  # PORTA is only 5 bits (RA0-RA4)
            is_output = ((trisa >> i) & 1) == 0

            if i == 4: # RA4 open drain special handling
                if is_output:
                    if (porta_latch >> 4) & 1: # Latch is 1, pin is Hi-Z (do not drive)
                        # External pull-up would determine level, simulator leaves pin state alone
                        pass
                    else: # Latch is 0, drive low
                        self.porta_pins &= ~(1 << 4)
                # else: RA4 is input, pin state determined by external stimulus (self.porta_pins)
            else: # Normal pins RA0-RA3
                if is_output:
                    # Physical pin follows latch value
                    if (porta_latch >> i) & 1:
                        self.porta_pins |= (1 << i)  # Set pin high
                    else:
                        self.porta_pins &= ~(1 << i)  # Set pin low
                # else: Pin is input, state determined by external stimulus (self.porta_pins)

    def handle_portb_write(self):
        """Handles writing to PORTB - updates the latch and affects physical pins when configured as outputs."""
        portb_latch = self.ram[SFR_PORTB_ADDR]
        # Need TRISB which is in Bank 1
        trisb = self.ram[SFR_TRISB_ADDR] # Read TRISB directly

        # Update the latch value used for RB port change detection *before* potentially changing pins
        # Datasheet: "A read of PORTB reads the status of the pins, whereas a write to it will write to the port latch."
        # The interrupt flag RBIF is set when *any* enabled RB<7:4> input changes state.
        # The change is detected by comparing the current pin value with the value latched on the *previous read* of PORTB.
        # Writing to PORTB updates the latch, which affects outputs immediately. It does NOT directly trigger the interrupt.
        # self.portb_latch_on_read = portb_latch # This was incorrect, portb_latch_on_read is updated on READ

        # For each pin configured as output, update the physical pin state to match latch
        for i in range(8):
            if ((trisb >> i) & 1) == 0:  # If pin is output (TRISB bit = 0)
                # Physical pin follows latch value
                if (portb_latch >> i) & 1:
                    self.portb_pins |= (1 << i)  # Set pin high
                else:
                    self.portb_pins &= ~(1 << i)  # Set pin low
            # else: Pin is input, state determined by external stimulus (self.portb_pins)

        # Writing to PORTB does not trigger check_rb_port_change directly.
        # The check happens when PORTB is read or when an input pin changes state.

    # --- Add these methods to ensure TRIS changes update pin states ---
    def handle_trisa_write(self):
        """Handles writing to TRISA - changes pin direction and immediately affects pin state for output pins."""
        trisa = self.ram[SFR_TRISA_ADDR] & 0x1F  # Only 5 bits implemented
        porta_latch = self.ram[SFR_PORTA_ADDR] & 0x1F

        for i in range(5):
            is_output = ((trisa >> i) & 1) == 0

            if i == 4: # RA4 open drain special handling
                if is_output:
                    if (porta_latch >> 4) & 1: # Latch is 1, pin is Hi-Z (do not drive)
                         pass # Pin state determined by external factors
                    else: # Latch is 0, drive low
                        self.porta_pins &= ~(1 << 4)
                # else: RA4 becomes input, stop driving (pin state determined by self.porta_pins)
            else: # Normal pins RA0-RA3
                if is_output:
                    # Pin becomes output, drive latch value onto pin
                    if (porta_latch >> i) & 1:
                        self.porta_pins |= (1 << i)
                    else:
                        self.porta_pins &= ~(1 << i)
                # else: Pin becomes input, stop driving (pin state determined by self.porta_pins)

    def handle_trisb_write(self):
        """Handles writing to TRISB - changes pin direction and immediately affects pin state for output pins."""
        trisb = self.ram[SFR_TRISB_ADDR]
        portb_latch = self.ram[SFR_PORTB_ADDR]

        for i in range(8):
            if ((trisb >> i) & 1) == 0:  # If pin becomes output (TRISB bit = 0)
                # Drive latch value onto pin
                if (portb_latch >> i) & 1:
                    self.portb_pins |= (1 << i)
                else:
                    self.portb_pins &= ~(1 << i)
            # else: Pin becomes input, stop driving (pin state determined by self.portb_pins)

        # Writing to TRISB might cause an RB port change condition if an output pin's
        # driven value differs from the value previously latched during a PORTB read.
        # However, the interrupt flag RBIF is set based on changes on *inputs*.
        # Let's call check_rb_port_change() here for simplicity, although datasheet implies
        # it's more related to input changes vs last read. Re-reading PORTB after TRISB write
        # would be the most accurate trigger.
        self.check_rb_port_change()

    def handle_option_write(self):
        self.prescaler_counter = 0  # Simplification: Clear prescaler on OPTION write

    def handle_pcl_write(self, is_computed_goto=False):
        """Handles writing to PCL."""
        pclath_val = self.ram[SFR_PCLATH_ADDR]

        if is_computed_goto:
            # For computed GOTOs (e.g., ADDWF PCL, F).
            # The value written to PCL is (PC<7:0> + W) & 0xFF.
            # However, the PC used here is the *incremented* PC (address of the *next* instruction).
            pc_low_byte = (self.pc + 1) & 0xFF # Use PC+1 for calculation
            intended_pcl = (pc_low_byte + self.w_reg) & 0xFF # Use correct attribute w_reg

            # The instruction writes this 'intended_pcl' value into RAM[SFR_PCL_ADDR].
            # This is crucial because some programs might rely on reading PCL after
            # a computed GOTO and expect this calculated value, even if the simulator might
            # read the PCL value incorrectly (e.g., read RAM[PCL] before PC increment).
            self.ram[SFR_PCL_ADDR] = intended_pcl
            pcl = intended_pcl # Use the recalculated value for PC update

            # Debug print (optional) - Corrected typo from self.w to self.w_reg
            # print(f"Computed GOTO: Recalculated PCL = 0x{pcl:02X} (PC_low={pc_low_byte:02X}, W={self.w_reg:02X})")

            # Now calculate the new PC using PCLATH<4:0> and the correct PCL value
            self.pc = ((pclath_val & 0x1F) << 8) | pcl
            # print(f"Computed GOTO: New PC = 0x{self.pc:03X} (PCL={pcl:02X}, PCLATH={pclath_val:02X})") # Original print
        else:
            # For direct writes (e.g., MOVWF PCL).
            # Assume the value written to RAM[SFR_PCL_ADDR] by the instruction is correct.
            pcl = self.ram[SFR_PCL_ADDR]
            self.pc = ((pclath_val & 0x1F) << 8) | pcl
            # print(f"PCL Direct Write: New PC = 0x{self.pc:03X} (PCL={pcl:02X}, PCLATH={pclath_val:02X})") # Original print

        self.pc &= 0x1FFF # Ensure PC is within 13 bits

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

    def handle_eecon2_write(self, value):
        if self.eecon2_write_step == 0 and value == 0x55:
            self.eecon2_write_step = 1
            print("EECON2 Write: Step 1 (0x55) received.")
        elif self.eecon2_write_step == 1 and value == 0xAA:
            self.eecon2_write_step = 2
            print("EECON2 Write: Step 2 (0xAA) received.")
        else:
            self.eecon2_write_step = 0
            print(f"EECON2 Write: Invalid sequence value 0x{value:02X} received.")
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

    def update_timer0(self, cycles=1):
        """Update Timer0 based on configuration and elapsed cycles."""
        # Get OPTION register (Bank 1, address 0x01)
        option_reg = self.get_ram(0x81)
        
        # Check which clock source to use
        t0cs = (option_reg >> 5) & 1  # T0CS bit (bit 5)
        
        if t0cs == 0:  # Internal clock mode (instruction cycle)
            t0se = 0  # Not used in internal clock mode
            psa = (option_reg >> 3) & 1  # PSA bit (bit 3)
            ps = option_reg & 0x07  # PS2:PS0 bits (bits 0-2)
            
            # Calculate prescaler value
            if psa == 0:  # Prescaler assigned to Timer0
                prescaler_values = {0: 2, 1: 4, 2: 8, 3: 16, 4: 32, 5: 64, 6: 128, 7: 256}
                prescaler_value = prescaler_values[ps]
                
                # Update prescaler counter
                self.prescaler_counter += cycles
                if self.prescaler_counter >= prescaler_value:
                    # Increment TMR0
                    increments = self.prescaler_counter // prescaler_value
                    self.prescaler_counter %= prescaler_value
                    
                    tmr0 = self.get_ram(0x01)  # TMR0 register
                    tmr0 = (tmr0 + increments) & 0xFF
                    self.set_ram(0x01, tmr0)
            else:  # Prescaler assigned to WDT
                # In this case, Timer0 is incremented on every instruction cycle
                tmr0 = self.get_ram(0x01)  # TMR0 register
                tmr0 = (tmr0 + cycles) & 0xFF
                self.set_ram(0x01, tmr0)
        else:  # External clock mode (RA4/T0CKI pin)
            # In external clock mode, Timer0 is handled when toggle_porta_pin is called
            pass
    
    def toggle_porta_pin(self, pin_index, value=None):
        """Toggle or set a specific pin on PORTA."""
        if pin_index < 0 or pin_index > 4:  # PORTA has 5 pins (RA0-RA4)
            return
        
        porta = self.get_ram(0x05)  # PORTA register
        trisa = self.get_ram(0x85)  # TRISA register
        
        # Check if the pin is configured as input
        if (trisa & (1 << pin_index)) == 0:
            return  # Pin is output, can't toggle externally
            
        old_value = (porta >> pin_index) & 1
        
        if value is None:
            # Toggle the pin
            new_value = 1 - old_value
        else:
            # Set to specified value
            new_value = 1 if value else 0
            
        # If it's already the desired value, do nothing
        if old_value == new_value:
            return
            
        # Set the pin value
        if new_value:
            porta |= (1 << pin_index)
        else:
            porta &= ~(1 << pin_index)
            
        self.set_ram(0x05, porta)
        
        # Handle Timer0 if this is RA4 (T0CKI pin) and Timer0 is in external clock mode
        if pin_index == 4:
            option_reg = self.get_ram(0x81)
            
            # Check if Timer0 is in external clock mode
            t0cs = (option_reg >> 5) & 1
            if t0cs == 1:  # External clock mode
                # Check T0SE bit to determine triggering edge
                t0se = (option_reg >> 4) & 1  # T0SE bit (bit 4)
                
                # Only trigger on the specified edge (falling edge if T0SE=1)
                edge_triggered = (t0se == 1 and old_value == 1 and new_value == 0) or \
                               (t0se == 0 and old_value == 0 and new_value == 1)
                               
                if edge_triggered:
                    # Check if prescaler is assigned to Timer0
                    psa = (option_reg >> 3) & 1
                    ps = option_reg & 0x07
                    
                    if psa == 0:  # Prescaler assigned to Timer0
                        # Increment prescaler counter
                        self.prescaler_counter += 1
                        prescaler_values = {0: 2, 1: 4, 2: 8, 3: 16, 4: 32, 5: 64, 6: 128, 7: 256}
                        prescaler_value = prescaler_values[ps]
                        
                        if self.prescaler_counter >= prescaler_value:
                            # Reset counter and increment TMR0
                            self.prescaler_counter = 0
                            tmr0 = self.get_ram(0x01)
                            tmr0 = (tmr0 + 1) & 0xFF
                            self.set_ram(0x01, tmr0)
                    else:  # No prescaler for Timer0
                        # Directly increment TMR0
                        tmr0 = self.get_ram(0x01)
                        tmr0 = (tmr0 + 1) & 0xFF
                        self.set_ram(0x01, tmr0)

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

        # Special case for opcode 0x090D - COMF 0x0D, W
        elif opcode == 0x090D:
            print(f"PC=0x{self.pc:03X}: COMF 0x0D, W (Special case)")
            val_f = self.get_ram(0x0D)  # Get value from file register 0x0D
            result = (~val_f) & 0xFF    # Complement (invert all bits)
            zero = result == 0
            self.update_status_flags(zero=zero)  # Only Z affected
            self.w_reg = result         # Store result in W (dest=0)

        # Byte-oriented file register operations (00 xxxx ....) - Need to check both 000 and 001 patterns
        elif (opcode >> 12) == 0b00:  # Check only top 2 bits instead of 3
            op_sub = (opcode >> 8) & 0b1111  # Bits 11:8 determine specific instruction

            if opcode == 0b00000001100100:  # CLRWDT
                print(f"PC=0x{self.pc:03X}: CLRWDT")
                # 00h -> WDT, 0 -> WDT prescaler, 1 -> TO, 1 -> PD
                status = self.ram[SFR_STATUS_ADDR]
                status |= (1 << STATUS_TO) | (1 << STATUS_PD)
                self.ram[SFR_STATUS_ADDR] = status
                # Reset WDT counter and elapsed time
                self.wdt_counter = 0
                self.wdt_elapsed_us = 0.0 # Ensure elapsed time is reset
                # Reset prescaler if assigned to WDT
                option_reg = self.get_ram(0x81)
                psa = (option_reg >> 3) & 1  # PSA bit
                if psa == 1:  # If prescaler assigned to WDT
                    self.prescaler_counter = 0
            elif opcode == 0b00000001100011:  # SLEEP
                print(f"PC=0x{self.pc:03X}: SLEEP")
                # 00h -> WDT, 0 -> WDT prescaler, 1 -> TO, 0 -> PD
                status = self.ram[SFR_STATUS_ADDR]
                status &= ~(1 << STATUS_PD)
                self.ram[SFR_STATUS_ADDR] = status
                # Only enter sleep if not already in it
                if not self.sleep_mode:
                    self.sleep_mode = True
                    self.running = False  # Halt simulation for SLEEP
                    print("SLEEP instruction executed. Entering sleep mode.")
                pc_increment = False  # Don't increment PC when entering sleep
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
            elif opcode == 0x0009:  # Todo keine Ahnung ob das so stimmt
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
                error_msg = f"Unknown or unsupported opcode: 0x{opcode:04X} at PC=0x{self.pc:03X}"
                print(f"ERROR: {error_msg}")
                raise ValueError(error_msg)

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
            error_msg = f"Unknown or unsupported opcode: 0x{opcode:04X} at PC=0x{self.pc:03X}"
            print(f"ERROR: {error_msg}")
            raise ValueError(error_msg)

        # Increment PC for next instruction (if not already handled by branch/skip/call/return)
        if pc_increment:
            self.pc = (self.pc + 1) & 0x1FFF  # Mask to 13 bits

        return cycles

    def reset(self, por=True):
        """Resets the simulator state."""
        print("--- RESET ---", "POR" if por else "MCLR")
        self.pc = 0
        self.w_reg = 0
        self.stack = [0] * 8
        self.stack_ptr = 0
        self.runtime_cycles = 0
        self.laufzeit_us = 0.0  # Reset runtime counter
        self.eecon2_write_step = 0
        self.prescaler_counter = 0
        self.tmr0_inhibit_cycles = 0
        self.porta_pins = 0
        self.portb_pins = 0
        self.portb_latch_on_read = 0
        self.wdt_counter = 0  # Reset WDT counter
        self.wdt_elapsed_us = 0.0  # Ensure WDT elapsed time is reset
        self.sleep_mode = False  # Exit sleep mode on reset

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

    def toggle_porta_pin(self, pin_index):
        """Toggles the simulated input level of a PORTA pin. Returns True if successful."""
        if self.get_tris_a_bit(pin_index):  # Only toggle if pin is configured as input
            self.porta_pins ^= (1 << pin_index)
            pin_level = (self.porta_pins >> pin_index) & 1
            print(f"Toggled RA{pin_index} input stimulus to {pin_level}")
            return True
        return False

    def toggle_portb_pin(self, pin_index):
        """Toggles the simulated input level of a PORTB pin. Returns True if successful."""
        if self.get_tris_b_bit(pin_index):  # Only toggle if pin is configured as input
            prev_level = (self.portb_pins >> pin_index) & 1
            self.portb_pins ^= (1 << pin_index)
            current_level = (self.portb_pins >> pin_index) & 1
            print(f"Toggled RB{pin_index} input stimulus to {current_level}")
            
            # Check for RB0/INT edge if applicable
            if pin_index == 0:
                option = self.ram[SFR_OPTION_REG_ADDR] if self.get_status_bit(STATUS_RP0) == 1 else 0xFF
                intedg = (option >> OPTION_INTEDG) & 1
                is_rising_edge = prev_level == 0 and current_level == 1
                is_falling_edge = prev_level == 1 and current_level == 0
                
                if (intedg == 1 and is_rising_edge) or (intedg == 0 and is_falling_edge):
                    if not self.get_intcon_bit(INTCON_INTF):
                        self.set_intcon_bit(INTCON_INTF)
                        print("INT/RB0 edge detected, INTF set.")
            
            # Check for port change if RB<7:4>
            if pin_index >= 4:
                self.check_rb_port_change()
            
            return True
        return False

    def check_rb_port_change(self):
        """Checks if RB<7:4> pins have changed and sets RBIF if so."""
        # Only check if RBIE is enabled 
        if not self.get_intcon_bit(INTCON_RBIE):
            return
        
        # Get current values for RB7:4
        current_rb_high = (self.portb_pins >> 4) & 0x0F
        latch_rb_high = (self.portb_latch_on_read >> 4) & 0x0F
        
        # If any bit is different from last read
        if current_rb_high != latch_rb_high:
            self.set_intcon_bit(INTCON_RBIF)
            print(f"RB Port Change detected. RBIF set. Current:{current_rb_high:04b} vs Latch:{latch_rb_high:04b}")

    def update_wdt(self, cycles=1):
        """Update the Watchdog Timer counter and check for timeout."""
        if not self.wdt_enabled:
            return False  # WDT disabled, no timeout
        
        # Convert instruction cycles to microseconds
        dt_us = cycles * (4.0 / self.frequency_mhz)
        
        # Check if WDT prescaler is assigned
        option_reg = self.get_ram(0x81)
        psa = (option_reg >> 3) & 1
        ps = option_reg & 0x07
        
        # If prescaler is assigned to WDT (PSA=1), use it to scale the cycles
        if psa == 1:  # Prescaler assigned to WDT
            prescaler_values = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16, 5: 32, 6: 64, 7: 128}
            dt_us *= prescaler_values[ps]
        
        self.wdt_elapsed_us += dt_us
        
        # Check for timeout - exactly 18ms (18000µs)
        if self.wdt_elapsed_us >= self.wdt_timeout_us:
            # WDT timed out - reset counter
            self.wdt_elapsed_us = 0.0
            
            # WDT timeout clears the TO bit
            self.ram[SFR_STATUS_ADDR] &= ~(1 << STATUS_TO)
            
            # Notify GUI if callback is set
            if self.on_wdt_timeout:
                self.on_wdt_timeout()
                
            # If in sleep mode, wake up 
            if self.sleep_mode:
                self.sleep_mode = False
                self.running = True
                # Increment PC when waking up from sleep
                self.pc = (self.pc + 1) & 0x1FFF
                print("WDT timeout detected during SLEEP - waking up processor")
                return True  # Indicate timeout occurred
                
            # Otherwise cause a reset if not in sleep mode
            if not self.step_mode:  # Don't auto-reset in step mode
                print("WDT timeout detected - resetting processor")
                self.reset(por=False)  # WDT reset (not power-on)
                return True
                
        return False  # No timeout occurred