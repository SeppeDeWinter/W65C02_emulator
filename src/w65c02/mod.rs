pub mod instruction;
use crate::w65c02::instruction::{W65C02OpDecode, ProcessorStatus, AddressingMode};

pub struct Processor {
    ra: u8,                 // Accumulator
    rx: u8,                 // X Index Register
    ry: u8,                 // Y Index Register
    sp: u8,                 // Stack Pointer
    pc: u16,                // Program Counter
    p: u8,                  // Processor Status
    rwb: bool,              // Read/Write
    address: u16,           // Address
    data: u8,               // Data
    memory: [u8; 0xFFFF]    // 65k of memory
}

impl Processor {
    /// Read data from the address.
    /// the processor's data is set to the data at the address in memory
    fn read_data_from_address(&mut self) {
        self.rwb = true;
        self.data = self.memory[self.address as usize]
    }
    /// Read the address from the address.
    /// the processor's address is set to the data at the address in memory.
    /// Addresses are stored in little endian format.
    fn read_address_from_address(&mut self) {
        self.rwb = true;
        // read low order bits
        self.read_data_from_address();
        let mut new_address = self.data as u16;                         // 0x00nn
        // read high order bits
        self.address += 1;
        self.read_data_from_address();
        //  0x00nn | 0xnn00 = 0xnnnn
        new_address = new_address | ((self.data as u16) << 8);
        self.address = new_address;
    }
    /// Read the zero page address from the address.
    /// the processor's address is set to the data at the address in memory.
    /// the high order bits are set to zero.
    fn read_zero_page_address_from_address(&mut self) {
        self.rwb = true;
        self.read_data_from_address();
        self.address = self.data as u16;
    }
    /// Write data to the address.
    fn write_data_to_address(&mut self) {
        self.rwb = false;
        self.memory[self.address as usize] = self.data;
    }
    /// Set processor flag.
    fn set_processor_status_flag(&mut self, flag: ProcessorStatus) {
        self.p = self.p | flag;
    }
    /// Clear processor flag.
    fn clear_processor_status_flag(&mut self, flag: ProcessorStatus) {
        self.p = self.p & !flag;
    }
    /// Set register value.
    fn set_ra(&mut self) {
        self.ra = self.data;
        // set processor status flags
        if self.ra == 0 {
            self.set_processor_status_flag(ProcessorStatus::Zero);
        } else {
            self.clear_processor_status_flag(ProcessorStatus::Zero);
        }
        if (self.ra as i8)< 0 {
            self.set_processor_status_flag(ProcessorStatus::Negative);
        } else {
            self.clear_processor_status_flag(ProcessorStatus::Negative);
        }
    }
    fn set_rx(&mut self) {
        self.rx = self.data;
        // set processor status flags
        if self.rx == 0 {
            self.set_processor_status_flag(ProcessorStatus::Zero);
        } else {
            self.clear_processor_status_flag(ProcessorStatus::Zero);
        }
        if (self.rx as i8) < 0 {
            self.set_processor_status_flag(ProcessorStatus::Negative);
        } else {
            self.clear_processor_status_flag(ProcessorStatus::Negative);
        }
    }
    fn set_ry(&mut self) {
        self.ry = self.data;
        // set processor status flags
        if self.ry == 0 {
            self.set_processor_status_flag(ProcessorStatus::Zero);
        } else {
            self.clear_processor_status_flag(ProcessorStatus::Zero);
        }
        if (self.ry as i8) < 0 {
            self.set_processor_status_flag(ProcessorStatus::Negative);
        } else {
            self.clear_processor_status_flag(ProcessorStatus::Negative);
        }
    }
    fn push_data_on_stack(&mut self) {
        self.address = self.sp as u16;
        self.write_data_to_address();
        self.sp -= 1;
    }
    fn pull_data_from_stack(&mut self) {
        self.sp += 1;
        self.address = self.sp as u16;
        self.read_data_from_address();
    }
    fn fetch_data(&mut self, mode: &AddressingMode) {
        match mode {
            AddressingMode::Implied => {},
            AddressingMode::Accumulator => {},
            AddressingMode::Immediate => {
                /*
                    Data is contained in instruction
                */
                self.address = self.pc + 1;   // Data is next byte
                self.read_data_from_address();
            },
            AddressingMode::Absolute => {
                /*  
                    The second and third bytes of the instruction
                    form a pointer to the data, and is stored in
                    little endian format.
                */ 
                self.address = self.pc + 1;
                self.read_address_from_address();
                self.read_data_from_address();
            },
            AddressingMode::AbsoluteXIndexed => {
                /*  
                    The second and third bytes of the instruction
                    form a pointer to the data, and is stored in
                    little endian format.
                    This pointer is offset by the X register.
                */ 
                self.address = self.pc + 1;
                self.read_address_from_address();
                self.address += self.rx as u16;
                self.read_data_from_address();
            },
            AddressingMode::AbsoluteYIndexed => {
                /*  
                    The second and third bytes of the instruction
                    form a pointer to the data, and is stored in
                    little endian format.
                    This pointer is offset by the X register.
                */ 
                self.address = self.pc + 1;
                self.read_address_from_address();
                self.address += self.ry as u16;
                self.read_data_from_address();
            },
            AddressingMode::AbsoluteIndirect => {
                /*  
                    The second and third byte of the instruction
                    specify a memory location.
                    The contents of this memory location is the low order byte of the effective address.
                    The next memory location contains the high order byte of the effective address.
                */ 
                self.address = self.pc + 1;
                self.read_address_from_address();
                self.read_address_from_address();
                self.read_data_from_address();     // never used given that this mode is only used by JMP
            },
            AddressingMode::AbsoluteXIndexedIndirect => {
                /*
                    The second and third byte of the instruction
                    specify a memory location. This pointer is offset by the X register.
                    The contents of this memory location (which is offset by the X register)
                    is the low order byte of the effective address.
                    The next memory location contains the high order byte of the effective address.
                */
                self.address = self.pc + 1;
                self.read_address_from_address();
                self.address += self.rx as u16;
                self.read_address_from_address();
                self.read_data_from_address();     // never used given that this mode is only used by JMP
            },
            AddressingMode::ZeroPage => {
                /*  
                    The second byte of the instruction
                    forms a pointer to the data, the high order byte is zero.
                */ 
                self.address = self.pc + 1;
                self.read_zero_page_address_from_address();
                self.read_data_from_address();
            },
            AddressingMode::ZeroPageXIndexed => {
                /*  
                    The second byte of the instruction
                    forms a pointer to the data, the high order byte is zero.
                    This pointer is offset by the X register.
                */ 
                self.address = self.pc + 1;
                self.read_zero_page_address_from_address();
                self.address += self.rx as u16;
                self.read_data_from_address();
            },
            AddressingMode::ZeroPageYIndexed => {
                /*  
                    The second byte of the instruction
                    forms a pointer to the data, the high order byte is zero.
                    This pointer is offset by the Y register.
                */ 
                self.address = self.pc + 1;
                self.read_zero_page_address_from_address();
                self.address += self.ry as u16;
                self.read_data_from_address();
            },
            AddressingMode::ZeroPageIndirect => {
                /*  
                    The second byte of the instruction
                    forms a zero page pointer that points to the low order byte of the effective address.
                */
                self.address = self.pc + 1;
                self.read_zero_page_address_from_address();
                self.read_address_from_address();
                self.read_data_from_address();
            },
            AddressingMode::ZeroPageXIndexedIndirect => {
                /*
                    The second byte of the instruction
                    forms a zero page pointer that points to the low order byte of the effective address.
                    This pointer is offset by the X register.
                */
                self.address = self.pc + 1;
                self.read_zero_page_address_from_address();
                self.address += self.rx as u16;
                self.read_address_from_address();
                self.read_data_from_address();
            },
            AddressingMode::ZeroPageIndirectIndexedY => {
                /*
                    The second byte of the instruction
                    forms a zero page pointer that points to the low order byte of the effective address.
                    This effective address is offset by the Y register.
                */
                self.address = self.pc + 1;
                self.read_zero_page_address_from_address();
                self.read_address_from_address();
                self.address += self.ry as u16;
                self.read_data_from_address();
            },
            AddressingMode::Relative => {
                /*
                    The second byte of the instruction
                    is a signed offset (one's complement) from the program counter.
                */
                self.address = self.pc + 1;
                self.read_data_from_address();
            }
        }
    }
}

impl W65C02OpDecode for Processor {}
