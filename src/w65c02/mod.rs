pub mod instruction;
use crate::w65c02::instruction::{W65C02OpDecode, AddressingMode};

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
    fn lda(&mut self, mode: AddressingMode) {
        mode.decode(self);
        self.ra = self.data;
    }
}

impl W65C02OpDecode for Processor {}
