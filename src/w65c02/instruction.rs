use super::Processor;

#[derive(Copy, Clone)]
pub enum ProcessorStatus {
    Carry       =   0b00000001,
    Zero        =   0b00000010,
    IRQDBdis    =   0b00000100,
    Decimal     =   0b00001000,
    Break       =   0b00010000,
    Overflow    =   0b01000000,
    Negative    =   0b10000000,
}

impl std::ops::BitAnd<u8> for ProcessorStatus {
    type Output = u8;
    fn bitand(self, rhs: u8) -> Self::Output {
        (self as u8) & rhs 
    }
}

impl std::ops::BitAnd<ProcessorStatus> for u8 {
    type Output = u8;
    fn bitand(self, rhs: ProcessorStatus) -> Self::Output {
        self & (rhs as u8)
    }
}

impl std::ops::BitOr<u8> for ProcessorStatus {
    type Output = u8;
    fn bitor(self, rhs: u8) -> Self::Output {
        (self as u8) | rhs 
    }
}

impl std::ops::BitOr<ProcessorStatus> for u8 {
    type Output = u8;
    fn bitor(self, rhs: ProcessorStatus) -> Self::Output {
        self | (rhs as u8)
    }
}

impl std::ops::Not for ProcessorStatus {
    type Output = u8;
    fn not(self) -> Self::Output {
        !(self as u8)
    }
}

impl PartialEq<ProcessorStatus> for u8 {
    fn eq(&self, other: &ProcessorStatus) -> bool {
        other == self
    }
}

impl PartialEq<u8> for ProcessorStatus {
    fn eq(&self, other: &u8) -> bool {
        self == other
    }
}
#[derive(Debug)]
pub enum AddressingMode {
    Implied,
    Accumulator,                // A
    Immediate,                  // #$nn
    Absolute,                   // $nnnn
    AbsoluteXIndexed,           // $nnnn,X
    AbsoluteYIndexed,           // $nnnn,Y
    AbsoluteIndirect,           // ($nnnn)
    AbsoluteXIndexedIndirect,   // ($nnnn,X)
    ZeroPage,                   // $nn
    ZeroPageXIndexed,           // $nn,X
    ZeroPageYIndexed,           // $nn,Y
    ZeroPageIndirect,           // ($nn)
    ZeroPageXIndexedIndirect,   // ($nn,X)
    ZeroPageIndirectIndexedY,   // ($nn),Y
    Relative,                   // $nnnn
}

impl AddressingMode {
    fn instruction_size(self) -> u16 {
        match self {
            AddressingMode::Implied                     => 1,
            AddressingMode::Accumulator                 => 1,
            AddressingMode::Immediate                   => 2,
            AddressingMode::Absolute                    => 3,
            AddressingMode::AbsoluteXIndexed            => 3,
            AddressingMode::AbsoluteYIndexed            => 3,
            AddressingMode::AbsoluteIndirect            => 3,
            AddressingMode::AbsoluteXIndexedIndirect    => 3,
            AddressingMode::ZeroPage                    => 2,
            AddressingMode::ZeroPageXIndexed            => 2,
            AddressingMode::ZeroPageYIndexed            => 2,
            AddressingMode::ZeroPageIndirect            => 2,
            AddressingMode::ZeroPageXIndexedIndirect    => 2,
            AddressingMode::ZeroPageIndirectIndexedY    => 2,
            AddressingMode::Relative                    => 2,
        }
    }
}
#[derive(Debug)]
pub enum Instruction {
    LDA(AddressingMode),        // Load accumulator from memory
    LDX(AddressingMode),        // Load X from memory
    LDY(AddressingMode),        // Load Y from memory
    STA(AddressingMode),        // Store accumulator in memory
    STX(AddressingMode),        // Store X in memory
    STY(AddressingMode),        // Store Y in memory
    STZ(AddressingMode),        // Store zero in memory
    TAX(AddressingMode),        // Transfer accumulator to X
    TAY(AddressingMode),        // Transfer accumulator to Y
    TSX(AddressingMode),        // Transfer stack pointer to X
    TXA(AddressingMode),        // Transfer X to accumulator
    TXS(AddressingMode),        // Transfer X to stack pointer
    TYA(AddressingMode),        // Transfer Y to accumulator
    PHA(AddressingMode),        // Push accumulator on stack
    PHP(AddressingMode),        // Push processor status on stack
    PHX(AddressingMode),        // Push X on stack
    PHY(AddressingMode),        // Push Y on stack
    PLA(AddressingMode),        // Pull accumulator from stack
    PLP(AddressingMode),        // Pull processor status from stack
    PLX(AddressingMode),        // Pull X from stack
    PLY(AddressingMode),        // Pull Y from stack
    ASL(AddressingMode),        // Arithmetic shift left
    LSR(AddressingMode),        // Logical shift right
    ROL(AddressingMode),        // Rotate left
    ROR(AddressingMode),        // Rotate right
    AND(AddressingMode),        // AND memory with accumulator
    BIT(AddressingMode),        // Test bits in memory with accumulator
    EOR(AddressingMode),        // Exclusive OR memory with accumulator
    ORA(AddressingMode),        // OR memory with accumulator
    TRB(AddressingMode),        // Test and reset bits in memory
    TSB(AddressingMode),        // Test and set bits in memory
    ADC(AddressingMode),        // Add memory to accumulator with carry
    CMP(AddressingMode),        // Compare memory and accumulator
    CPX(AddressingMode),        // Compare memory and X
    CPY(AddressingMode),        // Compare memory and Y
    SBC(AddressingMode),        // Subtract memory from accumulator with borrow
    DEC(AddressingMode),        // Decrement memory by one
    DEX(AddressingMode),        // Decrement X by one
    DEY(AddressingMode),        // Decrement Y by one
    INC(AddressingMode),        // Increment memory by one
    INX(AddressingMode),        // Increment X by one
    INY(AddressingMode),        // Increment Y by one
    BRA(AddressingMode),        // Branch always
    BRK(AddressingMode),        // Force break
    JMP(AddressingMode),        // Jump
    JSR(AddressingMode),        // Jump to subroutine
    RTI(AddressingMode),        // Return from interrupt
    RTS(AddressingMode),        // Return from subroutine
    BCC(AddressingMode),        // Branch on carry clear
    BCS(AddressingMode),        // Branch on carry set
    BEQ(AddressingMode),        // Branch on result zero
    BMI(AddressingMode),        // Branch on result minus
    BNE(AddressingMode),        // Branch on result not zero
    BPL(AddressingMode),        // Branch on result plus
    BVC(AddressingMode),        // Branch on overflow clear
    BVS(AddressingMode),        // Branch on overflow set
    CLC(AddressingMode),        // Clear carry flag
    CLD(AddressingMode),        // Clear decimal mode
    CLI(AddressingMode),        // Clear interrupt disable bit
    CLV(AddressingMode),        // Clear overflow flag
    SEC(AddressingMode),        // Set carry flag
    SED(AddressingMode),        // Set decimal mode
    SEI(AddressingMode),        // Set interrupt disable status
    NOP(AddressingMode),        // No operation
}
impl Instruction {
    pub fn execute(self, processor: &mut Processor) -> u16{
        match self {
            // Load operations
            Instruction::LDA(mode) => {
                processor.fetch_data(&mode);
                processor.set_ra();
                return mode.instruction_size();
            },
            Instruction::LDX(mode) => {
                processor.fetch_data(&mode);
                processor.set_rx();
                return mode.instruction_size();
            },
            Instruction::LDY(mode) => {
                processor.fetch_data(&mode);
                processor.set_ry();
                return mode.instruction_size();
            },
            Instruction::STA(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.ra;
                processor.write_data_to_address();
                return mode.instruction_size();
            },
            Instruction::STX(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.rx;
                processor.write_data_to_address();
                return mode.instruction_size();
            },
            Instruction::STY(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.ry;
                processor.write_data_to_address();
                return mode.instruction_size();
            },
            Instruction::STZ(mode) => {
                processor.fetch_data(&mode);
                processor.data = 0;
                processor.write_data_to_address();
                return mode.instruction_size();
            },

            // Transfer operations
            Instruction::TAX(mode) => {
                processor.data = processor.ra;
                processor.set_rx();
                return mode.instruction_size();
            },
            Instruction::TAY(mode) => {
                processor.data = processor.ra;
                processor.set_ry();
                return mode.instruction_size();
            },
            Instruction::TSX(mode) => {
                processor.data = processor.sp;
                processor.set_rx();
                return mode.instruction_size();
            },
            Instruction::TXA(mode) => {
                processor.data = processor.rx;
                processor.set_ra();
                return mode.instruction_size();
            },
            Instruction::TXS(mode) => {
                processor.sp = processor.rx;
                return mode.instruction_size();
            },
            Instruction::TYA(mode) => {
                processor.data = processor.ry;
                processor.set_ra();
                return mode.instruction_size();
            },

            // Stack operations
            Instruction::PHA(mode) => {
                processor.data = processor.ra;
                processor.push_data_on_stack();
                return mode.instruction_size();
            }
            Instruction::PHP(mode) => {
                processor.data = processor.p;
                processor.push_data_on_stack();
                return mode.instruction_size();
            },
            Instruction::PHX(mode) => {
                processor.data = processor.rx;
                processor.push_data_on_stack();
                return mode.instruction_size();
            },
            Instruction::PHY(mode) => {
                processor.data = processor.ry;
                processor.push_data_on_stack();
                return mode.instruction_size();
            },
            Instruction::PLA(mode) => {
                processor.pull_data_from_stack();
                processor.set_ra();
                return mode.instruction_size();
            },
            Instruction::PLP(mode) => {
                processor.pull_data_from_stack();
                processor.p = processor.data;
                return mode.instruction_size();
            },
            Instruction::PLX(mode) => {
                processor.pull_data_from_stack();
                processor.set_rx();
                return mode.instruction_size();
            },
            Instruction::PLY(mode) => {
                processor.pull_data_from_stack();
                processor.set_ry();
                return mode.instruction_size();
            },

            // Shift operations
            Instruction::ASL(mode) => {
                processor.fetch_data(&mode);
                let last_bit = 0b10000000 & processor.data;
                processor.data = processor.data << 1;
                if last_bit != 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                match mode {
                    AddressingMode::Accumulator => {
                        processor.set_ra();
                    },
                    _ => {
                        processor.write_data_to_address();
                        if processor.data == 0 {
                            processor.set_processor_status_flag(ProcessorStatus::Zero);
                        } else {
                            processor.clear_processor_status_flag(ProcessorStatus::Zero);
                        }
                        if (processor.data as i8) < 0 {
                            processor.set_processor_status_flag(ProcessorStatus::Negative);
                        } else {
                            processor.clear_processor_status_flag(ProcessorStatus::Negative); 
                        }
                    }
                }
                return mode.instruction_size();
            },
            Instruction::LSR(mode) => {
                processor.fetch_data(&mode);
                let first_bit = 0b00000001 & processor.data;
                processor.data = processor.data >> 1;
                if first_bit != 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                match mode {
                    AddressingMode::Accumulator => {
                        processor.set_ra();
                    },
                    _ => {
                        processor.write_data_to_address();
                        processor.clear_processor_status_flag(ProcessorStatus::Negative); // a 0 is shifted into the MSB
                        if processor.data == 0 {
                            processor.set_processor_status_flag(ProcessorStatus::Zero);
                        } else {
                            processor.clear_processor_status_flag(ProcessorStatus::Zero);
                        }
                    }
                }
                return mode.instruction_size();
            },
            Instruction::ROL(mode) => {
                /*
                    The rotate left instruction shifts either the accumulator or 
                    addressed memory left 1 bit, with the input carry being stored 
                    n bit 0 and with the input bit 7 being stored in the carry flags
                */
                processor.fetch_data(&mode);
                let last_bit = 0b10000000 & processor.data;
                processor.data = processor.data << 1;
                // this works because the carry flag is the 0th bit of the processor status register
                processor.data = processor.data | (processor.p & ProcessorStatus::Carry);
                if last_bit != 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                match mode {
                    AddressingMode::Accumulator => {
                        processor.set_ra();
                    },
                    _ => {
                        processor.write_data_to_address();
                        if processor.data == 0 {
                            processor.set_processor_status_flag(ProcessorStatus::Zero);
                        } else {
                            processor.clear_processor_status_flag(ProcessorStatus::Zero);
                        }
                        if (processor.data as i8) < 0 {
                            processor.set_processor_status_flag(ProcessorStatus::Negative);
                        } else {
                            processor.clear_processor_status_flag(ProcessorStatus::Negative); 
                        }
                    }
                }
                return mode.instruction_size();
            },
            Instruction::ROR(mode) => {
                /*
                    The rotate right instruction shifts either the accumulator or
                    addressed memory right 1 bit with bit 0 shifted into the carry
                    and carry shifted into bit 7.
                */
                processor.fetch_data(&mode);
                let first_bit = 0b00000001 & processor.data;
                processor.data = processor.data >> 1;
                processor.data = processor.data | ((processor.p & ProcessorStatus::Carry) << 7);
                if first_bit != 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                match mode {
                    AddressingMode::Accumulator => {
                        processor.set_ra();
                    },
                    _ => {
                        processor.write_data_to_address();
                        processor.clear_processor_status_flag(ProcessorStatus::Negative); // a 0 is shifted into the MSB
                        if processor.data == 0 {
                            processor.set_processor_status_flag(ProcessorStatus::Zero);
                        } else {
                            processor.clear_processor_status_flag(ProcessorStatus::Zero);
                        }
                    }
                }
                return mode.instruction_size();
            },

            // Logical operations
            Instruction::AND(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.ra & processor.data;
                processor.set_ra();
                return mode.instruction_size();
            },
            Instruction::BIT(mode) => {
                processor.fetch_data(&mode);
                let result = processor.ra & processor.data;
                if result == 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Zero);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Zero);
                }
                if (result as i8) < 8 {
                    processor.set_processor_status_flag(ProcessorStatus::Negative);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Negative);
                }
                if (result & 0b01000000) != 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Overflow);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Overflow);
                }
                return mode.instruction_size();
            },
            Instruction::EOR(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.ra ^ processor.data;
                processor.set_ra();
                return mode.instruction_size();
            },
            Instruction::ORA(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.ra | processor.data;
                processor.set_ra();
                return mode.instruction_size();
            },
            Instruction::TRB(mode) => {
                /*
                    This instruction tests and resets bits in memory, 
                    using the accumulator for both a test mask, and a reset mask. 
                    It performs a logical AND between the inverted bits of the 
                    accumulator and the bits in memory, storing the result back into 
                    memory.
                */
                processor.fetch_data(&mode);
                processor.data = !processor.ra & processor.data;
                processor.write_data_to_address();
                if processor.data == 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Zero);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Zero);
                }
                return mode.instruction_size();
            },
            Instruction::TSB(mode) => {
                /*
                    This instruction tests and sets bits in memory, 
                    using the accumulator for both a test mask, and a set mask. 
                    It performs a logical OR between the bits of the accumulator
                    and the bits in memory, storing the result back into memory.
                */
                processor.fetch_data(&mode);
                processor.data = processor.ra | processor.data;
                processor.write_data_to_address();
                if processor.data == 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Zero);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Zero);
                }
                return mode.instruction_size();
            },

            // Arithmetic operations
            Instruction::ADC(mode) => {
                /*
                    This instruction adds the value of memory and carry from
                    the previous operation to the value of the accumulator and
                    stores the result in the accumulator.
                */
                processor.fetch_data(&mode);
                let result = ( processor.ra as u16 )
                    .wrapping_add(  processor.data as u16 )
                    .wrapping_add(  processor.p as u16 & ProcessorStatus::Carry as u16 );
                processor.data = result as u8;
                processor.set_ra();
                if ( result > 0xFF ) |
                   ( result > 99 && ( processor.p & ProcessorStatus::Decimal != 0 ) ) {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                if (( result as i16 ) >  127) |
                   (( result as i16 ) < -128) {
                    processor.set_processor_status_flag(ProcessorStatus::Overflow);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Overflow);
                }
                return mode.instruction_size();
            },
            Instruction::CMP(mode) => {
                /*
                    This instruction subtracts the contents of memory from the 
                    contents of the accumulator.
                */
                processor.fetch_data(&mode);
                if processor.data <= processor.ra {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                let result = (processor.ra as i8).wrapping_sub(processor.data as i8);
                processor.data = result as u8;
                processor.set_ra();
                return mode.instruction_size();
            },
            Instruction::CPX(mode) => {
                /*
                    This instruction subtracts the value of the addressed
                    memory location from the content of index register X
                    using the adder but does not store the result; therefore,
                    its only use is to set the N, Z and C flags to allow for
                    comparison between the index register X and the value in memory.
                    
                    It causes the carry to be set on if the absolute value of the index
                    register X is equal to or greater than the data from memory.

                    If the results of the subtraction contain a bit 7, 
                    then the N flag is set, if not, it is reset. 
                    
                    If the value in memory is equal to the value in index register X,
                    the Z flag is set, otherwise it is reset.
                */
                processor.fetch_data(&mode);
                if processor.rx >= processor.data {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                let result = (processor.rx as i8).wrapping_sub(processor.data as i8);
                if (result as i8) < 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Negative);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Negative);
                }
                if result == 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Zero);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Zero);
                }
                return mode.instruction_size();
            },
            Instruction::CPY(mode) => {
                /*
                    This instruction subtracts the value of the addressed
                    memory location from the content of index register Y
                    using the adder but does not store the result; therefore,
                    its only use is to set the N, Z and C flags to allow for
                    comparison between the index register Y and the value in memory.
                    
                    It causes the carry to be set on if the absolute value of the index
                    register Y is equal to or greater than the data from memory.

                    If the results of the subtraction contain a bit 7, 
                    then the N flag is set, if not, it is reset. 
                    
                    If the value in memory is equal to the value in index register Y,
                    the Z flag is set, otherwise it is reset.
                */
                processor.fetch_data(&mode);
                if processor.ry >= processor.data {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                let result = (processor.ry as i8).wrapping_sub(processor.data as i8);
                if (result as i8) < 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Negative);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Negative);
                }
                if result == 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Zero);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Zero);
                }
                return mode.instruction_size();
            },
            Instruction::SBC(mode) => {
                /*
                    This instruction subtracts the value of memory and borrow from
                    the value of the accumulator, using two's complement arithmetic,
                    and stores the result in the accumulator. 
                    Borrow is defined as the carry flag complemented; therefore, a
                    resultant carry flag indicates that a borrow has not occurred.
                */
                processor.fetch_data(&mode);
                let c = (!processor.p & ProcessorStatus::Carry) as i16;
                let result = (processor.ra as i16)
                    .wrapping_sub(processor.data as i16)
                    .wrapping_sub(c);
                processor.data = result as u8;
                processor.set_ra();

                if result >= 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Carry);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Carry);
                }
                if result > 127 || result < -128 {
                    processor.set_processor_status_flag(ProcessorStatus::Overflow);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Overflow);
                }
                return mode.instruction_size();
            },

            // Increment and decrement operations
            Instruction::DEC(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.data.wrapping_sub(1);
                processor.write_data_to_address();
                if processor.data == 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Zero);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Zero);
                }
                if (processor.data as i8) < 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Negative);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Negative); 
                }
                return mode.instruction_size();
            },
            Instruction::DEX(mode) => {
                processor.data = processor.rx.wrapping_sub(1);
                processor.set_rx();
                return mode.instruction_size();
            },
            Instruction::DEY(mode) => {
                processor.data = processor.ry.wrapping_sub(1);
                processor.set_ry();
                return mode.instruction_size();
            },
            Instruction::INC(mode) => {
                processor.fetch_data(&mode);
                processor.data = processor.data.wrapping_add(1);
                processor.write_data_to_address();
                if processor.data == 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Zero);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Zero);
                }
                if (processor.data as i8) < 0 {
                    processor.set_processor_status_flag(ProcessorStatus::Negative);
                } else {
                    processor.clear_processor_status_flag(ProcessorStatus::Negative); 
                }
                return mode.instruction_size();
            },
            Instruction::INX(mode) => {
                processor.data = processor.rx.wrapping_add(1);
                processor.set_rx();
                return mode.instruction_size();
            },
            Instruction::INY(mode) => {
                processor.data = processor.ry.wrapping_add(1);
                processor.set_ry();
                return mode.instruction_size();
            },

            // Control flow operations
            Instruction::BRA(mode) => {
                processor.fetch_data(&mode);
                let offset = processor.data as i16;
                processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                return mode.instruction_size();
            },
            Instruction::BRK(mode) => {
                /*
                    The break command causes the microprocessor to go through an inter-­
                    rupt sequence under program control. This means that the program 
                    counter of the second byte after the BRK. is automatically stored
                    on the stack along with the processor status at the beginning of
                    the break instruction. The microprocessor then transfers control
                    to the interrupt vector.
                */
                processor.pc = processor.pc.wrapping_add(2);
                processor.push_pc_on_stack();
                // push processor status on stack
                processor.data = processor.p;
                processor.push_data_on_stack();
                // set interrupt disable flag
                processor.set_processor_status_flag(ProcessorStatus::IRQDBdis);
                processor.interrupt_vector();
                return mode.instruction_size();
            },
            Instruction::JMP(mode) => {
                processor.fetch_data(&mode);
                processor.pc = processor.address;
                return 0;
            },
            Instruction::JSR(mode) => {
                /*
                    This instruction transfers control of the program counter to a
                    subroutine location but leaves a return pointer on the stack to
                    allow the user to return to perform the next instruction in the
                    main program after the subroutine is complete.

                    To accomplish this, JSR instruction stores the program counter
                    address which points to the last byte of the jump instruc­tion
                    onto the stack using the stack pointer. The stack byte contains
                    the program count high first, followed by program count low.
                    The JSR then transfers the addresses following the jump
                    instruction to the program counter low and the program counter
                    high, thereby directing the program to begin at that new address.
                    
                    The JSR instruction affects no flags, causes the stack pointer to
                    be decremented by 2 and substitutes new values into the program
                    counter low and the program counter high.
                */
                processor.fetch_data(&mode);
                processor.pc = processor.pc.wrapping_add(2);
                processor.push_pc_on_stack();
                // push processor status on stack
                processor.data = processor.p;
                processor.push_data_on_stack();
                // set new address
                processor.pc = processor.address;
                return mode.instruction_size();
            },
            Instruction::RTI(mode) => {
                /*
                    This instruction transfers from the stack into the microprocessor
                    the processor status and the program counter location for the 
                    instruction which was interrupted. By virtue of the interrupt 
                    having stored this data before executing the instruction and 
                    the fact that the RTI reinitializes the microprocessor to the 
                    same state as when it was interrupted, the combination of 
                    interrupt plus RTI allows truly reentrant coding.
                    
                    The RTI instruction reinitializes all flags to the position to
                    the point they were at the time the interrupt was taken and sets
                    the program counter back to its pre-interrupt state. 
                    It affects no other registers in the microprocessor.
                */
                // pull processor status from stack
                processor.pull_data_from_stack();
                processor.p = processor.data;
                // pull program counter from stack
                processor.pull_pc_from_stack();
                return mode.instruction_size();
            },
            Instruction::RTS(mode) => {
                /*
                    This instruction loads the program count low and program count 
                    high from the stack into the program counter and increments the 
                    program counter so that it points to the instruction following 
                    the JSR. The stack pointer is adjusted by incrementing it twice.
                */
                processor.pull_pc_from_stack();
                processor.pc = processor.pc.wrapping_add(1);
                return mode.instruction_size();
            },
            Instruction::BCC(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Carry == 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },
            Instruction::BCS(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Carry != 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },
            Instruction::BEQ(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Zero != 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },
            Instruction::BMI(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Negative != 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },
            Instruction::BNE(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Zero == 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },
            Instruction::BPL(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Negative == 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },
            Instruction::BVC(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Overflow == 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },
            Instruction::BVS(mode) => {
                processor.fetch_data(&mode);
                if processor.p & ProcessorStatus::Overflow != 0 {
                    let offset = processor.data as i16;
                    processor.pc = (processor.pc as i16).wrapping_add(offset) as u16;
                }
                return mode.instruction_size();
            },

            // Status flag operations
            Instruction::CLC(mode) => {
                processor.clear_processor_status_flag(ProcessorStatus::Carry);
                return mode.instruction_size();
            },
            Instruction::CLD(mode) => {
                processor.clear_processor_status_flag(ProcessorStatus::Decimal);
                return mode.instruction_size();
            },
            Instruction::CLI(mode) => {
                processor.clear_processor_status_flag(ProcessorStatus::IRQDBdis);
                return mode.instruction_size();
            },
            Instruction::CLV(mode) => {
                processor.clear_processor_status_flag(ProcessorStatus::Overflow);
                return mode.instruction_size();
            },
            Instruction::SEC(mode) => {
                processor.set_processor_status_flag(ProcessorStatus::Carry);
                return mode.instruction_size();
            },
            Instruction::SED(mode) => {
                processor.set_processor_status_flag(ProcessorStatus::Decimal);
                return mode.instruction_size();
            },
            Instruction::SEI(mode) => {
                processor.set_processor_status_flag(ProcessorStatus::IRQDBdis);
                return mode.instruction_size();
            },

            // No operation
            Instruction::NOP(mode) => {
                return mode.instruction_size();
            }
        }
    }
}

pub trait W65C02OpDecode {
    fn op_decode(opcode: u8) -> Option<Instruction> {
        match opcode {
            0x00 => Some(Instruction::BRK(AddressingMode::Implied)),
            0x01 => Some(Instruction::ORA(AddressingMode::ZeroPageXIndexedIndirect)),
            0x02 => None,
            0x03 => None,
            0x04 => Some(Instruction::TSB(AddressingMode::ZeroPage)),
            0x05 => Some(Instruction::ORA(AddressingMode::ZeroPage)),
            0x06 => Some(Instruction::ASL(AddressingMode::ZeroPage)),
            0x07 => None,
            0x08 => Some(Instruction::PHP(AddressingMode::Implied)),
            0x09 => Some(Instruction::ORA(AddressingMode::Immediate)),
            0x0A => Some(Instruction::ASL(AddressingMode::Accumulator)),
            0x0B => None,
            0x0C => Some(Instruction::TSB(AddressingMode::Absolute)),
            0x0D => Some(Instruction::ORA(AddressingMode::Absolute)),
            0x0E => Some(Instruction::ASL(AddressingMode::Absolute)),
            0x0F => None,
            
            0x10 => Some(Instruction::BPL(AddressingMode::Relative)),
            0x11 => Some(Instruction::ORA(AddressingMode::ZeroPageIndirectIndexedY)),
            0x12 => Some(Instruction::ORA(AddressingMode::ZeroPageIndirect)),
            0x13 => None,
            0x14 => Some(Instruction::TRB(AddressingMode::ZeroPage)),
            0x15 => Some(Instruction::ORA(AddressingMode::ZeroPageXIndexed)),
            0x16 => Some(Instruction::ASL(AddressingMode::ZeroPageXIndexed)),
            0x17 => None,
            0x18 => Some(Instruction::CLC(AddressingMode::Implied)),
            0x19 => Some(Instruction::ORA(AddressingMode::AbsoluteYIndexed)),
            0x1A => Some(Instruction::INC(AddressingMode::Accumulator)),
            0x1B => None,
            0x1C => Some(Instruction::TRB(AddressingMode::Absolute)),
            0x1D => Some(Instruction::ORA(AddressingMode::AbsoluteXIndexed)),
            0x1E => Some(Instruction::ASL(AddressingMode::AbsoluteXIndexed)),
            0x1F => None,
            
            0x20 => Some(Instruction::JSR(AddressingMode::Absolute)),
            0x21 => Some(Instruction::AND(AddressingMode::ZeroPageXIndexedIndirect)),
            0x22 => None,
            0x23 => None,
            0x24 => Some(Instruction::BIT(AddressingMode::ZeroPage)),
            0x25 => Some(Instruction::AND(AddressingMode::ZeroPage)),
            0x26 => Some(Instruction::ROL(AddressingMode::ZeroPage)),
            0x27 => None,
            0x28 => Some(Instruction::PLP(AddressingMode::Implied)),
            0x29 => Some(Instruction::AND(AddressingMode::Immediate)),
            0x2A => Some(Instruction::ROL(AddressingMode::Accumulator)),
            0x2B => None,
            0x2C => Some(Instruction::BIT(AddressingMode::Absolute)),
            0x2D => Some(Instruction::AND(AddressingMode::Absolute)),
            0x2E => Some(Instruction::ROL(AddressingMode::Absolute)),
            0x2F => None,
            
            0x30 => Some(Instruction::BMI(AddressingMode::Relative)),
            0x31 => Some(Instruction::AND(AddressingMode::ZeroPageXIndexedIndirect)),
            0x32 => Some(Instruction::AND(AddressingMode::ZeroPageIndirect)),
            0x33 => None,
            0x34 => Some(Instruction::BIT(AddressingMode::ZeroPageXIndexed)),
            0x35 => Some(Instruction::AND(AddressingMode::ZeroPageXIndexed)),
            0x36 => Some(Instruction::ROL(AddressingMode::ZeroPageXIndexed)),
            0x37 => None,
            0x38 => Some(Instruction::SEC(AddressingMode::Implied)),
            0x39 => Some(Instruction::AND(AddressingMode::AbsoluteYIndexed)),
            0x3A => Some(Instruction::DEC(AddressingMode::Accumulator)),
            0x3B => None,
            0x3C => Some(Instruction::BIT(AddressingMode::AbsoluteXIndexed)),
            0x3D => Some(Instruction::AND(AddressingMode::AbsoluteXIndexed)),
            0x3E => Some(Instruction::ROL(AddressingMode::AbsoluteXIndexed)),
            0x3F => None,

            0x40 => Some(Instruction::RTI(AddressingMode::Implied)),
            0x41 => Some(Instruction::EOR(AddressingMode::ZeroPageXIndexedIndirect)),
            0x42 => None,
            0x43 => None,
            0x44 => None,
            0x45 => Some(Instruction::EOR(AddressingMode::ZeroPage)),
            0x46 => Some(Instruction::LSR(AddressingMode::ZeroPage)),
            0x47 => None,
            0x48 => Some(Instruction::PHA(AddressingMode::Implied)),
            0x49 => Some(Instruction::EOR(AddressingMode::Immediate)),
            0x4A => Some(Instruction::LSR(AddressingMode::Accumulator)),
            0x4B => None,
            0x4C => Some(Instruction::JMP(AddressingMode::Absolute)),
            0x4D => Some(Instruction::EOR(AddressingMode::Absolute)),
            0x4E => Some(Instruction::LSR(AddressingMode::Absolute)),
            0x4F => None,

            0x50 => Some(Instruction::BVC(AddressingMode::Relative)),
            0x51 => Some(Instruction::EOR(AddressingMode::ZeroPageIndirectIndexedY)),
            0x52 => Some(Instruction::EOR(AddressingMode::ZeroPageIndirect)),
            0x53 => None,
            0x54 => None,
            0x55 => Some(Instruction::EOR(AddressingMode::ZeroPageXIndexed)),
            0x56 => Some(Instruction::LSR(AddressingMode::ZeroPageXIndexed)),
            0x57 => None,
            0x58 => Some(Instruction::CLI(AddressingMode::Implied)),
            0x59 => Some(Instruction::EOR(AddressingMode::AbsoluteYIndexed)),
            0x5A => Some(Instruction::PHY(AddressingMode::Implied)),
            0x5B => None,
            0x5C => None,
            0x5D => Some(Instruction::EOR(AddressingMode::AbsoluteXIndexed)),
            0x5E => Some(Instruction::LSR(AddressingMode::AbsoluteXIndexed)),
            0x5F => None,

            0x60 => Some(Instruction::RTS(AddressingMode::Implied)),
            0x61 => Some(Instruction::ADC(AddressingMode::ZeroPageXIndexedIndirect)),
            0x62 => None,
            0x63 => None,
            0x64 => Some(Instruction::STZ(AddressingMode::ZeroPage)),
            0x65 => Some(Instruction::ADC(AddressingMode::ZeroPage)),
            0x66 => Some(Instruction::ROR(AddressingMode::ZeroPage)),
            0x67 => None,
            0x68 => Some(Instruction::PLA(AddressingMode::Implied)),
            0x69 => Some(Instruction::ADC(AddressingMode::Immediate)),
            0x6A => Some(Instruction::ROR(AddressingMode::Accumulator)),
            0x6B => None,
            0x6C => Some(Instruction::JMP(AddressingMode::AbsoluteIndirect)),
            0x6D => Some(Instruction::ADC(AddressingMode::Absolute)),
            0x6E => Some(Instruction::ROR(AddressingMode::Absolute)),
            0x6F => None,

            0x70 => Some(Instruction::BVS(AddressingMode::Relative)),
            0x71 => Some(Instruction::ADC(AddressingMode::ZeroPageIndirectIndexedY)),
            0x72 => Some(Instruction::ADC(AddressingMode::ZeroPageIndirect)),
            0x73 => None,
            0x74 => Some(Instruction::STZ(AddressingMode::ZeroPageXIndexed)),
            0x75 => Some(Instruction::ADC(AddressingMode::ZeroPageXIndexed)),
            0x76 => Some(Instruction::ROR(AddressingMode::ZeroPageXIndexed)),
            0x77 => None,
            0x78 => Some(Instruction::SEI(AddressingMode::Implied)),
            0x79 => Some(Instruction::ADC(AddressingMode::AbsoluteYIndexed)),
            0x7A => Some(Instruction::PLY(AddressingMode::Implied)),
            0x7B => None,
            0x7C => Some(Instruction::JMP(AddressingMode::AbsoluteXIndexedIndirect)),
            0x7D => Some(Instruction::ADC(AddressingMode::AbsoluteXIndexed)),
            0x7E => Some(Instruction::ROR(AddressingMode::AbsoluteXIndexed)),
            0x7F => None,

            0x80 => Some(Instruction::BRA(AddressingMode::Relative)),
            0x81 => Some(Instruction::STA(AddressingMode::ZeroPageXIndexedIndirect)),
            0x82 => None,
            0x83 => None,
            0x84 => Some(Instruction::STY(AddressingMode::ZeroPage)),
            0x85 => Some(Instruction::STA(AddressingMode::ZeroPage)),
            0x86 => Some(Instruction::STX(AddressingMode::ZeroPage)),
            0x87 => None,
            0x88 => Some(Instruction::DEY(AddressingMode::Implied)),
            0x89 => Some(Instruction::BIT(AddressingMode::Immediate)),
            0x8A => Some(Instruction::TXA(AddressingMode::Implied)),
            0x8B => None,
            0x8C => Some(Instruction::STY(AddressingMode::Absolute)),
            0x8D => Some(Instruction::STA(AddressingMode::Absolute)),
            0x8E => Some(Instruction::STX(AddressingMode::Absolute)),
            0x8F => None,

            0x90 => Some(Instruction::BCC(AddressingMode::Relative)),
            0x91 => Some(Instruction::STA(AddressingMode::ZeroPageIndirectIndexedY)),
            0x92 => Some(Instruction::STA(AddressingMode::ZeroPageIndirect)),
            0x93 => None,
            0x94 => Some(Instruction::STY(AddressingMode::ZeroPageXIndexed)),
            0x95 => Some(Instruction::STA(AddressingMode::ZeroPageXIndexed)),
            0x96 => Some(Instruction::STX(AddressingMode::ZeroPageYIndexed)),
            0x97 => None,
            0x98 => Some(Instruction::TYA(AddressingMode::Implied)),
            0x99 => Some(Instruction::STA(AddressingMode::AbsoluteYIndexed)),
            0x9A => Some(Instruction::TXS(AddressingMode::Implied)),
            0x9B => None,
            0x9C => Some(Instruction::STZ(AddressingMode::Absolute)),
            0x9D => Some(Instruction::STA(AddressingMode::AbsoluteXIndexed)),
            0x9E => Some(Instruction::STZ(AddressingMode::AbsoluteXIndexed)),
            0x9F => None,

            0xA0 => Some(Instruction::LDY(AddressingMode::Immediate)),
            0xA1 => Some(Instruction::LDA(AddressingMode::ZeroPageXIndexedIndirect)),
            0xA2 => Some(Instruction::LDX(AddressingMode::Immediate)),
            0xA3 => None,
            0xA4 => Some(Instruction::LDY(AddressingMode::ZeroPage)),
            0xA5 => Some(Instruction::LDA(AddressingMode::ZeroPage)),
            0xA6 => Some(Instruction::LDX(AddressingMode::ZeroPage)),
            0xA7 => None,
            0xA8 => Some(Instruction::TAY(AddressingMode::Implied)),
            0xA9 => Some(Instruction::LDA(AddressingMode::Immediate)),
            0xAA => Some(Instruction::TAX(AddressingMode::Implied)),
            0xAB => None,
            0xAC => Some(Instruction::LDY(AddressingMode::Absolute)),
            0xAD => Some(Instruction::LDA(AddressingMode::Absolute)),
            0xAE => Some(Instruction::LDX(AddressingMode::Absolute)),
            0xAF => None,

            0xB0 => Some(Instruction::BCS(AddressingMode::Relative)),
            0xB1 => Some(Instruction::LDA(AddressingMode::ZeroPageIndirectIndexedY)),
            0xB2 => Some(Instruction::LDA(AddressingMode::ZeroPageIndirect)),
            0xB3 => None,
            0xB4 => Some(Instruction::LDY(AddressingMode::ZeroPageXIndexed)),
            0xB5 => Some(Instruction::LDA(AddressingMode::ZeroPageXIndexed)),
            0xB6 => Some(Instruction::LDX(AddressingMode::ZeroPageYIndexed)),
            0xB7 => None,
            0xB8 => Some(Instruction::CLV(AddressingMode::Implied)),
            0xB9 => Some(Instruction::LDA(AddressingMode::AbsoluteYIndexed)),
            0xBA => Some(Instruction::TSX(AddressingMode::Implied)),
            0xBB => None,
            0xBC => Some(Instruction::LDY(AddressingMode::AbsoluteXIndexed)),
            0xBD => Some(Instruction::LDA(AddressingMode::AbsoluteXIndexed)),
            0xBE => Some(Instruction::LDX(AddressingMode::AbsoluteYIndexed)),
            0xBF => None,

            0xC0 => Some(Instruction::CPY(AddressingMode::Immediate)),
            0xC1 => Some(Instruction::CMP(AddressingMode::ZeroPageXIndexedIndirect)),
            0xC2 => None,
            0xC3 => None,
            0xC4 => Some(Instruction::CPY(AddressingMode::ZeroPage)),
            0xC5 => Some(Instruction::CMP(AddressingMode::ZeroPage)),
            0xC6 => Some(Instruction::DEC(AddressingMode::ZeroPage)),
            0xC7 => None,
            0xC8 => Some(Instruction::INY(AddressingMode::Implied)),
            0xC9 => Some(Instruction::CMP(AddressingMode::Immediate)),
            0xCA => Some(Instruction::DEX(AddressingMode::Implied)),
            0xCB => None,
            0xCC => Some(Instruction::CPY(AddressingMode::Absolute)),
            0xCD => Some(Instruction::CMP(AddressingMode::Absolute)),
            0xCE => Some(Instruction::DEC(AddressingMode::Absolute)),
            0xCF => None,

            0xD0 => Some(Instruction::BNE(AddressingMode::Relative)),
            0xD1 => Some(Instruction::CMP(AddressingMode::ZeroPageIndirectIndexedY)),
            0xD2 => Some(Instruction::CMP(AddressingMode::ZeroPageIndirect)),
            0xD3 => None,
            0xD4 => None,
            0xD5 => Some(Instruction::CMP(AddressingMode::ZeroPageXIndexed)),
            0xD6 => Some(Instruction::DEC(AddressingMode::ZeroPageXIndexed)),
            0xD7 => None,
            0xD8 => Some(Instruction::CLD(AddressingMode::Implied)),
            0xD9 => Some(Instruction::CMP(AddressingMode::AbsoluteYIndexed)),
            0xDA => Some(Instruction::PHX(AddressingMode::Implied)),
            0xDB => None,
            0xDC => None,
            0xDD => Some(Instruction::CMP(AddressingMode::AbsoluteXIndexed)),
            0xDE => Some(Instruction::DEC(AddressingMode::AbsoluteXIndexed)),
            0xDF => None,

            0xE0 => Some(Instruction::CPX(AddressingMode::Immediate)),
            0xE1 => Some(Instruction::SBC(AddressingMode::ZeroPageXIndexedIndirect)),
            0xE2 => None,
            0xE3 => None,
            0xE4 => Some(Instruction::CPX(AddressingMode::ZeroPage)),
            0xE5 => Some(Instruction::SBC(AddressingMode::ZeroPage)),
            0xE6 => Some(Instruction::INC(AddressingMode::ZeroPage)),
            0xE7 => None,
            0xE8 => Some(Instruction::INX(AddressingMode::Implied)),
            0xE9 => Some(Instruction::SBC(AddressingMode::Immediate)),
            0xEA => Some(Instruction::NOP(AddressingMode::Implied)),
            0xEB => None,
            0xEC => Some(Instruction::CPX(AddressingMode::Absolute)),
            0xED => Some(Instruction::SBC(AddressingMode::Absolute)),
            0xEE => Some(Instruction::INC(AddressingMode::Absolute)),
            0xEF => None,

            0xF0 => Some(Instruction::BEQ(AddressingMode::Relative)),
            0xF1 => Some(Instruction::SBC(AddressingMode::ZeroPageIndirectIndexedY)),
            0xF2 => Some(Instruction::SBC(AddressingMode::ZeroPageIndirect)),
            0xF3 => None,
            0xF4 => None,
            0xF5 => Some(Instruction::SBC(AddressingMode::ZeroPageXIndexed)),
            0xF6 => Some(Instruction::INC(AddressingMode::ZeroPageXIndexed)),
            0xF7 => None,
            0xF8 => Some(Instruction::SED(AddressingMode::Implied)),
            0xF9 => Some(Instruction::SBC(AddressingMode::AbsoluteYIndexed)),
            0xFA => Some(Instruction::PLX(AddressingMode::Implied)),
            0xFB => None,
            0xFC => None,
            0xFD => Some(Instruction::SBC(AddressingMode::AbsoluteXIndexed)),
            0xFE => Some(Instruction::INC(AddressingMode::AbsoluteXIndexed)),
            0xFF => None,
        }
    }
}

