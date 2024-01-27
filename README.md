# WDC 65C02 Emulator

A WDC 65C02 emulator written in Rust.
This project is a work in progress.

**Warning: This project is not yet complete, and is not ready for use. It still contains many bugs!**

## Usage

```bash

w65c02 a.out

```

Where `a.out` is a binary file containing 65C02 instructions, containing exactly 1k bytes of data.

For example, running this program that calculates the Fibonacci sequence:

```asm

RESULT_1 = $d000
RESULT_2 = $d001

reset:
  lda #$01
  sta RESULT_1
  sta RESULT_2

loop:
  lda RESULT_1
  adc RESULT_2
  ldx RESULT_1
  sta RESULT_1
  stx RESULT_2
  jmp loop

  .org $fffc
  .word reset
  .word $00

```

Assembling this program using [vasm](http://sun.hasenbraten.de/vasm/)

```bash

vasm6502_oldstyle -c02 -Fbin test.s

```

Results in the following debug output from the emulator (Press enter to step through the program):

```bash
PC		0011
RA		59	89
RX		37	55
RY		00	0
SP		0000
P		0000
Address		d000
Data		37	55
Instruction	Some(LDX(Absolute))

Memory:
0000:	a9 01 8d 00 d0 8d 01 d0 ad 00 d0 6d 01 d0 ae 00
0010:	d0 8d 00 d0 8e 01 d0 4c 08 00 00 00 00 00 00 00
...
d000:	37 22 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```
