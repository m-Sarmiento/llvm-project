//===-- OPEN8InstrInfo.td - OPEN8 Instruction Formats ----------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// OPEN8 Instruction Format Definitions.
//
//===----------------------------------------------------------------------===//

// A generic OPEN8 instruction.
class OPEN8Inst<dag outs, dag ins, string asmstr, list<dag> pattern> : Instruction
{
  let Namespace = "OPEN8";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;

  field bits<32> SoftFail = 0;
}

/// A 8-bit OPEN8 instruction.
class OPEN8Inst8<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst<outs, ins, asmstr, pattern>
{
  field bits<8> Inst;

  let Size = 1;
}

/// A 16-bit OPEN8 instruction.
class OPEN8Inst16<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst<outs, ins, asmstr, pattern>
{
  field bits<16> Inst;

  let Size = 2;
}

/// A 24-bit OPEN8 instruction.
class OPEN8Inst24<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst<outs, ins, asmstr, pattern>
{
  field bits<24> Inst;

  let Size = 3;
}

/// a 32-bit OPEN8 instruction.
class OPEN8Inst32<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst<outs, ins, asmstr, pattern>
{
  field bits<32> Inst;

  let Size = 4;
}

// A class for pseudo instructions.
// Pseudo instructions are not real OPEN8 instructions. The DAG stores
// pseudo instructions which are replaced by real OPEN8 instructions by
// OPEN8ExpandPseudoInsts.cpp.
//
// For example, the ADDW (add wide, as in add 16 bit values) instruction
// is defined as a pseudo instruction. In OPEN8ExpandPseudoInsts.cpp,
// the instruction is then replaced by two add instructions - one for each byte.
class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  let Pattern = pattern;

  let isPseudo = 1;
  let isCodeGenOnly = 1;
}

// OPEN8 Instructions Format

class OPEN8Op<bits<5> val> {
  bits<5> Value = val;
}
def OP_INC    : OPEN8Op<0b00000>;
def OP_ADC    : OPEN8Op<0b00001>;
def OP_TX0    : OPEN8Op<0b00010>;
def OP_OR     : OPEN8Op<0b00011>;
def OP_AND    : OPEN8Op<0b00100>;
def OP_XOR    : OPEN8Op<0b00101>;
def OP_ROL    : OPEN8Op<0b00110>;
def OP_ROR    : OPEN8Op<0b00111>;
def OP_DEC    : OPEN8Op<0b01000>;
def OP_SBC    : OPEN8Op<0b01001>;
def OP_ADD    : OPEN8Op<0b01010>;
def OP_STP    : OPEN8Op<0b01011>;
def OP_BTT    : OPEN8Op<0b01100>;
def OP_CLP    : OPEN8Op<0b01101>;
def OP_T0X    : OPEN8Op<0b01110>;
def OP_CMP    : OPEN8Op<0b01111>;
def OP_PSH    : OPEN8Op<0b10000>;
def OP_POP    : OPEN8Op<0b10001>;
def OP_BR0    : OPEN8Op<0b10010>;
def OP_BR1    : OPEN8Op<0b10011>;
def OP_DBNZ   : OPEN8Op<0b10100>;
def OP_INT    : OPEN8Op<0b10101>;
def OP_MUL    : OPEN8Op<0b10110>;
def OP_INST   : OPEN8Op<0b10111>;
//selecyt def OPEN8Ints
def OP_UPP    : OPEN8Op<0b11000>;
def OP_STA    : OPEN8Op<0b11001>;
def OP_STX    : OPEN8Op<0b11010>;
def OP_STO    : OPEN8Op<0b11011>;
def OP_LDI    : OPEN8Op<0b11100>;
def OP_LDA    : OPEN8Op<0b11101>;
def OP_LDX    : OPEN8Op<0b11110>;
def OP_LDO    : OPEN8Op<0b11111>;

class OPEN8Ints<bits<3> val> {
  bits<3> Value = val;
}
def OP_RSP    : OPEN8Ints<0b000>;
def OP_RTS    : OPEN8Ints<0b001>;
def OP_RTI    : OPEN8Ints<0b010>;
def OP_BRK    : OPEN8Ints<0b011>;
def OP_JMP    : OPEN8Ints<0b100>;
def OP_SMSK   : OPEN8Ints<0b101>;
def OP_GMSK   : OPEN8Ints<0b110>;
def OP_JSR    : OPEN8Ints<0b111>;

class OPEN8Opbr<bits<8> val> {
  bits<8> Value = val;
}
def OP_BR00    : OPEN8Opbr<0b10010000>;
def OP_BR10    : OPEN8Opbr<0b10011000>;
def OP_BR01    : OPEN8Opbr<0b10010001>;
def OP_BR11    : OPEN8Opbr<0b10011001>;
def OP_BR02    : OPEN8Opbr<0b10010010>;
def OP_BR12    : OPEN8Opbr<0b10011010>;
def OP_BR03    : OPEN8Opbr<0b10010011>;
def OP_BR13    : OPEN8Opbr<0b10011011>;
def OP_BR04    : OPEN8Opbr<0b10010100>;
def OP_BR14    : OPEN8Opbr<0b10011100>;
def OP_BR05    : OPEN8Opbr<0b10010101>;
def OP_BR15    : OPEN8Opbr<0b10011101>;
def OP_BR06    : OPEN8Opbr<0b10010110>;
def OP_BR16    : OPEN8Opbr<0b10011110>;
def OP_BR07    : OPEN8Opbr<0b10010111>;
def OP_BR17    : OPEN8Opbr<0b10011111>;

//===----------------------------------------------------------------------===//
//Opcode / register instruction: <|opcode|rrr|>
// opcode = 5 bits.
// r = register = 3 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class OPEN8OpReg<OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst8<outs, ins, asmstr, pattern>
{
  bits<3> rn;

  let Inst{7-3} = opcode.Value;
  let Inst{2-0} = rn{2-0};
}

//For STX and LDX instrcutions
class OPEN8OpRegX<bits<1> A, OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst8<outs, ins, asmstr, pattern>
{
  bits<3> rn;

  let Inst{7-3} = opcode.Value;
  let Inst{2-1} = rn{2-1};
  let Inst{0} = A;

}

//===----------------------------------------------------------------------===//
//Opcode / N instruction: <|opcode|nnn|>
// opcode = 5 bits.
// n = immediate = 3 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class OPEN8OpN<OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst8<outs, ins, asmstr, pattern>
{
  bits<3> n;

  let Inst{7-3} = opcode.Value;
  let Inst{2-0} = n{2-0};
}

//===----------------------------------------------------------------------===//
//Opcode / BitIndex / Offset instruction: <|opcode|nnn|oooooooo|>
// opcode = 5 bits.
// n = bitindex = 3 bits
// o = offset = 8 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class OPEN8OpBitOff2<OPEN8Opbr opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  //bits<3> bitIndex;
  bits<8> offset;

  let Inst{15-8} = opcode.Value;
  //let Inst{10-8} = bitIndex{2-0};
  let Inst{7-0} = offset{7-0};
}

//===----------------------------------------------------------------------===//
//Opcode / BitIndex / Offset instruction: <|opcode|nnn|oooooooo|>
// opcode = 5 bits.
// n = bitindex = 3 bits
// o = offset = 8 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//

class OPEN8OpBitOff<OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<3> bitIndex;
  bits<8> offset;

  let Inst{15-11} = opcode.Value;
  let Inst{10-8} = bitIndex{2-0};
  let Inst{7-0} = offset{7-0};
}

//===----------------------------------------------------------------------===//
//Opcode / Register / Offset instruction: <|opcode|rrr|oooooooo|>
// opcode = 5 bits.
// r = register = 3 bits
// o = offset = 8 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class OPEN8OpRegOff<OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<3> rn;
  bits<8> offset;

  let Inst{15-11} = opcode.Value;
  let Inst{10-8} = rn{2-0};
  let Inst{7-0} = offset{7-0};
}

//For STO and LDO instructions
class OPEN8OpRegOffX<bits<1> A, OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<3> rn;
  bits<8> offset;

  let Inst{15-11} = opcode.Value;
  let Inst{10-9} = rn{2-1};
  let Inst{8} = A;
  let Inst{7-0} = offset{7-0};
}

//===----------------------------------------------------------------------===//
//Opcode / select instruction: <|opcode|sss|>
// opcode = 5 bits.
// s = select = 3 bits
//===----------------------------------------------------------------------===//
class OPEN8OpSel<OPEN8Op opcode,OPEN8Ints select, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst8<outs, ins, asmstr, pattern>
{
  let Inst{7-3} = opcode.Value;
  let Inst{2-0} = select.Value;
}

//===----------------------------------------------------------------------===//
//Opcode / select / adresss instruction: <|opcode|sss|aaaaaaaaaaaaaaaa|>
// opcode = 5 bits.
// s = select = 3 bits
// a = adress = 16 bits
//===----------------------------------------------------------------------===//
class OPEN8OpSelAddr<OPEN8Op opcode,OPEN8Ints select, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst24<outs, ins, asmstr, pattern>
{
  bits<16> addr;

  let Inst{23-19} = opcode.Value;
  let Inst{18-16} = select.Value;
  let Inst{15-0} = addr{15-0};
}

//===----------------------------------------------------------------------===//
//Opcode / reg / adresss instruction: <|opcode|rrr|aaaaaaaaaaaaaaaa|>
// opcode = 5 bits.
// r = register = 3 bits
// a = adress = 16 bits
//===----------------------------------------------------------------------===//
class OPEN8OpRegAddr<OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst24<outs, ins, asmstr, pattern>
{
  bits<3> rn;
  bits<16> addr;

  let Inst{23-19} = opcode.Value;
  let Inst{18-16} = rn{2-0};
  let Inst{15-0} = addr;
}

//===----------------------------------------------------------------------===//
//Opcode / Register / immmediate instruction: <|opcode|rrr|iiiiiiii|>
// opcode = 5 bits.
// r = register = 3 bits
// i = immediate = 8 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class OPEN8OpRegImm<OPEN8Op opcode, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<3> rn;
  bits<8> immediate;

  let Inst{15-11} = opcode.Value;
  let Inst{10-8} = rn{2-0};
  let Inst{7-0} = immediate{7-0};
}


//===----------------------------------------------------------------------===//
// Register / register instruction: <|opcode|ffrd|dddd|rrrr|>
// opcode = 4 bits.
// f = secondary opcode = 2 bits
// d = destination = 5 bits
// r = source = 5 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class FRdRr<bits<4> opcode, bits<2> f, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> rd;
  bits<5> rr;

  let Inst{15-12} = opcode;
  let Inst{11-10} = f;
  let Inst{9} = rr{4};
  let Inst{8-4} = rd;
  let Inst{3-0} = rr{3-0};
}

class FTST<bits<4> opcode, bits<2> f, dag outs, dag ins, string asmstr,
            list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> rd;

  let Inst{15-12} = opcode;
  let Inst{11-10} = f;
  let Inst{9} = rd{4};
  let Inst{8-4} = rd;
  let Inst{3-0} = rd{3-0};
}

//===----------------------------------------------------------------------===//
// Instruction of the format `<mnemonic> Z, Rd`
// <|1001|001r|rrrr|0ttt>
//===----------------------------------------------------------------------===//
class FZRd<bits<3> t, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> rd;

  let Inst{15-12} = 0b1001;

  let Inst{11-9} = 0b001;
  let Inst{8} = rd{4};

  let Inst{7-4} = rd{3-0};

  let Inst{3} = 0;
  let Inst{2-0} = t;
}

//===----------------------------------------------------------------------===//
// Register / immediate8 instruction: <|opcode|KKKK|dddd|KKKK|>
// opcode = 4 bits.
// K = constant data = 8 bits
// d = destination = 4 bits
// (Only accepts r16-r31)
//===----------------------------------------------------------------------===//
class FRdK<bits<4> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<4> rd;
  bits<8> k;

  let Inst{15-12} = opcode;
  let Inst{11-8} = k{7-4};
  let Inst{7-4} = rd{3-0};
  let Inst{3-0} = k{3-0};

  let isAsCheapAsAMove = 1;
}

//===----------------------------------------------------------------------===//
// Register instruction: <|opcode|fffd|dddd|ffff|>
// opcode = 4 bits.
// f = secondary opcode = 7 bits
// d = destination = 5 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class FRd<bits<4> opcode, bits<7> f, dag outs, dag ins, string asmstr,
          list<dag> pattern> : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> d;

  let Inst{15-12} = opcode;
  let Inst{11-9} = f{6-4};
  let Inst{8-4} = d;
  let Inst{3-0} = f{3-0};

  let DecoderMethod = "decodeFRd";
}

//===----------------------------------------------------------------------===//
// [STD/LDD] P+q, Rr special encoding: <|10q0|qqtr|rrrr|pqqq>
// t = type (1 for STD, 0 for LDD)
// q = displacement (6 bits)
// r = register (5 bits)
// p = pointer register (1 bit) [1 for Y, 0 for Z]
//===----------------------------------------------------------------------===//
class FSTDLDD<bit type, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<7> memri;
  bits<5> reg; // the GP register

  let Inst{15-14} = 0b10;
  let Inst{13} = memri{5};
  let Inst{12} = 0;

  let Inst{11-10} = memri{4-3};
  let Inst{9} = type;
  let Inst{8} = reg{4};

  let Inst{7-4} = reg{3-0};

  let Inst{3} = memri{6};
  let Inst{2-0} = memri{2-0};
}

//===---------------------------------------------------------------------===//
// An ST/LD instruction.
// <|100i|00tr|rrrr|ppaa|>
// t = type (1 for store, 0 for load)
// a = regular/postinc/predec (reg = 0b00, postinc = 0b01, predec = 0b10)
// p = pointer register
// r = src/dst register
//
// Note that the bit labelled 'i' above does not follow a simple pattern,
// so there exists a post encoder method to set it manually.
//===---------------------------------------------------------------------===//
class FSTLD<bit type, bits<2> mode, dag outs, dag ins,
            string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<2> ptrreg;
  bits<5> reg;

  let Inst{15-13} = 0b100;
  // This bit varies depending on the arguments and the mode.
  // We have a post encoder method to set this bit manually.
  let Inst{12} = 0;

  let Inst{11-10} = 0b00;
  let Inst{9} = type;
  let Inst{8} = reg{4};

  let Inst{7-4} = reg{3-0};

  let Inst{3-2} = ptrreg{1-0};
  let Inst{1-0} = mode{1-0};

  let PostEncoderMethod = "loadStorePostEncoder";
}

//===---------------------------------------------------------------------===//
// Special format for the LPM/ELPM instructions
// [E]LPM Rd, Z[+]
// <|1001|000d|dddd|01ep>
// d = destination register
// e = is elpm
// p = is postincrement
//===---------------------------------------------------------------------===//
class FLPMX<bit e, bit p, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
   bits<5> reg;

   let Inst{15-12} = 0b1001;

   let Inst{11-9} = 0b000;
   let Inst{8} = reg{4};

   let Inst{7-4} = reg{3-0};

   let Inst{3-2} = 0b01;
   let Inst{1} = e;
   let Inst{0} = p;

   let DecoderMethod = "decodeFLPMX";
}

//===----------------------------------------------------------------------===//
// MOVWRdRr special encoding: <|0000|0001|dddd|rrrr|>
// d = destination = 4 bits
// r = source = 4 bits
// (Only accepts even registers)
//===----------------------------------------------------------------------===//
class FMOVWRdRr<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> d;
  bits<5> r;

  let Inst{15-8} = 0b00000001;
  let Inst{7-4} = d{4-1};
  let Inst{3-0} = r{4-1};

  let DecoderMethod = "decodeFMOVWRdRr";
}

//===----------------------------------------------------------------------===//
// MULSrr special encoding: <|0000|0010|dddd|rrrr|>
// d = multiplicand = 4 bits
// r = multiplier = 4 bits
// (Only accepts r16-r31)
//===----------------------------------------------------------------------===//
class FMUL2RdRr<bit f, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> rd;              // accept 5 bits but only encode the lower 4
  bits<5> rr;              // accept 5 bits but only encode the lower 4

  let Inst{15-9} = 0b0000001;
  let Inst{8} = f;
  let Inst{7-4} = rd{3-0};
  let Inst{3-0} = rr{3-0};

  let DecoderMethod = "decodeFMUL2RdRr";
}

// Special encoding for the FMUL family of instructions.
//
// <0000|0011|fddd|frrr|>
//
// ff = 0b01 for FMUL
//      0b10 for FMULS
//      0b11 for FMULSU
//
// ddd = destination register
// rrr = source register
class FFMULRdRr<bits<2> f, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<3> rd;
  bits<3> rr;

  let Inst{15-8} = 0b00000011;
  let Inst{7} = f{1};
  let Inst{6-4} = rd;
  let Inst{3} = f{0};
  let Inst{2-0} = rr;

  let DecoderMethod = "decodeFFMULRdRr";
}


//===----------------------------------------------------------------------===//
// Arithmetic word instructions (ADIW / SBIW): <|1001|011f|kkdd|kkkk|>
// f = secondary opcode = 1 bit
// k = constant data = 6 bits
// d = destination = 4 bits
// (Only accepts r25:24 r27:26 r29:28 r31:30)
//===----------------------------------------------------------------------===//
class FWRdK<bit f, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> dst;              // accept 5 bits but only encode bits 1 and 2
  bits<6> k;

  let Inst{15-9} = 0b1001011;
  let Inst{8} = f;
  let Inst{7-6} = k{5-4};
  let Inst{5-4} = dst{2-1};
  let Inst{3-0} = k{3-0};

  let DecoderMethod = "decodeFWRdK";
}

//===----------------------------------------------------------------------===//
// In I/O instruction: <|1011|0AAd|dddd|AAAA|>
// A = I/O location address = 6 bits
// d = destination = 5 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class FIORdA<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> d;
  bits<6> A;

  let Inst{15-11} = 0b10110;
  let Inst{10-9} = A{5-4};
  let Inst{8-4} = d;
  let Inst{3-0} = A{3-0};

  let DecoderMethod = "decodeFIORdA";
}

//===----------------------------------------------------------------------===//
// Out I/O instruction: <|1011|1AAr|rrrr|AAAA|>
// A = I/O location address = 6 bits
// d = destination = 5 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class FIOARr<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<6> A;
  bits<5> r;

  let Inst{15-11} = 0b10111;
  let Inst{10-9} = A{5-4};
  let Inst{8-4} = r;
  let Inst{3-0} = A{3-0};

  let DecoderMethod = "decodeFIOARr";
}

//===----------------------------------------------------------------------===//
// I/O bit instruction.
// <|1001|10tt|AAAA|Abbb>
// t = type (1 for SBI, 0 for CBI)
// A = I/O location address (5 bits)
// b = bit number
//===----------------------------------------------------------------------===//
class FIOBIT<bits<2> t, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> A;
  bits<3> b;

  let Inst{15-12} = 0b1001;

  let Inst{11-10} = 0b10;
  let Inst{9-8} = t;

  let Inst{7-4} = A{4-1};

  let Inst{3} = A{0};
  let Inst{2-0} = b{2-0};

  let DecoderMethod = "decodeFIOBIT";
}

//===----------------------------------------------------------------------===//
// BST/BLD instruction.
// <|1111|1ttd|dddd|0bbb>
// t = type (1 for BST, 0 for BLD)
// d = destination register
// b = bit
//===----------------------------------------------------------------------===//
class FRdB<bits<2> t, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<5> rd;
  bits<3> b;

  let Inst{15-12} = 0b1111;

  let Inst{11} = 0b1;
  let Inst{10-9} = t;
  let Inst{8} = rd{4};

  let Inst{7-4} = rd{3-0};

  let Inst{3} = 0;
  let Inst{2-0} = b;
}

// Special encoding for the `DES K` instruction.
//
// <|1001|0100|KKKK|1011>
//
// KKKK = 4 bit immediate
class FDES<dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<4> k;

  let Inst{15-12} = 0b1001;

  let Inst{11-8} = 0b0100;

  let Inst{7-4} = k;

  let Inst{3-0} = 0b1011;
}

//===----------------------------------------------------------------------===//
// Conditional Branching instructions: <|1111|0fkk|kkkk|ksss|>
// f = secondary opcode = 1 bit
// k = constant address = 7 bits
// s = bit in status register = 3 bits
//===----------------------------------------------------------------------===//
class FBRsk<bit f, bits<3> s, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<7> k;

  let Inst{15-11} = 0b11110;
  let Inst{10} = f;
  let Inst{9-3} = k;
  let Inst{2-0} = s;
}

//===----------------------------------------------------------------------===//
// Special, opcode only instructions: <|opcode|>
//===----------------------------------------------------------------------===//

class F16<bits<16> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  let Inst = opcode;
}

class F32<bits<32> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst32<outs, ins, asmstr, pattern>
{
  let Inst = opcode;
}

//===----------------------------------------------------------------------===//
// Branching instructions with immediate12: <|110f|kkkk|kkkk|kkkk|>
// f = secondary opcode = 1 bit
// k = constant address = 12 bits
//===----------------------------------------------------------------------===//
class FBRk<bit f, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<12> k;

  let Inst{15-13} = 0b110;
  let Inst{12} = f;
  let Inst{11-0} = k;
}

//===----------------------------------------------------------------------===//
// 32 bits branching instructions: <|1001|010k|kkkk|fffk|kkkk|kkkk|kkkk|kkkk|>
// f = secondary opcode = 3 bits
// k = constant address = 22 bits
//===----------------------------------------------------------------------===//
class F32BRk<bits<3> f, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst32<outs, ins, asmstr, pattern>
{
  bits<22> k;

  let Inst{31-25} = 0b1001010;
  let Inst{24-20} = k{21-17};
  let Inst{19-17} = f;
  let Inst{16-0} = k{16-0};
}

//===----------------------------------------------------------------------===//
// 32 bits direct mem instructions: <|1001|00fd|dddd|0000|kkkk|kkkk|kkkk|kkkk|>
// f = secondary opcode = 1 bit
// d = destination = 5 bits
// k = constant address = 16 bits
// (Accepts all registers)
//===----------------------------------------------------------------------===//
class F32DM<bit f, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst32<outs, ins, asmstr, pattern>
{
  bits<5> rd;
  bits<16> k;

  let Inst{31-28} = 0b1001;

  let Inst{27-26} = 0b00;
  let Inst{25} = f;
  let Inst{24} = rd{4};

  let Inst{23-20} = rd{3-0};

  let Inst{19-16} = 0b0000;

  let Inst{15-0} = k;
}

// <|1001|0100|bfff|1000>
class FS<bit b, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<3> s;

  let Inst{15-12} = 0b1001;

  let Inst{11-8} = 0b0100;

  let Inst{7} = b;
  let Inst{6-4} = s;

  let Inst{3-0} = 0b1000;
}

// Set/clr bit in status flag instructions/
// <BRBS|BRBC> s, k
// ---------------------
// <|1111|0fkk|kkkk|ksss>
class FSK<bit f, dag outs, dag ins, string asmstr, list<dag> pattern>
  : OPEN8Inst16<outs, ins, asmstr, pattern>
{
  bits<7> k;
  bits<3> s;

  let Inst{15-12} = 0b1111;

  let Inst{11} = 0;
  let Inst{10} = f;
  let Inst{9-8} = k{6-5};

  let Inst{7-4} = k{4-1};

  let Inst{3} = k{0};
  let Inst{2-0} = s;
}

class ExtensionPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let Defs = [SREG];
}

class StorePseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let Defs = [SP];
}

class SelectPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let usesCustomInserter = 1;

  let Uses = [SREG];
}

class ShiftPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let usesCustomInserter = 1;

  let Defs = [SREG];
}

