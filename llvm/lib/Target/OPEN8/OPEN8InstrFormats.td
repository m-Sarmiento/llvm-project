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

