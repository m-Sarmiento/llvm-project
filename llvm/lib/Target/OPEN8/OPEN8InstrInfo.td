//===-- OPEN8InstrInfo.td - OPEN8 Instruction defs -------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the OPEN8 instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "OPEN8InstrFormats.td"

//===----------------------------------------------------------------------===//
// OPEN8 Type Profiles
//===----------------------------------------------------------------------===//

def SDT_OPEN8CallSeqStart : SDCallSeqStart<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_OPEN8CallSeqEnd : SDCallSeqEnd<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_OPEN8Call : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
def SDT_OPEN8Wrapper : SDTypeProfile<1, 1, [SDTCisSameAs<0, 1>, SDTCisPtrTy<0>]>;
def SDT_OPEN8Brcond : SDTypeProfile<0, 2,
                                  [SDTCisVT<0, OtherVT>, SDTCisVT<1, i8>]>;
def SDT_OPEN8Cmp : SDTypeProfile<0, 2, [SDTCisSameAs<0, 1>]>;
def SDT_OPEN8Tst : SDTypeProfile<0, 1, [SDTCisInt<0>]>;
def SDT_OPEN8SelectCC : SDTypeProfile<1, 3, [SDTCisSameAs<0, 1>,
                                    SDTCisSameAs<1, 2>, SDTCisVT<3, i8>]>;

//===----------------------------------------------------------------------===//
// OPEN8 Specific Node Definitions
//===----------------------------------------------------------------------===//

def OPEN8rtsflag : SDNode<"OPEN8ISD::RTS_FLAG", SDTNone,
                        [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def OPEN8rtiflag : SDNode<"OPEN8ISD::RTI_FLAG", SDTNone,
                         [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def OPEN8callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_OPEN8CallSeqStart,
                              [SDNPHasChain, SDNPOutGlue]>;
def OPEN8callseq_end : SDNode<"ISD::CALLSEQ_END", SDT_OPEN8CallSeqEnd,
                            [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def OPEN8call : SDNode<"OPEN8ISD::CALL", SDT_OPEN8Call,
                     [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue, SDNPVariadic]>;

def OPEN8Wrapper : SDNode<"OPEN8ISD::WRAPPER", SDT_OPEN8Wrapper>;

def OPEN8brcond : SDNode<"OPEN8ISD::BRCOND", SDT_OPEN8Brcond,
                       [SDNPHasChain, SDNPInGlue]>;
def OPEN8cmp : SDNode<"OPEN8ISD::CMP", SDT_OPEN8Cmp, [SDNPOutGlue]>;
def OPEN8cmpc : SDNode<"OPEN8ISD::CMPC", SDT_OPEN8Cmp, [SDNPInGlue, SDNPOutGlue]>;
def OPEN8tst : SDNode<"OPEN8ISD::TST", SDT_OPEN8Tst, [SDNPOutGlue]>;
def OPEN8selectcc: SDNode<"OPEN8ISD::SELECT_CC", SDT_OPEN8SelectCC, [SDNPInGlue]>;

// Shift nodes.
def OPEN8lsl : SDNode<"OPEN8ISD::LSL", SDTIntUnaryOp>;
def OPEN8lsl4 : SDNode<"OPEN8ISD::LSL4", SDTIntUnaryOp>;
def OPEN8lsl8 : SDNode<"OPEN8ISD::LSL8", SDTIntUnaryOp>;
def OPEN8lsl12 : SDNode<"OPEN8ISD::LSL12", SDTIntUnaryOp>;
def OPEN8lsr : SDNode<"OPEN8ISD::LSR", SDTIntUnaryOp>;
def OPEN8lsr4 : SDNode<"OPEN8ISD::LSR4", SDTIntUnaryOp>;
def OPEN8lsr8 : SDNode<"OPEN8ISD::LSR8", SDTIntUnaryOp>;
def OPEN8lsr12 : SDNode<"OPEN8ISD::LSR12", SDTIntUnaryOp>;
def OPEN8rol : SDNode<"OPEN8ISD::ROL", SDTIntUnaryOp>;
def OPEN8ror : SDNode<"OPEN8ISD::ROR", SDTIntUnaryOp>;
def OPEN8asr : SDNode<"OPEN8ISD::ASR", SDTIntUnaryOp>;
def OPEN8asr8 : SDNode<"OPEN8ISD::ASR8", SDTIntUnaryOp>;
def OPEN8lsl7 : SDNode<"OPEN8ISD::LSL7", SDTIntUnaryOp>;
def OPEN8lsr7 : SDNode<"OPEN8ISD::LSR7", SDTIntUnaryOp>;
def OPEN8asr7 : SDNode<"OPEN8ISD::ASR7", SDTIntUnaryOp>;

// Pseudo shift nodes for non-constant shift amounts.
def OPEN8lslLoop : SDNode<"OPEN8ISD::LSLLOOP", SDTIntShiftOp>;
def OPEN8lsrLoop : SDNode<"OPEN8ISD::LSRLOOP", SDTIntShiftOp>;
def OPEN8rolLoop : SDNode<"OPEN8ISD::ROLLOOP", SDTIntShiftOp>;
def OPEN8rorLoop : SDNode<"OPEN8ISD::RORLOOP", SDTIntShiftOp>;
def OPEN8asrLoop : SDNode<"OPEN8ISD::ASRLOOP", SDTIntShiftOp>;

// SWAP node.
def OPEN8Swap : SDNode<"OPEN8ISD::SWAP", SDTIntUnaryOp>;

//===----------------------------------------------------------------------===//
// OPEN8 Operands, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

def imm8_neg_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(-N->getAPIntValue(), SDLoc(N), MVT::i8);
}]>;

def imm16_neg_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(-N->getAPIntValue(), SDLoc(N), MVT::i16);
}]>;

def imm0_63_neg : PatLeaf<(imm),
[{
  int64_t val = -N->getSExtValue();
  return val >= 0 && val < 64;
}], imm16_neg_XFORM>;

def uimm6 : PatLeaf<(imm), [{ return isUInt<6>(N->getZExtValue()); }]>;

// imm_com8_XFORM - Return the complement of a imm_com8 value
def imm_com8_XFORM : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(~((uint8_t)N->getZExtValue()), SDLoc(N),
                                   MVT::i8);
}]>;

// imm_com8 - Match an immediate that is a complement
// of a 8-bit immediate.
// Note: this pattern doesn't require an encoder method and such, as it's
// only used on aliases (Pat<> and InstAlias<>). The actual encoding
// is handled by the destination instructions, which use imm_com8.
def imm_com8_asmoperand : AsmOperandClass { let Name = "ImmCom8"; }
def imm_com8 : Operand<i8> {
  let ParserMatchClass = imm_com8_asmoperand;
}

/*def ioaddr_XFORM : SDNodeXForm<imm,
[{
  uint8_t offset = Subtarget->getIORegisterOffset();
  return CurDAG->getTargetConstant(uint8_t(N->getZExtValue()) - offset,
                                   SDLoc(N), MVT::i8);
}]>;*/

def iobitpos8_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(Log2_32(uint8_t(N->getZExtValue())),
                                   SDLoc(N), MVT::i8);
}]>;

def iobitposn8_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(Log2_32(uint8_t(~N->getZExtValue())),
                                   SDLoc(N), MVT::i8);
}]>;
/*
def ioaddr8 : PatLeaf<(imm),
[{
  uint8_t offset = Subtarget->getIORegisterOffset();
  uint64_t val = N->getZExtValue() - offset;
  return val < 0x40;
}], ioaddr_XFORM>;

def lowioaddr8 : PatLeaf<(imm),
[{
  uint8_t offset = Subtarget->getIORegisterOffset();
  uint64_t val = N->getZExtValue() - offset;
  return val < 0x20;
}], ioaddr_XFORM>;

def ioaddr16 : PatLeaf<(imm),
[{
  uint8_t offset = Subtarget->getIORegisterOffset();
  uint64_t val = N->getZExtValue() - offset;
  return val < 0x3f;
}], ioaddr_XFORM>;*/

def iobitpos8 : PatLeaf<(imm),
[{
  return isPowerOf2_32(uint8_t(N->getZExtValue()));
}], iobitpos8_XFORM>;

def iobitposn8 : PatLeaf<(imm),
[{
  return isPowerOf2_32(uint8_t(~N->getZExtValue()));
}], iobitposn8_XFORM>;

def MemriAsmOperand : AsmOperandClass {
  let Name = "Memri";
  let ParserMethod = "parseMemriOperand";
}

/// Address operand for `reg+imm` used by STD and LDD.
def memri : Operand<iPTR>
{
  let MIOperandInfo = (ops PTRDISPREGS, i16imm);

  let PrintMethod = "printMemri";
  let EncoderMethod = "encodeMemri";

  let ParserMatchClass = MemriAsmOperand;
}

// Address operand for `SP+imm` used by STD{W}SPQRr
def memspi : Operand<iPTR>
{
  let MIOperandInfo = (ops GPRSP, i16imm);
}

def relbrtarget_8 : Operand<OtherVT>
{
    let PrintMethod   = "printPCRelImm";
    let EncoderMethod = "encodeRelCondBrTarget<OPEN8::fixup_8_pcrel>";
}

// The target of a 22 or 16-bit call/jmp instruction.
def call_target : Operand<iPTR>
{
    let EncoderMethod = "encodeCallTarget";
    let DecoderMethod = "decodeCallTarget";
}

// A 16-bit address (which can lead to an R_OPEN8_16 relocation).
def imm16 : Operand<i16>
{
    let EncoderMethod = "encodeImm<OPEN8::fixup_16, 0>";
}

/// A 6-bit immediate used in the ADIW/SBIW instructions.
def imm_arith6 : Operand<i16>
{
    let EncoderMethod = "encodeImm<OPEN8::fixup_6_adiw, 0>";
}

/// An 8-bit immediate inside an instruction with the same format
/// as the `LDI` instruction (the `FRdK` format).
def imm_ldi8 : Operand<i8>
{
    let EncoderMethod = "encodeImm<OPEN8::fixup_ldi, 0>";
}

/// A 5-bit port number used in SBIC and friends (the `FIOBIT` format).
def imm_port5 : Operand<i8>
{
    let EncoderMethod = "encodeImm<OPEN8::fixup_port5, 0>";
}

/// A 6-bit port number used in the `IN` instruction and friends (the
/// `FIORdA` format.
def imm_port6 : Operand<i8>
{
    let EncoderMethod = "encodeImm<OPEN8::fixup_port6, 0>";
}

// Addressing mode pattern reg+imm6
def addr : ComplexPattern<iPTR, 2, "SelectAddr", [], [SDNPWantRoot]>;

// AsmOperand class for a pointer register.
// Used with the LD/ST family of instructions.
// See FSTLD in OPEN8InstrFormats.td
def PtrRegAsmOperand : AsmOperandClass
{
   let Name = "Reg";
}

// A special operand type for the LD/ST instructions.
// It converts the pointer register number into a two-bit field used in the
// instruction.
def LDSTPtrReg : Operand<i16>
{
    let MIOperandInfo = (ops PTRREGS);
    let EncoderMethod = "encodeLDSTPtrReg";

    let ParserMatchClass = PtrRegAsmOperand;
}

// A special operand type for the LDD/STD instructions.
// It behaves identically to the LD/ST version, except restricts
// the pointer registers to Y and Z.
def LDDSTDPtrReg : Operand<i16>
{
    let MIOperandInfo = (ops PTRDISPREGS);
    let EncoderMethod = "encodeLDSTPtrReg";

    let ParserMatchClass = PtrRegAsmOperand;
}

//===----------------------------------------------------------------------===//
// OPEN8 predicates for subtarget features
//===----------------------------------------------------------------------===//

def HasNEW       :    Predicate<"Subtarget->hasNEW()">,
                         AssemblerPredicate<(all_of FeatureNEW)>;

def HasSRAM       :    Predicate<"Subtarget->hasSRAM()">,
                         AssemblerPredicate<(all_of FeatureSRAM)>;

def HasJMPCALL    :    Predicate<"Subtarget->hasJMPCALL()">,
                         AssemblerPredicate<(all_of FeatureJMPCALL)>;

def HasIJMPCALL   :    Predicate<"Subtarget->hasIJMPCALL()">,
                         AssemblerPredicate<(all_of FeatureIJMPCALL)>;

def HasEIJMPCALL  :    Predicate<"Subtarget->hasEIJMPCALL()">,
                         AssemblerPredicate<(all_of FeatureEIJMPCALL)>;

def HasADDSUBIW   :    Predicate<"Subtarget->hasADDSUBIW()">,
                         AssemblerPredicate<(all_of FeatureADDSUBIW)>;

def HasSmallStack :    Predicate<"Subtarget->HasSmallStack()">,
                         AssemblerPredicate<(all_of FeatureSmallStack)>;

def HasMOVW       :    Predicate<"Subtarget->hasMOVW()">,
                         AssemblerPredicate<(all_of FeatureMOVW)>;

def HasLPM        :    Predicate<"Subtarget->hasLPM()">,
                         AssemblerPredicate<(all_of FeatureLPM)>;

def HasLPMX       :    Predicate<"Subtarget->hasLPMX()">,
                         AssemblerPredicate<(all_of FeatureLPMX)>;

def HasELPM       :    Predicate<"Subtarget->hasELPM()">,
                         AssemblerPredicate<(all_of FeatureELPM)>;

def HasELPMX      :    Predicate<"Subtarget->hasELPMX()">,
                         AssemblerPredicate<(all_of FeatureELPMX)>;

def HasSPM        :    Predicate<"Subtarget->hasSPM()">,
                         AssemblerPredicate<(all_of FeatureSPM)>;

def HasSPMX       :    Predicate<"Subtarget->hasSPMX()">,
                         AssemblerPredicate<(all_of FeatureSPMX)>;

def HasDES        :    Predicate<"Subtarget->hasDES()">,
                         AssemblerPredicate<(all_of FeatureDES)>;

def SupportsRMW   :    Predicate<"Subtarget->supportsRMW()">,
                         AssemblerPredicate<(all_of FeatureRMW)>;

def SupportsMultiplication : Predicate<"Subtarget->supportsMultiplication()">,
                               AssemblerPredicate<(all_of FeatureMultiplication)>;

def HasBREAK      :    Predicate<"Subtarget->hasBREAK()">,
                         AssemblerPredicate<(all_of FeatureBREAK)>;

def HasTinyEncoding : Predicate<"Subtarget->hasTinyEncoding()">,
                        AssemblerPredicate<(all_of FeatureTinyEncoding)>;


// OPEN8 specific condition code. These correspond to OPEN8_*_COND in
// OPEN8InstrInfo.td. They must be kept in synch.
def OPEN8_COND_EQ : PatLeaf<(i8 0)>;
def OPEN8_COND_NE : PatLeaf<(i8 1)>;
def OPEN8_COND_GE : PatLeaf<(i8 2)>;
def OPEN8_COND_LT : PatLeaf<(i8 3)>;
def OPEN8_COND_SH : PatLeaf<(i8 4)>;
def OPEN8_COND_LO : PatLeaf<(i8 5)>;
def OPEN8_COND_MI : PatLeaf<(i8 6)>;
def OPEN8_COND_PL : PatLeaf<(i8 7)>;


//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// OPEN8 Instruction list
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//

// ADJCALLSTACKDOWN/UP implicitly use/def SP because they may be expanded into
// a stack adjustment and the codegen must know that they may modify the stack
// pointer before prolog-epilog rewriting occurs.
// Pessimistically assume ADJCALLSTACKDOWN / ADJCALLSTACKUP will become
// sub / add which can clobber SREG.
let Defs = [SP, SREG],
Uses = [SP] in
{
  def ADJCALLSTACKDOWN : Pseudo<(outs),
                                (ins i16imm:$amt, i16imm:$amt2),
                                "#ADJCALLSTACKDOWN",
                                [(OPEN8callseq_start timm:$amt, timm:$amt2)]>;

  // R5R4 is used to update SP, since it is a scratch reg and this instruction
  // is placed after the function call then R5R4 should be always free.
  //let Defs = [R5R4],
  //Uses = [R5R4] in
  //:TODO: if we enable this, the pseudo is killed because it looks dead
  def ADJCALLSTACKUP : Pseudo<(outs),
                              (ins i16imm:$amt1, i16imm:$amt2),
                              "#ADJCALLSTACKUP",
                              [(OPEN8callseq_end timm:$amt1, timm:$amt2)]>;
}

//===----------------------------------------------------------------------===//
// Addition
//===----------------------------------------------------------------------===//
let isCommutable = 1,
Constraints = "$src = $rd",
Defs = [SREG] in
{
  // ADD Rd, Rr
  // Adds two 8-bit registers.
  def ADDRdRr : Pseudo<
                      (outs GPR8:$rd),
                      (ins GPR8:$src, GPR8:$rr),
                      "add\t$rd, $rr",
                      [(set i8:$rd, (add i8:$src, i8:$rr)),
                       (implicit SREG)]>;

  // ADDW Rd+1:Rd, Rr+1:Rr
  // Pseudo instruction to add four 8-bit registers as two 16-bit values.
  //
  // Expands to:
  // add Rd,    Rr
  // adc Rd+1, Rr+1
  def ADDWRdRr : Pseudo<(outs DREGS:$rd),
                        (ins DREGS:$src, DREGS:$rr),
                        "addw\t$rd, $rr",
                        [(set i16:$rd, (add i16:$src, i16:$rr)),
                         (implicit SREG)]>;

  // ADC Rd, Rr
  // Adds two 8-bit registers with carry.
  let Uses = [SREG] in
  def ADCRdRr : Pseudo<(outs GPR8:$rd),
                      (ins GPR8:$src, GPR8:$rr),
                      "adc\t$rd, $rr",
                      [(set i8:$rd, (adde i8:$src, i8:$rr)),
                       (implicit SREG)]>;

  // ADCW Rd+1:Rd, Rr+1:Rr
  // Pseudo instruction to add four 8-bit registers as two 16-bit values with
  // carry.
  //
  // Expands to:
  // adc Rd,   Rr
  // adc Rd+1, Rr+1
  let Uses = [SREG] in
  def ADCWRdRr : Pseudo<(outs DREGS:$rd),
                        (ins DREGS:$src, DREGS:$rr),
                        "adcw\t$rd, $rr",
                        [(set i16:$rd, (adde i16:$src, i16:$rr)),
                         (implicit SREG)]>;

/*
  // AIDW Rd, k
  // Adds an immediate 6-bit value K to Rd, placing the result in Rd.
  def ADIWRdK : FWRdK<0b0,
                      (outs IWREGS:$rd),
                      (ins IWREGS:$src, imm_arith6:$k),
                      "adiw\t$rd, $k",
                      [(set i16:$rd, (add i16:$src, uimm6:$k)),
                       (implicit SREG)]>,
                Requires<[HasADDSUBIW]>;
*/
}

//===----------------------------------------------------------------------===//
// Subtraction
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd",
Defs = [SREG] in
{
  // SUB Rd, Rr
  // Subtracts the 8-bit value of Rr from Rd and places the value in Rd.
  def SUBRdRr : Pseudo<(outs GPR8:$rd),
                      (ins GPR8:$src, GPR8:$rr),
                      "sub\t$rd, $rr",
                      [(set i8:$rd, (sub i8:$src, i8:$rr)),
                       (implicit SREG)]>;

  // SUBW Rd+1:Rd, Rr+1:Rr
  // Subtracts two 16-bit values and places the result into Rd.
  //
  // Expands to:
  // sub Rd,   Rr
  // sbc Rd+1, Rr+1
  def SUBWRdRr : Pseudo<(outs DREGS:$rd),
                        (ins DREGS:$src, DREGS:$rr),
                        "subw\t$rd, $rr",
                        [(set i16:$rd, (sub i16:$src, i16:$rr)),
                         (implicit SREG)]>;

/*  def SUBIRdK : FRdK<0b0101,
                     (outs LD8:$rd),
                     (ins LD8:$src, imm_ldi8:$k),
                     "subi\t$rd, $k",
                     [(set i8:$rd, (sub i8:$src, imm:$k)),
                      (implicit SREG)]>;
*/
  // SUBIW Rd+1:Rd, K+1:K
  //
  // Expands to:
  // subi Rd,   K
  // sbci Rd+1, K+1
  def SUBIWRdK : Pseudo<(outs DREGS:$rd),
                        (ins DREGS:$src, i16imm:$rr),
                        "subiw\t$rd, $rr",
                        [(set i16:$rd, (sub i16:$src, imm:$rr)),
                         (implicit SREG)]>;

/*  def SBIWRdK : FWRdK<0b1,
                      (outs IWREGS:$rd),
                      (ins IWREGS:$src, imm_arith6:$k),
                      "sbiw\t$rd, $k",
                      [(set i16:$rd, (sub i16:$src, uimm6:$k)),
                       (implicit SREG)]>,
                Requires<[HasADDSUBIW]>;
*/
  // Subtract with carry operations which must read the carry flag in SREG.
  let Uses = [SREG] in
  {
    def SBCRdRr : Pseudo<(outs GPR8:$rd),
                        (ins GPR8:$src, GPR8:$rr),
                        "sbc\t$rd, $rr",
                        [(set i8:$rd, (sube i8:$src, i8:$rr)),
                         (implicit SREG)]>;

    // SBCW Rd+1:Rd, Rr+1:Rr
    //
    // Expands to:
    // sbc Rd,   Rr
    // sbc Rd+1, Rr+1
    def SBCWRdRr : Pseudo<(outs DREGS:$rd),
                          (ins DREGS:$src, DREGS:$rr),
                          "sbcw\t$rd, $rr",
                          [(set i16:$rd, (sube i16:$src, i16:$rr)),
                           (implicit SREG)]>;

/*    def SBCIRdK : FRdK<0b0100,
                       (outs LD8:$rd),
                       (ins LD8:$src, imm_ldi8:$k),
                       "sbci\t$rd, $k",
                       [(set i8:$rd, (sube i8:$src, imm:$k)),
                        (implicit SREG)]>;
*/
    // SBCIW Rd+1:Rd, K+1:K
    // sbci Rd,   K
    // sbci Rd+1, K+1
/*   def SBCIWRdK : Pseudo<(outs DREGS:$rd),
                          (ins DREGS:$src, i16imm:$rr),
                          "sbciw\t$rd, $rr",
                          [(set i16:$rd, (sube i16:$src, imm:$rr)),
                           (implicit SREG)]>;
*/  
  }
}

//===----------------------------------------------------------------------===//
// Increment and Decrement
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rn",
Defs = [SREG] in
{
  def INCRd : OPEN8OpReg<OP_INC,
                     (outs GPR8:$rn),
                     (ins GPR8:$src),
                     "inc\t$rn",
                     [(set i8:$rn,(add i8:$src, 1)),(implicit SREG)]>;

  def DECRd : OPEN8OpReg<OP_DEC,
                      (outs GPR8:$rn),
                      (ins GPR8:$src),
                      "dec\t$rn",
                      [(set i8:$rn,(add i8:$src, -1)),(implicit SREG)]>; 
}

//===----------------------------------------------------------------------===//
// Multiplication
//===----------------------------------------------------------------------===//

let isCommutable = 1,
Defs = [R1, R0, SREG] in
{
  // MUL Rd, Rr
  // Multiplies Rd by Rr and places the result into R1:R0.
  let usesCustomInserter = 1 in {
    def MULRdRr :   Pseudo<(outs),
                        (ins GPR8:$lhs, GPR8:$rhs),
                        "mul\t$lhs, $rhs",
                        [/*(set R1, R0, (smullohi i8:$lhs, i8:$rhs))*/]>,
                    Requires<[SupportsMultiplication]>;

    def MULSRdRr : Pseudo<(outs),
                             (ins LD8:$lhs, LD8:$rhs),
                             "muls\t$lhs, $rhs",
                             []>,
                   Requires<[SupportsMultiplication]>;
  }
}

//===----------------------------------------------------------------------===//
// Logic
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd",
Defs = [SREG] in
{
  // Register-Register logic instructions (which have the
  // property of commutativity).
  let isCommutable = 1 in
  {
    def ANDRdRr : Pseudo<(outs GPR8:$rd),
                        (ins GPR8:$src, GPR8:$rr),
                        "and\t$rd, $rr",
                        [(set i8:$rd, (and i8:$src, i8:$rr)),
                         (implicit SREG)]>;

    // ANDW Rd+1:Rd, Rr+1:Rr
    //
    // Expands to:
    // and Rd,   Rr
    // and Rd+1, Rr+1
    def ANDWRdRr : Pseudo<(outs DREGS:$rd),
                          (ins DREGS:$src, DREGS:$rr),
                          "andw\t$rd, $rr",
                          [(set i16:$rd, (and i16:$src, i16:$rr)),
                           (implicit SREG)]>;

    def ORRdRr : Pseudo<(outs GPR8:$rd),
                       (ins GPR8:$src, GPR8:$rr),
                       "or\t$rd, $rr",
                       [(set i8:$rd, (or i8:$src, i8:$rr)),
                        (implicit SREG)]>;

    // ORW Rd+1:Rd, Rr+1:Rr
    //
    // Expands to:
    // or Rd,   Rr
    // or Rd+1, Rr+1
    def ORWRdRr : Pseudo<(outs DREGS:$rd),
                         (ins DREGS:$src, DREGS:$rr),
                         "orw\t$rd, $rr",
                         [(set i16:$rd, (or i16:$src, i16:$rr)),
                          (implicit SREG)]>;

    def EORRdRr : Pseudo<(outs GPR8:$rd),
                        (ins GPR8:$src, GPR8:$rr),
                        "eor\t$rd, $rr",
                        [(set i8:$rd, (xor i8:$src, i8:$rr)),
                         (implicit SREG)]>;

    // EORW Rd+1:Rd, Rr+1:Rr
    //
    // Expands to:
    // eor Rd,   Rr
    // eor Rd+1, Rr+1
    def EORWRdRr : Pseudo<(outs DREGS:$rd),
                          (ins DREGS:$src, DREGS:$rr),
                          "eorw\t$rd, $rr",
                          [(set i16:$rd, (xor i16:$src, i16:$rr)),
                           (implicit SREG)]>;
  }
/*
  def ANDIRdK : FRdK<0b0111,
                     (outs LD8:$rd),
                     (ins LD8:$src, imm_ldi8:$k),
                     "andi\t$rd, $k",
                     [(set i8:$rd, (and i8:$src, imm:$k)),
                      (implicit SREG)]>;

  // ANDI Rd+1:Rd, K+1:K
  //
  // Expands to:
  // andi Rd,   K
  // andi Rd+1, K+1
  def ANDIWRdK : Pseudo<(outs DREGS:$rd),
                        (ins DREGS:$src, i16imm:$k),
                        "andiw\t$rd, $k",
                        [(set i16:$rd, (and i16:$src, imm:$k)),
                         (implicit SREG)]>;

  def ORIRdK : FRdK<0b0110,
                    (outs LD8:$rd),
                    (ins LD8:$src, imm_ldi8:$k),
                    "ori\t$rd, $k",
                    [(set i8:$rd, (or i8:$src, imm:$k)),
                     (implicit SREG)]>;

  // ORIW Rd+1:Rd, K+1,K
  //
  // Expands to:
  // ori Rd,   K
  // ori Rd+1, K+1
  def ORIWRdK : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src, i16imm:$rr),
                       "oriw\t$rd, $rr",
                       [(set i16:$rd, (or i16:$src, imm:$rr)),
                        (implicit SREG)]>;
*/
}

//===----------------------------------------------------------------------===//
// One's/Two's Complement
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd",
Defs = [SREG] in
{
  def COMRd : Pseudo<(outs GPR8:$rd),
                  (ins GPR8:$src),
                  "com\t$rd",
                  [(set i8:$rd, (not i8:$src)), (implicit SREG)]>;

  // COMW Rd+1:Rd
  //
  // Expands to:
  // com Rd
  // com Rd+1
  def COMWRd : Pseudo<(outs DREGS:$rd),
                      (ins DREGS:$src),
                      "comw\t$rd",
                      [(set i16:$rd, (not i16:$src)), (implicit SREG)]>;

  def NEGRd : Pseudo<(outs GPR8:$rd),
                  (ins GPR8:$src),
                  "neg\t$rd",
                  [(set i8:$rd, (ineg i8:$src)), (implicit SREG)]>;

  // NEGW Rd+1:Rd
  //
  // Expands to:
  // neg Rd+1
  // neg Rd
  // sbc Rd+1, r1
  def NEGWRd : Pseudo<(outs DREGS:$rd),
                      (ins DREGS:$src),
                      "negw\t$rd",
                      [(set i16:$rd, (ineg i16:$src)), (implicit SREG)]>;
}
//===----------------------------------------------------------------------===//
// Jump instructions
//===----------------------------------------------------------------------===//
let isBarrier = 1,
isBranch = 1,
isTerminator = 1 in
{

  def JMPk : OPEN8OpSelAddr<OP_INST,
                OP_JMP,
                (outs),
                (ins call_target:$addr),
                "jmp\t$addr",
                [(br bb:$addr)]>;
}

//===----------------------------------------------------------------------===//
// Call instructions
//===----------------------------------------------------------------------===//
let isCall = 1 in
{
  // SP is marked as a use to prevent stack-pointer assignments that appear
  // immediately before calls from potentially appearing dead.
  //
  //:TODO: the imm field can be either 16 or 22 bits in devices with more
  // than 64k of ROM, fix it once we support the largest devices.
  let Uses = [SP] in
  def JSRk : OPEN8OpSelAddr<OP_INST,
                OP_JSR,
                (outs),
                     (ins call_target:$addr),
                     "jsr\t$addr",
                     [(OPEN8call imm:$addr)]>,
              Requires<[HasJMPCALL]>;
}

//===----------------------------------------------------------------------===//
// Return instructions.
//===----------------------------------------------------------------------===//
let isTerminator = 1,
isReturn = 1,
isBarrier = 1 in
{
  def RTS : OPEN8OpSel<OP_INST,
                OP_RTS,
                (outs),
                (ins),
                "rts",
                [(OPEN8rtsflag)]>;

  def RTI : OPEN8OpSel<OP_INST,
                OP_RTI,
                (outs),
                 (ins),
                 "rti",
                 [(OPEN8rtiflag)]>;
}

//===----------------------------------------------------------------------===//
// Compare operations.
//===----------------------------------------------------------------------===//
let Defs = [SREG] in
{
  def CPRdRr : Pseudo<(outs),
                     (ins GPR8:$rd, GPR8:$rr),
                     "cp\t$rd, $rr",
                     [(OPEN8cmp i8:$rd, i8:$rr), (implicit SREG)]>;

  // CPW Rd+1:Rd, Rr+1:Rr
  //
  // Expands to:
  // cp  Rd,   Rr
  // cpc Rd+1, Rr+1
  def CPWRdRr : Pseudo<(outs),
                       (ins DREGS:$src, DREGS:$src2),
                       "cpw\t$src, $src2",
                       [(OPEN8cmp i16:$src, i16:$src2), (implicit SREG)]>;

  let Uses = [SREG] in
  def CPCRdRr : Pseudo<(outs),
                     (ins GPR8:$rd, GPR8:$rr),
                     "cpc\t$rd, $rr",
                     [(OPEN8cmpc i8:$rd, i8:$rr), (implicit SREG)]>;

  // CPCW Rd+1:Rd. Rr+1:Rr
  //
  // Expands to:
  // cpc Rd,   Rr
  // cpc Rd+1, Rr+1
  let Uses = [SREG] in
  def CPCWRdRr : Pseudo<(outs),
                        (ins DREGS:$src, DREGS:$src2),
                        "cpcw\t$src, $src2",
                        [(OPEN8cmpc i16:$src, i16:$src2), (implicit SREG)]>;

}

  // Relative branches on status flag bits.
  let Uses = [SREG] in
  {
    // BR0 s, k
    // Branch if `s` flag in status register is 0.
        def BR0 : OPEN8OpBitOff<OP_BR0,
                     (outs),
                     (ins i8imm:$bitIndex, relbrtarget_8:$offset),
                     "br0\t$bitIndex, $offset",
                     []>;

    // BR1 s, k
    // Branch if `s` flag in status register is 1.
        def BR1 : OPEN8OpBitOff<OP_BR1,
                     (outs),
                        (ins i8imm:$bitIndex, relbrtarget_8:$offset),
                        "br1\t$bitIndex, $offset",
                     []>;
  }

//===----------------------------------------------------------------------===//
// PC-relative conditional branches
//===----------------------------------------------------------------------===//
// Based on status register. We cannot simplify these into instruction aliases
// because we also need to be able to specify a pattern to match for ISel.
let isBranch = 1,
isTerminator = 1,
Uses = [SREG] in
{
  def BRZ : OPEN8OpBitOff2<OP_BR10,
                    (outs),
                    (ins relbrtarget_8:$offset),
                    "brz\t$offset",
                    [(OPEN8brcond bb:$offset, OPEN8_COND_EQ)]>;

  def BRNZ : OPEN8OpBitOff2<OP_BR00,
                    (outs),
                    (ins relbrtarget_8:$offset),
                    "brnz\t$offset",
                    [(OPEN8brcond bb:$offset, OPEN8_COND_NE)]>;

  def BRNC : OPEN8OpBitOff2<OP_BR01,
                    (outs),
                    (ins relbrtarget_8:$offset),
                    "brnc\t$offset",
                    [(OPEN8brcond bb:$offset, OPEN8_COND_SH)]>;

  def BRC : OPEN8OpBitOff2<OP_BR11,
                    (outs),
                    (ins relbrtarget_8:$offset),
                    "brc\t$offset",
                    [(OPEN8brcond bb:$offset, OPEN8_COND_LO)]>;

  def BRLZ : OPEN8OpBitOff2<OP_BR12,
                    (outs),
                    (ins relbrtarget_8:$offset),
                    "brlz\t$offset",
                    [(OPEN8brcond bb:$offset, OPEN8_COND_MI)]>;

  def BRGEZ : OPEN8OpBitOff2<OP_BR02,
                    (outs),
                    (ins relbrtarget_8:$offset),
                    "brgez\t$offset",
                    [(OPEN8brcond bb:$offset, OPEN8_COND_PL)]>;

  //TODO: add to feature, open8 dont support signed branch native, need doc
  def BRGE : OPEN8OpBitOff2<OP_BR17,
                    (outs),
                    (ins relbrtarget_8:$target),
                    "brge\t$target",
                    [(OPEN8brcond bb:$target, OPEN8_COND_GE)]>;

  def BRLT : OPEN8OpBitOff2<OP_BR07,
                    (outs),
                    (ins relbrtarget_8:$target),
                    "brlt\t$target",
                    [(OPEN8brcond bb:$target, OPEN8_COND_LT)]>;
}

//===----------------------------------------------------------------------===//
// Data transfer instructions
//===----------------------------------------------------------------------===//
// 8 and 16-bit register move instructions.
let hasSideEffects = 0 in
{
  def MOVRdRr : Pseudo<
                      (outs GPR8:$rd),
                      (ins GPR8:$rr),
                      "mov\t$rd, $rr",
                      []>;

  def MOVWRdRr : Pseudo<(outs DREGS:$dst),
                           (ins DREGS:$src),
                           "movw\t$dst, $src",
                           []>,
                 Requires<[HasMOVW]>;
}

// Load immediate values into registers.
let isReMaterializable = 1 in
{

   def LDIRdk : OPEN8OpRegOff<OP_LDI,
                     (outs LD8:$rn),
                     (ins imm_ldi8:$offset),
                     "ldi\t$rn, $offset",
                     [(set i8:$rn, imm:$offset)]>;
  // LDIW Rd+1:Rd, K+1:K
  //
  // Expands to:
  // ldi Rd,   K
  // ldi Rd+1, K+1
  def LDIWRdk : Pseudo<(outs DREGS:$dst),
                       (ins i16imm:$src),
                       "ldiw\t$dst, $src",
                       [(set i16:$dst, imm:$src)]>;
}

// Load from data into register.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{
  def LDARdk : OPEN8OpRegAddr<OP_LDA,
                     (outs GPR8:$rn),
                     (ins imm16:$addr),
                     "lda\t$rn, $addr",
                     [(set i8:$rn, (load imm:$addr))]>;

  // LDAW Rd+1:Rd, K+1:K
  //
  // Expands to:
  // lds Rd,  (K+1:K)
  // lds Rd+1 (K+1:K) + 1
  def LDAWRdk : Pseudo<(outs DREGS:$dst),
                       (ins i16imm:$src),
                       "ldaw\t$dst, $src",
                       [(set i16:$dst, (load imm:$src))]>;
}

// Indirect loads.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{
  def LDRd : Pseudo<(outs GPR8:$reg),
                      (ins LDSTPtrReg:$ptrreg),
                      "ld\t$reg, $ptrreg",
                      [(set GPR8:$reg, (load i16:$ptrreg))]>,
                Requires<[HasSRAM]>;

  // LDW Rd+1:Rd, P
  //
  // Expands to:
  // ld  Rd,   P
  // ldd Rd+1, P+1
  let Constraints = "@earlyclobber $reg" in
  def LDWRd : Pseudo<(outs DREGS:$reg),
                        (ins PTRDISPREGS:$ptrreg),
                        "ldw\t$reg, $ptrreg",
                        [(set i16:$reg, (load i16:$ptrreg))]>,
                 Requires<[HasSRAM]>;
}

// Indirect loads (with postincrement or predecrement).
let mayLoad = 1,
hasSideEffects = 0,
Constraints = "$ptrreg = $base_wb,@earlyclobber $reg" in
{
  def LDRdPi : Pseudo<(outs GPR8:$reg, PTRREGS:$base_wb),
                        (ins LDSTPtrReg:$ptrreg),
                        "ld\t$reg, $ptrreg+",
                        []>,
                  Requires<[HasSRAM]>;

  // LDW Rd+1:Rd, P+
  // Expands to:
  // ld Rd,   P+
  // ld Rd+1, P+
  def LDWRdPi : Pseudo<(outs DREGS:$reg, PTRREGS:$base_wb),
                          (ins PTRREGS:$ptrreg),
                          "ldw\t$reg, $ptrreg+",
                          []>,
                   Requires<[HasSRAM]>;

}

// Load indirect with displacement operations.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{
  def LDO : OPEN8OpRegOffX<0,OP_LDO,
                      (outs),
                      (ins DREGS:$rn, i8imm:$offset),
                      "ldo\t$rn, $offset",
                      []>;  

  def LDOinc : OPEN8OpRegOffX<1,OP_LDO,
                      (outs),
                      (ins DREGS:$rn, i8imm:$offset),
                      !strconcat("lto\t$rn","++"," ,$offset"),
                      []>;   //TODO: add to disassembler 

  let Constraints = "@earlyclobber $reg" in
  def LDDRdQ : Pseudo< (outs GPR8:$reg),
                          (ins memri:$memri),
                          "ldd\t$reg, $memri",
                          [(set i8:$reg, (load addr:$memri))]>,
                  Requires<[HasSRAM]>;

  // LDDW Rd+1:Rd, P+q
  //
  // Expands to:
  // ldd Rd,   P+q
  // ldd Rd+1, P+q+1
  let Constraints = "@earlyclobber $dst" in
  def LDDWRdQ : Pseudo<(outs DREGS:$dst),
                          (ins memri:$memri),
                          "lddw\t$dst, $memri",
                          [(set i16:$dst, (load addr:$memri))]>,
                   Requires<[HasSRAM]>;

  // An identical pseudo instruction to LDDWRdPtrQ, expect restricted to the Y
  // register and without the @earlyclobber flag.
  //
  // Used to work around a bug caused by the register allocator not
  // being able to handle the expansion of a COPY into an machine instruction
  // that has an earlyclobber flag. This is because the register allocator will
  // try expand a copy from a register slot into an earlyclobber instruction.
  // Instructions that are earlyclobber need to be in a dedicated earlyclobber slot.
  //
  // This pseudo instruction can be used pre-OPEN8 pseudo expansion in order to
  // get a frame index load without directly using earlyclobber instructions.
  //
  // The pseudo expansion pass trivially expands this into LDDWRdQ.
  //
  // This instruction may be removed once PR13375 is fixed.
  let mayLoad = 1,
  hasSideEffects = 0 in
  def LDDWRdYQ : Pseudo<(outs DREGS:$dst),
                        (ins memri:$memri),
                        "lddw\t$dst, $memri",
                        []>,
                 Requires<[HasSRAM]>;
}

// Indirect store from register to data
def STAKRr : OPEN8OpRegAddr<OP_STA,
                   (outs),
                   (ins imm16:$addr, GPR8:$rn),
                   "sta\t$rn, $addr",
                   [(store i8:$rn, imm:$addr)]>;

// STAW K+1:K, Rr+1:Rr
//
// Expands to:
// sta Rr+1, (K+1:K) + 1
// sta Rr,   (K+1:K)
def STAWKRr : Pseudo<(outs),
                     (ins i16imm:$dst, DREGS:$src),
                     "staw\t$dst, $src",
                     [(store i16:$src, imm:$dst)]>;

// Indirect stores.
// ST P, Rr
// Stores the value of Rr into the location addressed by pointer P.
def STRr : Pseudo<(outs),
                    (ins DREGS:$ptrreg, GPR8:$reg),
                    "st\t$ptrreg, $reg",
                    [(store GPR8:$reg, i16:$ptrreg)]>;

// STW P, Rr+1:Rr
// Stores the value of Rr into the location addressed by pointer P.
//
// Expands to:
// st P, Rr
// std P+1, Rr+1
def STWRr : Pseudo<(outs),
                      (ins PTRDISPREGS:$ptrreg, DREGS:$reg),
                      "stw\t$ptrreg, $reg",
                      [(store i16:$reg, i16:$ptrreg)]>;

// Indirect stores (with postincrement or predecrement).
let Constraints = "$ptrreg = $base_wb,@earlyclobber $base_wb" in
{

  // ST P+, Rr
  // Stores the value of Rr into the location addressed by pointer P.
  // Post increments P.
  def STPiRr : Pseudo<(outs PTRREGS:$base_wb),
                        (ins PTRREGS:$ptrreg, GPR8:$reg, i8imm:$offs),
                        "st\t$ptrreg+, $reg",
                        [(set i16:$base_wb,
                         (post_store GPR8:$reg, i16:$ptrreg, imm:$offs))]>,
                  Requires<[HasSRAM]>;

  // STW P+, Rr+1:Rr
  // Stores the value of Rr into the location addressed by pointer P.
  // Post increments P.
  //
  // Expands to:
  // st P+, Rr
  // st P+, Rr+1
  def STWPiRr : Pseudo<(outs PTRREGS:$base_wb),
                          (ins PTRREGS:$ptrreg, DREGS:$trh, i8imm:$offs),
                          "stw\t$ptrreg+, $trh",
                          [(set PTRREGS:$base_wb,
                           (post_store DREGS:$trh, PTRREGS:$ptrreg, imm:$offs))]>;

}


def STO : OPEN8OpRegOffX<0, OP_STO,
                     (outs),
                     (ins DREGS:$rn, i8imm:$offset),
                     "sto\t$rn, $offset",
                     []>; 

def STOinc : OPEN8OpRegOffX<1, OP_STO,
                     (outs),
                     (ins DREGS:$rn, i8imm:$offset),
                     !strconcat("sto\t$rn","++"," ,$offset"),
                     []>;  //TODO: add to disassembler 

// Store indirect with displacement operations.
// STD P+q, Rr
// Stores the value of Rr into the location addressed by pointer P with a
// displacement of q. Does not modify P.

def STDQRr : Pseudo< (outs),
                        (ins memri:$memri, GPR8:$reg),
                        "std\t$memri, $reg",
                        [(store i8:$reg, addr:$memri)]>;

// STDW P+q, Rr+1:Rr
// Stores the value of Rr into the location addressed by pointer P with a
// displacement of q. Does not modify P.
//
// Expands to:
// std P+q,   Rr
// std P+q+1, Rr+1
def STDWQRr : Pseudo<(outs),
                        (ins memri:$memri, DREGS:$src),
                        "stdw\t$memri, $src",
                        [(store i16:$src, addr:$memri)]>;

// Stack push/pop operations.
let Defs = [SP],
Uses = [SP],
hasSideEffects = 0 in
{
  // Stack push operations.
  let mayStore = 1 in
  {
    def PUSHRr : OPEN8OpReg<OP_PSH,
                     (outs),
                     (ins GPR8:$rn),
                     "psh\t$rn",
                     []>;

    def PUSHWRr : Pseudo<(outs),
                         (ins DREGS:$reg),
                         "pshw\t$reg",
                         []>;
  }

  // Stack pop operations.
  let mayLoad = 1 in
  {
    def POPRd : OPEN8OpReg<OP_POP,
                    (outs GPR8:$rn),
                    (ins),
                    "pop\t$rn",
                    []>;

    def POPWRd : Pseudo<(outs DREGS:$reg),
                        (ins),
                        "popw\t$reg",
                        []>;
  }
}

//===----------------------------------------------------------------------===//
// Bit and bit-test instructions
//===----------------------------------------------------------------------===//

// Bit shift/rotate operations.
let Constraints = "$src = $rd",
Defs = [SREG] in
{
  // 8-bit LSL is an alias of ADD Rd, Rd

  def LSLWRd : Pseudo<(outs DREGS:$rd),
                      (ins DREGS:$src),
                      "lslw\t$rd",
                      [(set i16:$rd, (OPEN8lsl i16:$src)), (implicit SREG)]>;

  def LSLB7Rd : Pseudo<(outs GPR8:$rd),
                       (ins GPR8:$src),
                       "lslb7\t$rd",
                       [(set i8:$rd, (OPEN8lsl7 i8:$src)), (implicit SREG)]>;

  def LSLW4Rd : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src),
                       "lslw4\t$rd",
                       [(set i16:$rd, (OPEN8lsl4 i16:$src)), (implicit SREG)]>;

  def LSLW8Rd : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src),
                       "lslw8\t$rd",
                       [(set i16:$rd, (OPEN8lsl8 i16:$src)), (implicit SREG)]>;

  def LSLW12Rd : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src),
                       "lslw12\t$rd",
                       [(set i16:$rd, (OPEN8lsl12 i16:$src)), (implicit SREG)]>;

  def LSRRd : Pseudo<(outs GPR8:$rd),
                  (ins GPR8:$src),
                  "lsr\t$rd",
                  [(set i8:$rd, (OPEN8lsr i8:$src)), (implicit SREG)]>;

  def LSRB7Rd : Pseudo<(outs GPR8:$rd),
                       (ins GPR8:$src),
                       "lsrb7\t$rd",
                       [(set i8:$rd, (OPEN8lsr7 i8:$src)), (implicit SREG)]>;

  def LSRWRd : Pseudo<(outs DREGS:$rd),
                      (ins DREGS:$src),
                      "lsrw\t$rd",
                      [(set i16:$rd, (OPEN8lsr i16:$src)), (implicit SREG)]>;

  def LSRW4Rd : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src),
                       "lsrw4\t$rd",
                       [(set i16:$rd, (OPEN8lsr4 i16:$src)), (implicit SREG)]>;

  def LSRW8Rd : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src),
                       "lsrw8\t$rd",
                       [(set i16:$rd, (OPEN8lsr8 i16:$src)), (implicit SREG)]>;

  def LSRW12Rd : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src),
                       "lsrw12\t$rd",
                       [(set i16:$rd, (OPEN8lsr12 i16:$src)), (implicit SREG)]>;

  def ASRRd : Pseudo<(outs GPR8:$rd),
                  (ins GPR8:$src),
                  "asr\t$rd",
                  [(set i8:$rd, (OPEN8asr i8:$src)), (implicit SREG)]>;

  def ASRB7Rd : Pseudo<(outs GPR8:$rd),
                       (ins GPR8:$src),
                       "asrb7\t$rd",
                       [(set i8:$rd, (OPEN8asr7 i8:$src)), (implicit SREG)]>;

  def ASRWRd : Pseudo<(outs DREGS:$rd),
                      (ins DREGS:$src),
                      "asrw\t$rd",
                      [(set i16:$rd, (OPEN8asr i16:$src)), (implicit SREG)]>;

  def ASRW8Rd : Pseudo<(outs DREGS:$rd),
                       (ins DREGS:$src),
                       "asrw8\t$rd",
                       [(set i16:$rd, (OPEN8asr8 i16:$src)), (implicit SREG)]>;

  // Bit rotate operations.
  let Uses = [SREG] in
  {
    // 8-bit ROL is an alias of ADC Rd, Rd

    def ROLBRd : Pseudo<(outs GPR8:$rd),
                        (ins GPR8:$src),
                        "rolb\t$rd",
                        [(set i8:$rd, (OPEN8rol i8:$src)), (implicit SREG)]>;

    def RORBRd : Pseudo<(outs GPR8:$rd),
                        (ins GPR8:$src),
                        "rorb\t$rd",
                        [(set i8:$rd, (OPEN8ror i8:$src)), (implicit SREG)]>;

    def ROLWRd : Pseudo<(outs DREGS:$rd),
                        (ins DREGS:$src),
                        "rolw\t$rd",
                        [(set i16:$rd, (OPEN8rol i16:$src)), (implicit SREG)]>;

    def RORRd : Pseudo<(outs GPR8:$rd),
                    (ins GPR8:$src),
                    "ror\t$rd",
                    []>;

    def RORWRd : Pseudo<(outs DREGS:$rd),
                        (ins DREGS:$src),
                        "rorw\t$rd",
                        [(set i16:$rd, (OPEN8ror i16:$src)), (implicit SREG)]>;
  }
}

// SWAP Rd
// Swaps the high and low nibbles in a register.
// TODO: maybe remove
let Constraints = "$src = $rd" in
def SWAPRd : Pseudo<(outs GPR8:$rd),
                 (ins GPR8:$src),
                 "swap\t$rd",
                 [(set i8:$rd, (OPEN8Swap i8:$src))]>;

// LSL Rd
// Alias for ADD Rd, Rd
// --------------
// Logical shift left one bit.
def LSL1 : InstAlias<"lsl\t$rd", (ADDRdRr GPR8:$rd, GPR8:$rd)>;

def ROL1 : InstAlias<"rol\t$rd", (ADCRdRr GPR8:$rd, GPR8:$rd)>;

// SER Rd
// Alias for LDI Rd, 0xff
// ---------
// Sets all bits in a register.
def : InstAlias<"ser\t$rd", (LDIRdk LD8:$rd, 0xff), 0>;

let Defs = [SREG] in
def STP : OPEN8OpN<OP_STP,
               (outs),
               (ins i8imm:$n),
               "stp\t$n",
               []>;

let Defs = [SREG] in
def CLP : OPEN8OpN<OP_CLP,
               (outs),
               (ins i8imm:$n),
               "clp\t$n",
               []>;

// Set/clear aliases for the zero (Z) status flag (bit 0). 
def : InstAlias<"stz", (STP 0)>;
def : InstAlias<"clz", (CLP 0)>;

// Set/clear aliases for the carry (C) status flag (bit 1).
def : InstAlias<"stc", (STP 1)>;
def : InstAlias<"clc", (CLP 1)>;

// Set/clear aliases for the negative (N) status flag (bit 2).
def : InstAlias<"stn", (STP 2)>;
def : InstAlias<"cln", (CLP 2)>;

// Set/clear aliases for the interrupt (I) status flag (bit 3).
def : InstAlias<"sti", (STP 3)>;
def : InstAlias<"cli", (CLP 3)>;

// Set/clear aliases for the signed overflow (v) status flag (bit 4).
def : InstAlias<"stv", (STP 4)>;
def : InstAlias<"clv", (CLP 4)>;

//===----------------------------------------------------------------------===//
// Special/Control instructions
//===----------------------------------------------------------------------===//

def BRK : OPEN8OpSel<OP_INST,
              OP_BRK,
              (outs),
              (ins),
              "brk",
              []>;

//===----------------------------------------------------------------------===//
// Pseudo instructions for later expansion
//===----------------------------------------------------------------------===//

//:TODO: Optimize this for wider types AND optimize the following code
//       compile int foo(char a, char b, char c, char d) {return d+b;}
//       looks like a missed sext_inreg opportunity.
def SEXT : ExtensionPseudo<
  (outs DREGS:$dst),
  (ins GPR8:$src),
  "sext\t$dst, $src",
  [(set i16:$dst, (sext i8:$src)), (implicit SREG)]
>;

def ZEXT : ExtensionPseudo<
  (outs DREGS:$dst),
  (ins GPR8:$src),
  "zext\t$dst, $src",
  [(set i16:$dst, (zext i8:$src)), (implicit SREG)]
>;

// This pseudo gets expanded into a movw+adiw thus it clobbers SREG.
let Defs = [SREG],
    hasSideEffects = 0 in
def FRMIDX : Pseudo<(outs DREGS:$dst),
                    (ins DREGS:$src, i16imm:$src2),
                    "frmidx\t$dst, $src, $src2",
                    []>;

// This pseudo is either converted to a regular store or a push which clobbers
// SP.
def STDSPQRr : StorePseudo<
  (outs),
  (ins memspi:$dst, GPR8:$src),
  "stdstk\t$dst, $src",
  [(store i8:$src, addr:$dst)]
>;

// This pseudo is either converted to a regular store or a push which clobbers
// SP.
def STDWSPQRr : StorePseudo<
  (outs),
  (ins memspi:$dst, DREGS:$src),
  "stdwstk\t$dst, $src",
  [(store i16:$src, addr:$dst)]
>;

// SP read/write pseudos.
let hasSideEffects = 0 in
{
  let Uses = [SP] in
  def SPREAD : Pseudo<
    (outs DREGS:$dst),
    (ins GPRSP:$src),
    "spread\t$dst, $src",
    []
  >;

  let Defs = [SP] in
  def SPWRITE : Pseudo<
    (outs GPRSP:$dst),
    (ins DREGS:$src),
    "spwrite\t$dst, $src",
    []>;
}

def Select8 : SelectPseudo<
  (outs GPR8:$dst),
  (ins GPR8:$src, GPR8:$src2, i8imm:$cc),
  "# Select8 PSEUDO",
  [(set i8:$dst, (OPEN8selectcc i8:$src, i8:$src2, imm:$cc))]
>;

def Select16 : SelectPseudo<
  (outs DREGS:$dst),
  (ins DREGS:$src, DREGS:$src2, i8imm:$cc),
  "# Select16 PSEUDO",
  [(set i16:$dst, (OPEN8selectcc i16:$src, i16:$src2, imm:$cc))]
>;

def Lsl8 : ShiftPseudo<
  (outs GPR8:$dst),
  (ins GPR8:$src, GPR8:$cnt),
  "# Lsl8 PSEUDO",
  [(set i8:$dst, (OPEN8lslLoop i8:$src, i8:$cnt))]
>;

def Lsl16 : ShiftPseudo<
  (outs DREGS:$dst),
  (ins DREGS:$src, GPR8:$cnt),
  "# Lsl16 PSEUDO",
  [(set i16:$dst, (OPEN8lslLoop i16:$src, i8:$cnt))]
>;

def Lsr8 : ShiftPseudo<
  (outs GPR8:$dst),
  (ins GPR8:$src, GPR8:$cnt),
  "# Lsr8 PSEUDO",
  [(set i8:$dst, (OPEN8lsrLoop i8:$src, i8:$cnt))]
>;

def Lsr16 : ShiftPseudo<
  (outs DREGS:$dst),
   (ins DREGS:$src, GPR8:$cnt),
   "# Lsr16 PSEUDO",
   [(set i16:$dst, (OPEN8lsrLoop i16:$src, i8:$cnt))]
>;

def Rol8 : ShiftPseudo<
  (outs GPR8:$dst),
  (ins GPR8:$src, GPR8:$cnt),
  "# Rol8 PSEUDO",
  [(set i8:$dst, (OPEN8rolLoop i8:$src, i8:$cnt))]
>;

def Rol16 : ShiftPseudo<
  (outs DREGS:$dst),
  (ins DREGS:$src, GPR8:$cnt),
  "# Rol16 PSEUDO",
  [(set i16:$dst, (OPEN8rolLoop i16:$src, i8:$cnt))]
>;

def Ror8 : ShiftPseudo<
  (outs GPR8:$dst),
  (ins GPR8:$src, GPR8:$cnt),
  "# Ror8 PSEUDO",
  [(set i8:$dst, (OPEN8rorLoop i8:$src, i8:$cnt))]
>;

def Ror16 : ShiftPseudo<
  (outs DREGS:$dst),
  (ins DREGS:$src, GPR8:$cnt),
  "# Ror16 PSEUDO",
  [(set i16:$dst, (OPEN8rorLoop i16:$src, i8:$cnt))]
>;

def Asr8 : ShiftPseudo<
  (outs GPR8:$dst),
  (ins GPR8:$src, GPR8:$cnt),
  "# Asr8 PSEUDO",
  [(set i8:$dst, (OPEN8asrLoop i8:$src, i8:$cnt))]
>;

def Asr16 : ShiftPseudo<
  (outs DREGS:$dst),
   (ins DREGS:$src, GPR8:$cnt),
   "# Asr16 PSEUDO",
   [(set i16:$dst, (OPEN8asrLoop i16:$src, i8:$cnt))]
>;


//===----------------------------------------------------------------------===//
// Non-Instruction Patterns
//===----------------------------------------------------------------------===//

//:TODO: look in x86InstrCompiler.td for odd encoding trick related to
// add x, 128 -> sub x, -128. Clang is emitting an eor for this (ldi+eor)

// the add instruction always writes the carry flag
def : Pat<(addc i8:$src, i8:$src2),
          (ADDRdRr i8:$src, i8:$src2)>;
def : Pat<(addc DREGS:$src, DREGS:$src2),
          (ADDWRdRr DREGS:$src, DREGS:$src2)>;

// all sub instruction variants always writes the carry flag
def : Pat<(subc i8:$src, i8:$src2),
          (SUBRdRr i8:$src, i8:$src2)>;
def : Pat<(subc i16:$src, i16:$src2),
          (SUBWRdRr i16:$src, i16:$src2)>;

// These patterns convert add (x, -imm) to sub (x, imm) since we dont have
// any add with imm instructions. Also take care of the adiw/sbiw instructions.
/*def : Pat<(add i16:$src1, imm0_63_neg:$src2),
          (SBIWRdK i16:$src1, (imm0_63_neg:$src2))>;
def : Pat<(add i16:$src1, imm:$src2),
          (SUBIWRdK i16:$src1, (imm16_neg_XFORM imm:$src2))>;
def : Pat<(addc i16:$src1, imm:$src2),
          (SUBIWRdK i16:$src1, (imm16_neg_XFORM imm:$src2))>;

def : Pat<(add i8:$src1, imm:$src2),
          (SUBIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;
def : Pat<(addc i8:$src1, imm:$src2),
          (SUBIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;
def : Pat<(adde i8:$src1, imm:$src2),
          (SBCIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;*/

// Calls.
def : Pat<(OPEN8call (i16 tglobaladdr:$dst)),
          (JSRk tglobaladdr:$dst)>;
def : Pat<(OPEN8call (i16 texternalsym:$dst)),
          (JSRk texternalsym:$dst)>;

// `anyext`
def : Pat<(i16 (anyext i8:$src)),
          (INSERT_SUBREG (i16 (IMPLICIT_DEF)), i8:$src, sub_lo)>;

// `trunc`
def : Pat<(i8 (trunc i16:$src)),
          (EXTRACT_SUBREG i16:$src, sub_lo)>;

// sext_inreg
def : Pat<(sext_inreg i16:$src, i8),
          (SEXT (i8 (EXTRACT_SUBREG i16:$src, sub_lo)))>;

// GlobalAddress
def : Pat<(i16 (OPEN8Wrapper tglobaladdr:$dst)),
          (LDIWRdk tglobaladdr:$dst)>;
/*
def : Pat<(add i16:$src, (OPEN8Wrapper tglobaladdr:$src2)),
          (SUBIWRdK i16:$src, tglobaladdr:$src2)>;
*/
def : Pat<(i8 (load (OPEN8Wrapper tglobaladdr:$dst))),
          (LDARdk tglobaladdr:$dst)>;
def : Pat<(i16 (load (OPEN8Wrapper tglobaladdr:$dst))),
          (LDAWRdk tglobaladdr:$dst)>;
def : Pat<(store i8:$src, (i16 (OPEN8Wrapper tglobaladdr:$dst))),
          (STAKRr tglobaladdr:$dst, i8:$src)>;
def : Pat<(store i16:$src, (i16 (OPEN8Wrapper tglobaladdr:$dst))),
          (STAWKRr tglobaladdr:$dst, i16:$src)>;

// BlockAddress
def : Pat<(i16 (OPEN8Wrapper tblockaddress:$dst)),
          (LDIWRdk tblockaddress:$dst)>;

def : Pat<(i8 (trunc (OPEN8lsr8 DREGS:$src))),
          (EXTRACT_SUBREG DREGS:$src, sub_hi)>;

// :FIXME: DAGCombiner produces an shl node after legalization from these seq:
// BR_JT -> (mul x, 2) -> (shl x, 1)
def : Pat<(shl i16:$src1, (i8 1)),
          (LSLWRd i16:$src1)>;

// Lowering of 'tst' node to 'TST' instruction.
// TST is an alias of AND Rd, Rd.
def : Pat<(OPEN8tst i8:$rd),
          (ANDRdRr GPR8:$rd, GPR8:$rd)>;

// Lowering of 'lsl' node to 'LSL' instruction.
// LSL is an alias of 'ADD Rd, Rd'
def : Pat<(OPEN8lsl i8:$rd),
          (ADDRdRr GPR8:$rd, GPR8:$rd)>;

//OPEN8 INSTRUCTIONS
let Constraints = "$src = $rn",
Defs = [SREG] in
{
  //===----------------------------------------------------------------------===//
  // Increment and Decrement
  //===----------------------------------------------------------------------===//

  def ROL : OPEN8OpReg<OP_ROL,
                     (outs GPR8:$rn),
                     (ins GPR8:$src),
                     "rol\t$rn",
                     [(set i8:$rn, (OPEN8rol i8:$src)), (implicit SREG)]>;  

  def ROR : OPEN8OpReg<OP_ROR,
                     (outs GPR8:$rn),
                     (ins GPR8:$src),
                     "ror\t$rn",
                     [(set i8:$rn, (OPEN8ror i8:$src)), (implicit SREG)]>; 

}

let Defs = [R0, SREG] in
{
  def ADC : OPEN8OpReg<OP_ADC,
                     (outs),
                     (ins GPR8:$rn),
                     "adc\t$rn",
                     [(set R0,(adde i8:$rn, R0)),(implicit SREG)]>;

def TX0 : OPEN8OpReg<OP_TX0,
                     (outs),
                     (ins GPR8:$rn),
                     "tx0\t$rn",
                     [(set R0,(adde i8:$rn, 0)),(implicit SREG)]>;

 def SBC : OPEN8OpReg<OP_SBC,
                     (outs),
                     (ins GPR8:$rn),
                     "sbc\t$rn",
                     [(set R0,(sub i8:$rn, R0)),(implicit SREG)]>; 

 def ADD : OPEN8OpReg<OP_ADD,
                     (outs),
                     (ins GPR8:$rn),
                     "add\t$rn",
                     [(set R0,(add i8:$rn, R0)),(implicit SREG)]>; 

 def T0X : OPEN8OpReg<OP_T0X,
                     (outs),
                     (ins GPR8:$rn),
                     "t0x\t$rn",
                     [(set i8:$rn,(add R0, 0)),(implicit SREG)]>; 

  def CMP : OPEN8OpReg<OP_CMP,
                     (outs),
                     (ins GPR8:$rn),
                     "cmp\t$rn",
                     [(OPEN8cmp R0, i8:$rn), (implicit SREG)]>; 

  def OR : OPEN8OpReg<OP_OR,
                     (outs),
                     (ins GPR8:$rn),
                     "or\t$rn",
                     [(set R0,(or i8:$rn, R0)),(implicit SREG)]>; 

  def AND : OPEN8OpReg<OP_AND,
                     (outs),
                     (ins GPR8:$rn),
                     "and\t$rn",
                     [(set R0,(and i8:$rn, R0)),(implicit SREG)]>; 

  def XOR : OPEN8OpReg<OP_XOR,
                     (outs),
                     (ins GPR8:$rn),
                     "xor\t$rn",
                     [(set R0,(xor i8:$rn, R0)),(implicit SREG)]>; 
                     
}




  def BTT : OPEN8OpN<OP_BTT,
                (outs),
                (ins i8imm:$n),
                "btt\t$n",
                []>;

  def DBNZ : OPEN8OpRegOff<OP_DBNZ,
                     (outs),
                     (ins GPR8:$rn, i8imm:$offset),
                     "dbnz\t$rn, $offset",
                     []>; 

  def INT : OPEN8OpN<OP_INT,
                (outs),
                (ins i8imm:$n),
                "int\t$n",
                []>;

  def MUL : OPEN8OpReg<OP_MUL,
                     (outs),
                     (ins GPR8:$rn),
                     "mul\t$rn",
                     []>; 

  def RSP : OPEN8OpSel<OP_INST,
                OP_RSP,
                (outs),
                (ins),
                "rsp",
                []>;

  def SMSK : OPEN8OpSel<OP_INST,
                OP_SMSK,
                (outs),
                (ins),
                "smsk",
                []>;

  def GMSK : OPEN8OpSel<OP_INST,
                OP_GMSK,
                (outs),
                (ins),
                "gmsk",
                []>;

  def UPP : OPEN8OpReg<OP_UPP,
                     (outs),
                     (ins DREGS:$rn),
                     "upp\t$rn",
                     [(set R1R0,(add R1R0, i16:$rn)),(implicit SREG)]>; 

  def STX : OPEN8OpRegX<0,OP_STX,
                     (outs),
                     (ins DREGS:$rn),
                     "stx\t$rn",
                     []>; 

  def STXinc : OPEN8OpRegX<1,OP_STX,
                     (outs),
                     (ins DREGS:$rn),
                     !strconcat("stx\t$rn","++"),
                     []>;
  

  def LDX : OPEN8OpRegX<0,OP_LDX,
                     (outs),
                     (ins DREGS:$rn),
                     "ldx\t$rn",
                     []>; 

  def LDXinc : OPEN8OpRegX<1,OP_LDX,
                     (outs),
                     (ins DREGS:$rn),
                     !strconcat("ldx\t$rn","++"),
                     []>;  //TODO: add to disassembler 

  def JMPZ : Pseudo<(outs),
                         (ins call_target:$addr),
                         "jmpz\t$addr",
                         []>;

  def JMPNZ : Pseudo<(outs),
                         (ins call_target:$addr),
                         "jmpnz\t$addr",
                         []>;
  
  def JMPLZ : Pseudo<(outs),
                         (ins call_target:$addr),
                         "jmplz\t$addr",
                         []>;

  def JMPGEZ : Pseudo<(outs),
                         (ins call_target:$addr),
                         "jmpgez\t$addr",
                         []>;

  def JMPC : Pseudo<(outs),
                         (ins call_target:$addr),
                         "jmpc\t$addr",
                         []>;
  
  def JMPNC : Pseudo<(outs),
                         (ins call_target:$addr),
                         "jmpnc\t$addr",
                         []>;
                         
def : InstAlias<"nop", (BRK)>;

def CLR : InstAlias<"clr", (XOR R0)>;
