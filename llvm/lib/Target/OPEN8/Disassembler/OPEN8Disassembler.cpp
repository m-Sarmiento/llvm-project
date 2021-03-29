//===- OPEN8Disassembler.cpp - Disassembler for OPEN8 ---------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the OPEN8 Disassembler.
//
//===----------------------------------------------------------------------===//

#include "OPEN8.h"
#include "OPEN8RegisterInfo.h"
#include "OPEN8Subtarget.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"
#include "TargetInfo/OPEN8TargetInfo.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "open8-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// A disassembler class for OPEN8.
class OPEN8Disassembler : public MCDisassembler {
public:
  OPEN8Disassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  virtual ~OPEN8Disassembler() {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;
};
}

static MCDisassembler *createOPEN8Disassembler(const Target &T,
                                             const MCSubtargetInfo &STI,
                                             MCContext &Ctx) {
  return new OPEN8Disassembler(STI, Ctx);
}


extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeOPEN8Disassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheOPEN8Target(),
                                         createOPEN8Disassembler);
}

static const uint16_t GPRDecoderTable[] = {
  OPEN8::R0, OPEN8::R1, OPEN8::R2, OPEN8::R3,
  OPEN8::R4, OPEN8::R5, OPEN8::R6, OPEN8::R7
};

static const unsigned DREGSRegsTable[] = {
OPEN8::R1R0, OPEN8::R3R2, OPEN8::R5R4, OPEN8::R7R6
};

static DecodeStatus DecodeGPR8RegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address, const void *Decoder) {
  if (RegNo > 7)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeLD8RegisterClass(MCInst &Inst, unsigned RegNo,
                                           uint64_t Address, const void *Decoder) {
  if (RegNo > 3)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo+4];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

/*static DecodeStatus DecodePTRREGSRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address, const void *Decoder) {
  // Note: this function must be defined but does not seem to be called.
  assert(false && "unimplemented: PTRREGS register class");
  return MCDisassembler::Success;
}*/

static DecodeStatus DecodeDREGSRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address, const void *Decoder) {
  if (RegNo > 7 || RegNo%2 != 0)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(DREGSRegsTable[RegNo/2]));
  return MCDisassembler::Success;
}

/*static DecodeStatus DecodeImm8(MCInst &Inst, unsigned Value,
                                               uint64_t Address, const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(Value));
  return MCDisassembler::Success;
}*/

/*static DecodeStatus DecodeImm16(MCInst &Inst, unsigned Value,
                                               uint64_t Address, const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(Value));
  return MCDisassembler::Success; 
}*/

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Field,
                                     uint64_t Address, const void *Decoder) {
  // Call targets need to be shifted left by one so this needs a custom
  // decoder.
  Inst.addOperand(MCOperand::createImm(Field));
  return MCDisassembler::Success;
}


#include "OPEN8GenDisassemblerTables.inc"

static DecodeStatus readInstruction8(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 1) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 1;
  Insn = (Bytes[0] << 0);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {
  if (Bytes.size() < 2) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 2;
  Insn = (Bytes[1] << 0) | (Bytes[0] << 8) ; //TODO: REV this

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction24(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 3) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 3;
  Insn = (Bytes[0] << 16) | (Bytes[1] << 0) | (Bytes[2] << 8); //Probably not the best way to dissasembler 24-bit isntructions

  return MCDisassembler::Success;
}

static const uint8_t *getDecoderTable(uint64_t Size) {

  switch (Size) {
    case 1: return DecoderTable8;
    case 2: return DecoderTable16;
    case 3: return DecoderTable24;
    //case 4: return DecoderTable32;
    default: llvm_unreachable("instructions must be 8, 16, or 24 bits");
  }
}

DecodeStatus OPEN8Disassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  uint32_t Insn;

  DecodeStatus Result;

  // Try decode a 8-bit instruction.
  {
    Result = readInstruction8(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail) return MCDisassembler::Fail;

    // Try to auto-decode a 8-bit instruction.
    Result = decodeInstruction(getDecoderTable(Size), Instr,
                               Insn, Address, this, STI);

    if (Result != MCDisassembler::Fail)
      return Result;
  }

  // Try decode a 16-bit instruction.
  {
    Result = readInstruction16(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail) return MCDisassembler::Fail;

    // Try to auto-decode a 16-bit instruction.
    Result = decodeInstruction(getDecoderTable(Size), Instr,
                               Insn, Address, this, STI);

    if (Result != MCDisassembler::Fail)
      return Result;
  }

  // Try decode a 24-bit instruction.
  {
    Result = readInstruction24(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail) return MCDisassembler::Fail;

    Result = decodeInstruction(getDecoderTable(Size), Instr, Insn,
                               Address, this, STI);

    if (Result != MCDisassembler::Fail)
      return Result;
  }
  return MCDisassembler::Fail;
}

typedef DecodeStatus (*DecodeFunc)(MCInst &MI, unsigned insn, uint64_t Address,
                                   const void *Decoder);

