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
  OPEN8::R4, OPEN8::R5, OPEN8::R6, OPEN8::R7,
  OPEN8::R8, OPEN8::R9, OPEN8::R10, OPEN8::R11,
  OPEN8::R12, OPEN8::R13, OPEN8::R14, OPEN8::R15,
  OPEN8::R16, OPEN8::R17, OPEN8::R18, OPEN8::R19,
  OPEN8::R20, OPEN8::R21, OPEN8::R22, OPEN8::R23,
  OPEN8::R24, OPEN8::R25, OPEN8::R26, OPEN8::R27,
  OPEN8::R28, OPEN8::R29, OPEN8::R30, OPEN8::R31,
};

static DecodeStatus DecodeGPR8RegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address, const void *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeLD8RegisterClass(MCInst &Inst, unsigned RegNo,
                                           uint64_t Address, const void *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo+16];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodePTRREGSRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address, const void *Decoder) {
  // Note: this function must be defined but does not seem to be called.
  assert(false && "unimplemented: PTRREGS register class");
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Insn,
                                     uint64_t Address, const void *Decoder);

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn,
                              uint64_t Address, const void *Decoder);

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder);

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder);

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

#include "OPEN8GenDisassemblerTables.inc"

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  Inst.addOperand(MCOperand::createImm(addr));
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(addr));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = fieldFromInstruction(Insn, 3, 5);
  unsigned b = fieldFromInstruction(Insn, 0, 3);
  Inst.addOperand(MCOperand::createImm(addr));
  Inst.addOperand(MCOperand::createImm(b));
  return MCDisassembler::Success;
}

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Field,
                                     uint64_t Address, const void *Decoder) {
  // Call targets need to be shifted left by one so this needs a custom
  // decoder.
  Inst.addOperand(MCOperand::createImm(Field << 1));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn,
                              uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder) {
  if (decodeFRd(Inst, Insn, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(OPEN8::R31R30));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 3) + 16;
  unsigned r = fieldFromInstruction(Insn, 0, 3) + 16;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned r = fieldFromInstruction(Insn, 4, 4) * 2;
  unsigned d = fieldFromInstruction(Insn, 0, 4) * 2;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 2) * 2 + 24; // starts at r24:r25
  unsigned k = 0;
  k |= fieldFromInstruction(Insn, 0, 4);
  k |= fieldFromInstruction(Insn, 6, 2) << 4;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(k));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned rd = fieldFromInstruction(Insn, 4, 4) + 16;
  unsigned rr = fieldFromInstruction(Insn, 0, 4) + 16;
  if (DecodeGPR8RegisterClass(Inst, rd, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, rr, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {
  if (Bytes.size() < 2) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 2;
  Insn = (Bytes[0] << 0) | (Bytes[1] << 8);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 4;
  Insn = (Bytes[0] << 16) | (Bytes[1] << 24) | (Bytes[2] << 0) | (Bytes[3] << 8);

  return MCDisassembler::Success;
}

static const uint8_t *getDecoderTable(uint64_t Size) {

  switch (Size) {
    case 2: return DecoderTable16;
    case 4: return DecoderTable32;
    default: llvm_unreachable("instructions must be 16 or 32-bits");
  }
}

DecodeStatus OPEN8Disassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  uint32_t Insn;

  DecodeStatus Result;

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

  // Try decode a 32-bit instruction.
  {
    Result = readInstruction32(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail) return MCDisassembler::Fail;

    Result = decodeInstruction(getDecoderTable(Size), Instr, Insn,
                               Address, this, STI);

    if (Result != MCDisassembler::Fail) {
      return Result;
    }

    return MCDisassembler::Fail;
  }
}

typedef DecodeStatus (*DecodeFunc)(MCInst &MI, unsigned insn, uint64_t Address,
                                   const void *Decoder);

