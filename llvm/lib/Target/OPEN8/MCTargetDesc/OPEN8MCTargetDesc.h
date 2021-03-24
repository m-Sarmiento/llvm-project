//===-- OPEN8MCTargetDesc.h - OPEN8 Target Descriptions -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides OPEN8 specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_OPEN8_MCTARGET_DESC_H
#define LLVM_OPEN8_MCTARGET_DESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {

class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class Target;

MCInstrInfo *createOPEN8MCInstrInfo();

/// Creates a machine code emitter for OPEN8.
MCCodeEmitter *createOPEN8MCCodeEmitter(const MCInstrInfo &MCII,
                                      const MCRegisterInfo &MRI,
                                      MCContext &Ctx);

/// Creates an assembly backend for OPEN8.
MCAsmBackend *createOPEN8AsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO);

/// Creates an ELF object writer for OPEN8.
std::unique_ptr<MCObjectTargetWriter> createOPEN8ELFObjectWriter(uint8_t OSABI);

} // end namespace llvm

#define GET_REGINFO_ENUM
#include "OPEN8GenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#include "OPEN8GenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "OPEN8GenSubtargetInfo.inc"

#endif // LLVM_OPEN8_MCTARGET_DESC_H
