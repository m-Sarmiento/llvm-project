//===-- OPEN8MCTargetDesc.cpp - OPEN8 Target Descriptions ---------------------===//
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

#include "OPEN8ELFStreamer.h"
#include "OPEN8InstPrinter.h"
#include "OPEN8MCAsmInfo.h"
#include "OPEN8MCELFStreamer.h"
#include "OPEN8MCTargetDesc.h"
#include "OPEN8TargetStreamer.h"
#include "TargetInfo/OPEN8TargetInfo.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "OPEN8GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "OPEN8GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "OPEN8GenRegisterInfo.inc"

using namespace llvm;

MCInstrInfo *llvm::createOPEN8MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitOPEN8MCInstrInfo(X);

  return X;
}

static MCRegisterInfo *createOPEN8MCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitOPEN8MCRegisterInfo(X, 0);

  return X;
}

static MCSubtargetInfo *createOPEN8MCSubtargetInfo(const Triple &TT,
                                                 StringRef CPU, StringRef FS) {
  return createOPEN8MCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCInstPrinter *createOPEN8MCInstPrinter(const Triple &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI) {
  if (SyntaxVariant == 0) {
    return new OPEN8InstPrinter(MAI, MII, MRI);
  }

  return nullptr;
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  return createELFStreamer(Context, std::move(MAB), std::move(OW),
                           std::move(Emitter), RelaxAll);
}

static MCTargetStreamer *
createOPEN8ObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new OPEN8ELFStreamer(S, STI);
}

static MCTargetStreamer *createMCAsmTargetStreamer(MCStreamer &S,
                                                   formatted_raw_ostream &OS,
                                                   MCInstPrinter *InstPrint,
                                                   bool isVerboseAsm) {
  return new OPEN8TargetAsmStreamer(S);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeOPEN8TargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<OPEN8MCAsmInfo> X(getTheOPEN8Target());

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(getTheOPEN8Target(), createOPEN8MCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(getTheOPEN8Target(), createOPEN8MCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(getTheOPEN8Target(),
                                          createOPEN8MCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(getTheOPEN8Target(),
                                        createOPEN8MCInstPrinter);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(getTheOPEN8Target(), createOPEN8MCCodeEmitter);

  // Register the obj streamer
  TargetRegistry::RegisterELFStreamer(getTheOPEN8Target(), createMCStreamer);

  // Register the obj target streamer.
  TargetRegistry::RegisterObjectTargetStreamer(getTheOPEN8Target(),
                                               createOPEN8ObjectTargetStreamer);

  // Register the asm target streamer.
  TargetRegistry::RegisterAsmTargetStreamer(getTheOPEN8Target(),
                                            createMCAsmTargetStreamer);

  // Register the asm backend (as little endian).
  TargetRegistry::RegisterMCAsmBackend(getTheOPEN8Target(), createOPEN8AsmBackend);
}

