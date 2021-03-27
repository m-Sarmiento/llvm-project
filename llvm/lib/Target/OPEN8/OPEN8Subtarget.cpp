//===-- OPEN8Subtarget.cpp - OPEN8 Subtarget Information ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the OPEN8 specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "OPEN8Subtarget.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Support/TargetRegistry.h"

#include "OPEN8.h"
#include "OPEN8TargetMachine.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"

#define DEBUG_TYPE "open8-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "OPEN8GenSubtargetInfo.inc"

namespace llvm {

OPEN8Subtarget::OPEN8Subtarget(const Triple &TT, const std::string &CPU,
                           const std::string &FS, const OPEN8TargetMachine &TM)
    : OPEN8GenSubtargetInfo(TT, CPU, /*TuneCPU*/ CPU, FS), ELFArch(0),

      // Subtarget features
      m_hasNEW(false), m_hasSRAM(false), m_hasJMPCALL(false), m_hasIJMPCALL(false),
      m_hasEIJMPCALL(false), m_hasADDSUBIW(false), m_hasSmallStack(false),
      m_hasMOVW(false), m_hasLPM(false), m_hasLPMX(false), m_hasELPM(false),
      m_hasELPMX(false), m_hasSPM(false), m_hasSPMX(false), m_hasDES(false),
      m_supportsRMW(false), m_supportsMultiplication(false), m_hasBREAK(false),
      m_hasTinyEncoding(false), m_hasMemMappedGPR(false),
      m_FeatureSetDummy(false),

      InstrInfo(), FrameLowering(),
      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)), TSInfo() {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /*TuneCPU*/ CPU, FS);
}

OPEN8Subtarget &
OPEN8Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                              const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /*TuneCPU*/ CPU, FS);
  return *this;
}

} // end of namespace llvm
