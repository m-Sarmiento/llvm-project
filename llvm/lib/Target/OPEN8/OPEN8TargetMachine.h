//===-- OPEN8TargetMachine.h - Define TargetMachine for OPEN8 -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the OPEN8 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_OPEN8_TARGET_MACHINE_H
#define LLVM_OPEN8_TARGET_MACHINE_H

#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#include "OPEN8FrameLowering.h"
#include "OPEN8ISelLowering.h"
#include "OPEN8InstrInfo.h"
#include "OPEN8SelectionDAGInfo.h"
#include "OPEN8Subtarget.h"

namespace llvm {

/// A generic OPEN8 implementation.
class OPEN8TargetMachine : public LLVMTargetMachine {
public:
  OPEN8TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Optional<Reloc::Model> RM,
                   Optional<CodeModel::Model> CM,
                   CodeGenOpt::Level OL, bool JIT);

  const OPEN8Subtarget *getSubtargetImpl() const;
  const OPEN8Subtarget *getSubtargetImpl(const Function &) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return this->TLOF.get();
  }

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  bool isMachineVerifierClean() const override {
    return false;
  }

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  OPEN8Subtarget SubTarget;
};

} // end namespace llvm

#endif // LLVM_OPEN8_TARGET_MACHINE_H
