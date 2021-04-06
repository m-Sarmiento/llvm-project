//===-- OPEN8TargetMachine.cpp - Define TargetMachine for OPEN8 ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the OPEN8 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "OPEN8TargetMachine.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"

#include "OPEN8.h"
#include "OPEN8TargetObjectFile.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"
#include "TargetInfo/OPEN8TargetInfo.h"

namespace llvm {

static const char *OPEN8DataLayout = "e-p:16:8-i8:8-i16:8-i24:8-i32:8-i64:8-i128:8-f32:8-f64:8-n8-a:8";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    return "open81";
  }

  return CPU;
}

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  return RM.getValueOr(Reloc::Static);
}

OPEN8TargetMachine::OPEN8TargetMachine(const Target &T, const Triple &TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Optional<Reloc::Model> RM,
                                   Optional<CodeModel::Model> CM,
                                   CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, OPEN8DataLayout, TT, getCPU(CPU), FS, Options,
                        getEffectiveRelocModel(RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      SubTarget(TT, std::string(getCPU(CPU)), std::string(FS), *this) {
  this->TLOF = std::make_unique<OPEN8TargetObjectFile>();
  initAsmInfo();
}

namespace {
/// OPEN8 Code Generator Pass Configuration Options.
class OPEN8PassConfig : public TargetPassConfig {
public:
  OPEN8PassConfig(OPEN8TargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  OPEN8TargetMachine &getOPEN8TargetMachine() const {
    return getTM<OPEN8TargetMachine>();
  }

  bool addInstSelector() override;
  void addPreSched2() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
};
} // namespace

TargetPassConfig *OPEN8TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new OPEN8PassConfig(*this, PM);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeOPEN8Target() {
  // Register the target.
  RegisterTargetMachine<OPEN8TargetMachine> X(getTheOPEN8Target());

  auto &PR = *PassRegistry::getPassRegistry();
  initializeOPEN8ExpandPseudoPass(PR);
  initializeOPEN8RelaxMemPass(PR);
}

const OPEN8Subtarget *OPEN8TargetMachine::getSubtargetImpl() const {
  return &SubTarget;
}

const OPEN8Subtarget *OPEN8TargetMachine::getSubtargetImpl(const Function &) const {
  return &SubTarget;
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

bool OPEN8PassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createOPEN8ISelDag(getOPEN8TargetMachine(), getOptLevel()));
  // Create the frame analyzer pass used by the PEI pass.
  addPass(createOPEN8FrameAnalyzerPass());

  return false;
}

void OPEN8PassConfig::addPreRegAlloc() {
  // Create the dynalloc SP save/restore pass to handle variable sized allocas.
  addPass(createOPEN8DynAllocaSRPass());
}

void OPEN8PassConfig::addPreSched2() {
  addPass(createOPEN8RelaxMemPass());
  addPass(createOPEN8ExpandPseudoPass());
}

void OPEN8PassConfig::addPreEmitPass() {
  // Must run branch selection immediately preceding the asm printer.
  addPass(&BranchRelaxationPassID);
}

} // end of namespace llvm
