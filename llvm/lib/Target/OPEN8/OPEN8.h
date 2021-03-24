//===-- OPEN8.h - Top-level interface for OPEN8 representation ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// OPEN8 back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_OPEN8_H
#define LLVM_OPEN8_H

#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class OPEN8TargetMachine;
class FunctionPass;

FunctionPass *createOPEN8ISelDag(OPEN8TargetMachine &TM,
                               CodeGenOpt::Level OptLevel);
FunctionPass *createOPEN8ExpandPseudoPass();
FunctionPass *createOPEN8FrameAnalyzerPass();
FunctionPass *createOPEN8RelaxMemPass();
FunctionPass *createOPEN8DynAllocaSRPass();
FunctionPass *createOPEN8BranchSelectionPass();

void initializeOPEN8ExpandPseudoPass(PassRegistry&);
void initializeOPEN8RelaxMemPass(PassRegistry&);

/// Contains the OPEN8 backend.
namespace OPEN8 {

/// An integer that identifies all of the supported OPEN8 address spaces.
enum AddressSpace { DataMemory, ProgramMemory };

/// Checks if a given type is a pointer to program memory.
template <typename T> bool isProgramMemoryAddress(T *V) {
  return cast<PointerType>(V->getType())->getAddressSpace() == ProgramMemory;
}

inline bool isProgramMemoryAccess(MemSDNode const *N) {
  auto V = N->getMemOperand()->getValue();

  return (V != nullptr) ? isProgramMemoryAddress(V) : false;
}

} // end of namespace OPEN8

} // end namespace llvm

#endif // LLVM_OPEN8_H
