//===-- OPEN8TargetInfo.cpp - OPEN8 Target Implementation ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/OPEN8TargetInfo.h"
#include "llvm/Support/TargetRegistry.h"
namespace llvm {
Target &getTheOPEN8Target() {
  static Target TheOPEN8Target;
  return TheOPEN8Target;
}
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeOPEN8TargetInfo() {
  llvm::RegisterTarget<llvm::Triple::open8> X(llvm::getTheOPEN8Target(), "open8",
                                            "Open8 urisc Microcontroller", "OPEN8");
}

