//===-- OPEN8MCAsmInfo.h - OPEN8 asm properties ---------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the OPEN8MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_OPEN8_ASM_INFO_H
#define LLVM_OPEN8_ASM_INFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {

class Triple;

/// Specifies the format of OPEN8 assembly files.
class OPEN8MCAsmInfo : public MCAsmInfo {
public:
  explicit OPEN8MCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
};

} // end namespace llvm

#endif // LLVM_OPEN8_ASM_INFO_H
