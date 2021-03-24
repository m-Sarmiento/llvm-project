//===-- OPEN8TargetStreamer.h - OPEN8 Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_OPEN8_TARGET_STREAMER_H
#define LLVM_OPEN8_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic OPEN8 target output stream.
class OPEN8TargetStreamer : public MCTargetStreamer {
public:
  explicit OPEN8TargetStreamer(MCStreamer &S);

  void finish() override;
};

/// A target streamer for textual OPEN8 assembly code.
class OPEN8TargetAsmStreamer : public OPEN8TargetStreamer {
public:
  explicit OPEN8TargetAsmStreamer(MCStreamer &S);
};

} // end namespace llvm

#endif // LLVM_OPEN8_TARGET_STREAMER_H
