//===----- OPEN8ELFStreamer.h - OPEN8 Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_OPEN8_ELF_STREAMER_H
#define LLVM_OPEN8_ELF_STREAMER_H

#include "OPEN8TargetStreamer.h"

namespace llvm {

/// A target streamer for an OPEN8 ELF object file.
class OPEN8ELFStreamer : public OPEN8TargetStreamer {
public:
  OPEN8ELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  MCELFStreamer &getStreamer() {
    return static_cast<MCELFStreamer &>(Streamer);
  }
};

} // end namespace llvm

#endif
