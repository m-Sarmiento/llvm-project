//===--------- OPEN8MCELFStreamer.cpp - OPEN8 subclass of MCELFStreamer -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a stub that parses a MCInst bundle and passes the
// instructions on to the real streamer.
//
//===----------------------------------------------------------------------===//
#define DEBUG_TYPE "open8mcelfstreamer"

#include "MCTargetDesc/OPEN8MCELFStreamer.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCObjectWriter.h"

using namespace llvm;

void OPEN8MCELFStreamer::emitValueForModiferKind(
    const MCSymbol *Sym, unsigned SizeInBytes, SMLoc Loc,
    OPEN8MCExpr::VariantKind ModifierKind) {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_OPEN8_NONE;
  if (ModifierKind == OPEN8MCExpr::VK_OPEN8_None) {
    Kind = MCSymbolRefExpr::VK_OPEN8_DIFF8;
    if (SizeInBytes == SIZE_LONG)
      Kind = MCSymbolRefExpr::VK_OPEN8_DIFF32;
    else if (SizeInBytes == SIZE_WORD)
      Kind = MCSymbolRefExpr::VK_OPEN8_DIFF16;
  } else if (ModifierKind == OPEN8MCExpr::VK_OPEN8_LO8)
    Kind = MCSymbolRefExpr::VK_OPEN8_LO8;
  else if (ModifierKind == OPEN8MCExpr::VK_OPEN8_HI8)
    Kind = MCSymbolRefExpr::VK_OPEN8_HI8;
  else if (ModifierKind == OPEN8MCExpr::VK_OPEN8_HH8)
    Kind = MCSymbolRefExpr::VK_OPEN8_HLO8;
  MCELFStreamer::emitValue(MCSymbolRefExpr::create(Sym, Kind, getContext()),
                           SizeInBytes, Loc);
}

namespace llvm {
MCStreamer *createOPEN8ELFStreamer(Triple const &TT, MCContext &Context,
                                 std::unique_ptr<MCAsmBackend> MAB,
                                 std::unique_ptr<MCObjectWriter> OW,
                                 std::unique_ptr<MCCodeEmitter> CE) {
  return new OPEN8MCELFStreamer(Context, std::move(MAB), std::move(OW),
                              std::move(CE));
}

} // end namespace llvm
