//===-- OPEN8MCExpr.cpp - OPEN8 specific MC expression classes ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "OPEN8MCExpr.h"

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char * const Spelling;
  OPEN8MCExpr::VariantKind VariantKind;
} ModifierNames[] = {
    {"lo8", OPEN8MCExpr::VK_OPEN8_LO8},       {"hi8", OPEN8MCExpr::VK_OPEN8_HI8},
    {"hh8", OPEN8MCExpr::VK_OPEN8_HH8}, // synonym with hlo8

    {"lo8_gs", OPEN8MCExpr::VK_OPEN8_LO8_GS}, {"hi8_gs", OPEN8MCExpr::VK_OPEN8_HI8_GS},
    {"gs", OPEN8MCExpr::VK_OPEN8_GS},
};

} // end of anonymous namespace

const OPEN8MCExpr *OPEN8MCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) OPEN8MCExpr(Kind, Expr, Negated);
}

void OPEN8MCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_OPEN8_None);

  if (isNegated())
    OS << '-';

  OS << getName() << '(';
  getSubExpr()->print(OS, MAI);
  OS << ')';
}

bool OPEN8MCExpr::evaluateAsConstant(int64_t &Result) const {
  MCValue Value;

  bool isRelocatable =
      getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = evaluateAsInt64(Value.getConstant());
    return true;
  }

  return false;
}

bool OPEN8MCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAsmLayout *Layout,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool isRelocatable = SubExpr->evaluateAsRelocatable(Value, Layout, Fixup);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Layout) return false;

    MCContext &Context = Layout->getAssembler().getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None)
      return false;

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t OPEN8MCExpr::evaluateAsInt64(int64_t Value) const {
  if (Negated)
    Value *= -1;

  switch (Kind) {
  case OPEN8MCExpr::VK_OPEN8_LO8:
    Value &= 0xff;
    break;
  case OPEN8MCExpr::VK_OPEN8_HI8:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case OPEN8MCExpr::VK_OPEN8_HH8:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case OPEN8MCExpr::VK_OPEN8_HHI8:
    Value &= 0xff000000;
    Value >>= 24;
    break;
  case OPEN8MCExpr::VK_OPEN8_LO8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff;
    break;
  case OPEN8MCExpr::VK_OPEN8_HI8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff00;
    Value >>= 8;
    break;
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case OPEN8MCExpr::VK_OPEN8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    break;

  case OPEN8MCExpr::VK_OPEN8_None:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value) & 0xff;
}

OPEN8::Fixups OPEN8MCExpr::getFixupKind() const {
  OPEN8::Fixups Kind = OPEN8::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_OPEN8_LO8:
    Kind = isNegated() ? OPEN8::fixup_lo8_ldi_neg : OPEN8::fixup_lo8_ldi;
    break;
  case VK_OPEN8_HI8:
    Kind = isNegated() ? OPEN8::fixup_hi8_ldi_neg : OPEN8::fixup_hi8_ldi;
    break;
  case VK_OPEN8_HH8:
    Kind = isNegated() ? OPEN8::fixup_hh8_ldi_neg : OPEN8::fixup_hh8_ldi;
    break;
  case VK_OPEN8_HHI8:
    Kind = isNegated() ? OPEN8::fixup_ms8_ldi_neg : OPEN8::fixup_ms8_ldi;
    break;

  case VK_OPEN8_GS:
    Kind = OPEN8::fixup_16;
    break;
  case VK_OPEN8_LO8_GS:
    Kind = OPEN8::fixup_lo8_ldi_gs;
    break;
  case VK_OPEN8_HI8_GS:
    Kind = OPEN8::fixup_hi8_ldi_gs;
    break;

  case VK_OPEN8_None:
    llvm_unreachable("Uninitialized expression");
  }

  return Kind;
}

void OPEN8MCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

const char *OPEN8MCExpr::getName() const {
  const auto &Modifier =
      llvm::find_if(ModifierNames, [this](ModifierEntry const &Mod) {
        return Mod.VariantKind == Kind;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->Spelling;
  }
  return nullptr;
}

OPEN8MCExpr::VariantKind OPEN8MCExpr::getKindByName(StringRef Name) {
  const auto &Modifier =
      llvm::find_if(ModifierNames, [&Name](ModifierEntry const &Mod) {
        return Mod.Spelling == Name;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }
  return VK_OPEN8_None;
}

} // end of namespace llvm

