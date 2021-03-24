//===-- OPEN8ELFObjectWriter.cpp - OPEN8 ELF Writer ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/OPEN8FixupKinds.h"
#include "MCTargetDesc/OPEN8MCExpr.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes OPEN8 machine code into an ELF32 object file.
class OPEN8ELFObjectWriter : public MCELFObjectTargetWriter {
public:
  OPEN8ELFObjectWriter(uint8_t OSABI);

  virtual ~OPEN8ELFObjectWriter() {}

  unsigned getRelocType(MCContext &Ctx,
                        const MCValue &Target,
                        const MCFixup &Fixup,
                        bool IsPCRel) const override;
};

OPEN8ELFObjectWriter::OPEN8ELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_OPEN8, true) {}

unsigned OPEN8ELFObjectWriter::getRelocType(MCContext &Ctx,
                                          const MCValue &Target,
                                          const MCFixup &Fixup,
                                          bool IsPCRel) const {
  MCSymbolRefExpr::VariantKind Modifier = Target.getAccessVariant();
  switch ((unsigned) Fixup.getKind()) {
  case FK_Data_1:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_OPEN8_8;
    case MCSymbolRefExpr::VK_OPEN8_DIFF8://
      return ELF::R_OPEN8_8;
    case MCSymbolRefExpr::VK_OPEN8_LO8://
      return ELF::R_OPEN8_8;
    case MCSymbolRefExpr::VK_OPEN8_HI8://
      return ELF::R_OPEN8_8;
    case MCSymbolRefExpr::VK_OPEN8_HLO8://
      return ELF::R_OPEN8_8;
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_OPEN8_32;
    case MCSymbolRefExpr::VK_OPEN8_DIFF32://
      return ELF::R_OPEN8_32;
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_OPEN8_16;
    case MCSymbolRefExpr::VK_OPEN8_NONE:
    case MCSymbolRefExpr::VK_OPEN8_PM://
      return ELF::R_OPEN8_16;
    case MCSymbolRefExpr::VK_OPEN8_DIFF16://
      return ELF::R_OPEN8_16;
    }
  case OPEN8::fixup_32:
    return ELF::R_OPEN8_32;
  case OPEN8::fixup_7_pcrel://
    return ELF::R_OPEN8_PCREL;
  case OPEN8::fixup_13_pcrel://
    return ELF::R_OPEN8_PCREL;
  case OPEN8::fixup_16:
    return ELF::R_OPEN8_16;
  case OPEN8::fixup_16_pm://
    return ELF::R_OPEN8_16;
  case OPEN8::fixup_lo8_ldi:
    return ELF::R_OPEN8_LO8_LDI;
  case OPEN8::fixup_hi8_ldi:
    return ELF::R_OPEN8_HI8_LDI;
  case OPEN8::fixup_hh8_ldi://
    return ELF::R_OPEN8_HI8_LDI;
  case OPEN8::fixup_lo8_ldi_neg:
    return ELF::R_OPEN8_LO8_LDI_NEG;
  case OPEN8::fixup_hi8_ldi_neg:
    return ELF::R_OPEN8_HI8_LDI_NEG;
  case OPEN8::fixup_hh8_ldi_neg://
    return ELF::R_OPEN8_HI8_LDI_NEG;
  case OPEN8::fixup_lo8_ldi_pm://
    return ELF::R_OPEN8_LO8_LDI;
  case OPEN8::fixup_hi8_ldi_pm://
    return ELF::R_OPEN8_HI8_LDI;
  case OPEN8::fixup_hh8_ldi_pm://
    return ELF::R_OPEN8_HI8_LDI;
  case OPEN8::fixup_lo8_ldi_pm_neg://
    return ELF::R_OPEN8_LO8_LDI_NEG;
  case OPEN8::fixup_hi8_ldi_pm_neg://
    return ELF::R_OPEN8_HI8_LDI_NEG;
  case OPEN8::fixup_hh8_ldi_pm_neg://
    return ELF::R_OPEN8_HI8_LDI_NEG;
  case OPEN8::fixup_call:
    return ELF::R_OPEN8_CALL;
  case OPEN8::fixup_ldi:
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_6:
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_6_adiw:
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_ms8_ldi://
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_ms8_ldi_neg://
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_lo8_ldi_gs://
    return ELF::R_OPEN8_LO8_LDI;
  case OPEN8::fixup_hi8_ldi_gs://
    return ELF::R_OPEN8_HI8_LDI;
  case OPEN8::fixup_8://
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_8_lo8://
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_8_hi8://
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_8_hlo8://
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_diff8://
    return ELF::R_OPEN8_8;
  case OPEN8::fixup_diff16://
    return ELF::R_OPEN8_16;
  case OPEN8::fixup_diff32://
    return ELF::R_OPEN8_32;
  case OPEN8::fixup_lds_sts_16://
    return ELF::R_OPEN8_16;
  case OPEN8::fixup_port6://
    return ELF::R_OPEN8_NONE;
  case OPEN8::fixup_port5://
    return ELF::R_OPEN8_NONE;
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createOPEN8ELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<OPEN8ELFObjectWriter>(OSABI);
}

} // end of namespace llvm

