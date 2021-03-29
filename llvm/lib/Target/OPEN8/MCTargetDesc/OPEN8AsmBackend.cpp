//===-- OPEN8AsmBackend.cpp - OPEN8 Asm Backend  ------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the OPEN8AsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/OPEN8AsmBackend.h"
#include "MCTargetDesc/OPEN8FixupKinds.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

// FIXME: we should be doing checks to make sure asm operands
// are not out of bounds.

namespace adjust {

using namespace llvm;

static void signed_width(unsigned Width, uint64_t Value,
                         std::string Description, const MCFixup &Fixup,
                         MCContext *Ctx = nullptr) {
  if (!isIntN(Width, Value)) {
    std::string Diagnostic = "out of range " + Description;

    int64_t Min = minIntN(Width);
    int64_t Max = maxIntN(Width);

    Diagnostic += " (expected an integer in the range " + std::to_string(Min) +
      " to " + std::to_string(Max) + ")";

    if (Ctx) {
      Ctx->reportFatalError(Fixup.getLoc(), Diagnostic);
    } else {
      llvm_unreachable(Diagnostic.c_str());
    }
  }
}

static void unsigned_width(unsigned Width, uint64_t Value,
                           std::string Description, const MCFixup &Fixup,
                           MCContext *Ctx = nullptr) {
  if (!isUIntN(Width, Value)) {
    std::string Diagnostic = "out of range " + Description;

    int64_t Max = maxUIntN(Width);

    Diagnostic += " (expected an integer in the range 0 to " +
      std::to_string(Max) + ")";

    if (Ctx) {
      Ctx->reportFatalError(Fixup.getLoc(), Diagnostic);
    } else {
      llvm_unreachable(Diagnostic.c_str());
    }
  }
}

/// Adjusts the value of a branch target before fixup application.
static void adjustBranch(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                         MCContext *Ctx = nullptr) {
  unsigned_width(Size , Value, std::string("branch target"), Fixup, Ctx);
}

/// Adjusts the value of a relative branch target before fixup application.
static void adjustRelativeBranch(unsigned Size, const MCFixup &Fixup,
                                 uint64_t &Value, MCContext *Ctx = nullptr) {
  signed_width(Size, Value, std::string("branch target"), Fixup, Ctx);
}

/// 22-bit absolute fixup.
///
/// Resolves to:
/// 1001 kkkk 010k kkkk kkkk kkkk 111k kkkk
///
/// Offset of 0 (so the result is left shifted by 3 bits before application).
static void fixup_call(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                       MCContext *Ctx = nullptr) {
  adjustBranch(Size, Fixup, Value, Ctx);
  Value &= 0xffff;
}

/// 8-bit PC-relative fixup.
///
/// Resolves to:
/// 0000 00kk kkkk k000
/// Offset of 0 (so the result is left shifted by 3 bits before application).
static void fixup_8_pcrel(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                          MCContext *Ctx = nullptr) {
  adjustRelativeBranch(Size, Fixup, Value, Ctx);

  Value &= 0xff;
}

/// Fixups relating to the LDI instruction.
namespace ldi {

/// Adjusts a value to fix up the immediate of an `LDI Rd, K` instruction.
///
/// Resolves to:
static void fixup(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                  MCContext *Ctx = nullptr) {
  uint64_t upper = Value & 0xf0;
  uint64_t lower = Value & 0x0f;

  Value = (upper << 4) | lower;
}

static void neg(uint64_t &Value) { Value *= -1; }

static void lo8(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                MCContext *Ctx = nullptr) {
  Value &= 0xff;
  ldi::fixup(Size, Fixup, Value, Ctx);
}

static void hi8(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                MCContext *Ctx = nullptr) {
  Value = (Value & 0xff00) >> 8;
  ldi::fixup(Size, Fixup, Value, Ctx);
}

static void hh8(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                MCContext *Ctx = nullptr) {
  Value = (Value & 0xff0000) >> 16;
  ldi::fixup(Size, Fixup, Value, Ctx);
}

static void ms8(unsigned Size, const MCFixup &Fixup, uint64_t &Value,
                MCContext *Ctx = nullptr) {
  Value = (Value & 0xff000000) >> 24;
  ldi::fixup(Size, Fixup, Value, Ctx);
}

} // end of ldi namespace
} // end of adjust namespace

namespace llvm {

// Prepare value for the target space for it
void OPEN8AsmBackend::adjustFixupValue(const MCFixup &Fixup,
                                     const MCValue &Target,
                                     uint64_t &Value,
                                     MCContext *Ctx) const {
  // The size of the fixup in bits.
  uint64_t Size = OPEN8AsmBackend::getFixupKindInfo(Fixup.getKind()).TargetSize;

  unsigned Kind = Fixup.getKind();
  switch (Kind) {
  default:
    llvm_unreachable("unhandled fixup");
  case OPEN8::fixup_8_pcrel:
    adjust::fixup_8_pcrel(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_call:
    adjust::fixup_call(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_ldi:
    adjust::ldi::fixup(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_lo8_ldi:
    adjust::ldi::lo8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_lo8_ldi_gs:
    adjust::ldi::lo8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_hi8_ldi:
    adjust::ldi::hi8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_hi8_ldi_gs:
    adjust::ldi::hi8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_hh8_ldi:
    adjust::ldi::hh8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_ms8_ldi:
    adjust::ldi::ms8(Size, Fixup, Value, Ctx);
    break;

  case OPEN8::fixup_lo8_ldi_neg:
    adjust::ldi::neg(Value);
    adjust::ldi::lo8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_hi8_ldi_neg:
    adjust::ldi::neg(Value);
    adjust::ldi::hi8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_hh8_ldi_neg:
    adjust::ldi::neg(Value);
    adjust::ldi::hh8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_ms8_ldi_neg:
    adjust::ldi::neg(Value);
    adjust::ldi::ms8(Size, Fixup, Value, Ctx);
    break;
  case OPEN8::fixup_16:
    adjust::unsigned_width(16, Value, std::string("port number"), Fixup, Ctx);

    Value &= 0xffff;
    break;

  // Fixups which do not require adjustments.
  case FK_Data_1:
  case FK_Data_2:
  case FK_Data_4:
  case FK_Data_8:
    break;

  case FK_GPRel_4:
    llvm_unreachable("don't know how to adjust this fixup");
    break;
  }
}

std::unique_ptr<MCObjectTargetWriter>
OPEN8AsmBackend::createObjectTargetWriter() const {
  return createOPEN8ELFObjectWriter(MCELFObjectTargetWriter::getOSABI(OSType));
}

void OPEN8AsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved,
                               const MCSubtargetInfo *STI) const {
  adjustFixupValue(Fixup, Target, Value, &Asm.getContext());
  if (Value == 0)
    return; // Doesn't change encoding.

  MCFixupKindInfo Info = getFixupKindInfo(Fixup.getKind());

  // The number of bits in the fixup mask
  auto NumBits = Info.TargetSize + Info.TargetOffset;
  auto NumBytes = (NumBits / 8) + ((NumBits % 8) == 0 ? 0 : 1);

  // Shift the value into position.
  Value <<= Info.TargetOffset;

  unsigned Offset = Fixup.getOffset();
  assert(Offset + NumBytes <= Data.size() && "Invalid fixup offset!");

  // For each byte of the fragment that the fixup touches, mask in the
  // bits from the fixup value.
  for (unsigned i = 0; i < NumBytes; ++i) {
    uint8_t mask = (((Value >> (i * 8)) & 0xff));
    Data[Offset + i] |= mask;
  }
}

MCFixupKindInfo const &OPEN8AsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  // NOTE: Many OPEN8 fixups work on sets of non-contignous bits. We work around
  // this by saying that the fixup is the size of the entire instruction.
  const static MCFixupKindInfo Infos[OPEN8::NumTargetFixupKinds] = {
      // This table *must* be in same the order of fixup_* kinds in
      // OPEN8FixupKinds.h.
      //
      // name                    offset  bits  flags
      {"fixup_32", 0, 32, 0},

      {"fixup_8_pcrel", 0, 8, MCFixupKindInfo::FKF_IsPCRel},

      {"fixup_16", 0, 16, 0},

      {"fixup_ldi", 0, 8, 0},

      {"fixup_lo8_ldi", 0, 8, 0},
      {"fixup_hi8_ldi", 0, 8, 0},
      {"fixup_hh8_ldi", 0, 8, 0},
      {"fixup_ms8_ldi", 0, 8, 0},

      {"fixup_lo8_ldi_neg", 0, 8, 0},
      {"fixup_hi8_ldi_neg", 0, 8, 0},
      {"fixup_hh8_ldi_neg", 0, 8, 0},
      {"fixup_ms8_ldi_neg", 0, 8, 0},

      {"fixup_call", 0, 16, 0},

      {"fixup_lo8_ldi_gs", 0, 8, 0},
      {"fixup_hi8_ldi_gs", 0, 8, 0},

      {"fixup_8", 0, 8, 0},
      {"fixup_8_lo8", 0, 8, 0},
      {"fixup_8_hi8", 0, 8, 0},
      {"fixup_8_hlo8", 0, 8, 0},

      {"fixup_diff8", 0, 8, 0},
      {"fixup_diff16", 0, 16, 0},
      {"fixup_diff32", 0, 32, 0},

      {"fixup_lds_sts_16", 0, 16, 0},

  };

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");

  return Infos[Kind - FirstTargetFixupKind];
}

bool OPEN8AsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
  // If the count is not 2-byte aligned, we must be writing data into the text
  // section (otherwise we have unaligned instructions, and thus have far
  // bigger problems), so just write zeros instead.
  //assert((Count % 2) == 0 && "NOP instructions must be 2 bytes");

  OS.write_zeros(Count);
  return true;
}

bool OPEN8AsmBackend::shouldForceRelocation(const MCAssembler &Asm,
                                          const MCFixup &Fixup,
                                          const MCValue &Target) {
  switch ((unsigned) Fixup.getKind()) {
  default: return false;
  // Fixups which should always be recorded as relocations.
  case OPEN8::fixup_8_pcrel:
  case OPEN8::fixup_call:
    return true;
  }
}

MCAsmBackend *createOPEN8AsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new OPEN8AsmBackend(STI.getTargetTriple().getOS());
}

} // end of namespace llvm

