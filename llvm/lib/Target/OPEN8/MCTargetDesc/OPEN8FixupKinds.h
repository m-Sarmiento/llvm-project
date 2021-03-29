//===-- OPEN8FixupKinds.h - OPEN8 Specific Fixup Entries ------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_OPEN8_FIXUP_KINDS_H
#define LLVM_OPEN8_FIXUP_KINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace OPEN8 {

/// The set of supported fixups.
///
/// Although most of the current fixup types reflect a unique relocation
/// one can have multiple fixup types for a given relocation and thus need
/// to be uniquely named.
///
/// \note This table *must* be in the same order of
///       MCFixupKindInfo Infos[OPEN8::NumTargetFixupKinds]
///       in `OPEN8AsmBackend.cpp`.
enum Fixups {
  /// A 32-bit OPEN8 fixup.
  fixup_32 = FirstTargetFixupKind,

  /// A 8-bit PC-relative fixup for the family of conditional
  /// branches which take 8-bit targets (BRNE,BRGT,etc).
  fixup_8_pcrel,

  /// A 16-bit address.
  fixup_16,

  /// Replaces the 8-bit immediate with another value.
  fixup_ldi,

  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the lower 8 bits of a 16-bit value (bits 0-7).
  fixup_lo8_ldi,
  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the upper 8 bits of a 16-bit value (bits 8-15).
  fixup_hi8_ldi,
  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the upper 8 bits of a 24-bit value (bits 16-23).
  fixup_hh8_ldi,
  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the upper 8 bits of a 32-bit value (bits 24-31).
  fixup_ms8_ldi,

  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the lower 8 bits of a negated 16-bit value (bits 0-7).
  fixup_lo8_ldi_neg,
  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the upper 8 bits of a negated 16-bit value (bits 8-15).
  fixup_hi8_ldi_neg,
  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the upper 8 bits of a negated negated 24-bit value (bits 16-23).
  fixup_hh8_ldi_neg,
  /// Replaces the immediate operand of a 16-bit `Rd, K` instruction
  /// with the upper 8 bits of a negated negated 32-bit value (bits 24-31).
  fixup_ms8_ldi_neg,

  /// A 16-bit fixup for the target of a `CALL k` or `JMP k` instruction.
  fixup_call,

  fixup_6,
  /// A symbol+addr fixup for the `LDD <x>+<n>, <r>" family of instructions.
  fixup_6_adiw,

  fixup_lo8_ldi_gs,
  fixup_hi8_ldi_gs,

  fixup_8,
  fixup_8_lo8,
  fixup_8_hi8,
  fixup_8_hlo8,

  fixup_diff8,
  fixup_diff16,
  fixup_diff32,

  fixup_lds_sts_16,

  /// A 6-bit port address.
  fixup_port6,
  /// A 5-bit port address.
  fixup_port5,

  // Marker
  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};
}
} // end of namespace llvm::OPEN8

#endif // LLVM_OPEN8_FIXUP_KINDS_H
