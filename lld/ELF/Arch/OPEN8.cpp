//===- OPEN8.cpp ------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// OPEN8 is a 8-bit micrcontroller designed for small
// baremetal programs. OPEN8 have 8 8-bit registers.
// The tiniest OPEN8 has 1k byte RAM and 1 KiB program memory, and the largest
// one supports up to 2^16 data address space and 2^16 code address space.
//
// Since it is a baremetal programming, there's usually no loader to load
// ELF files on OPEN8s. You are expected to link your program against address
// 0 and pull out a .text section from the result using objcopy, so that you
// can write the linked code to on-chip flush memory. You can do that with
// the following commands:
//
//   ld.lld -Ttext=0 -o foo foo.o
//   objcopy -O binary --only-section=.text foo output.bin
//
// Note that the current OPEN8 support is very preliminary so you can't
// link any useful program yet, though.
//
//===----------------------------------------------------------------------===//

#include "InputFiles.h"
#include "Symbols.h"
#include "Target.h"
#include "lld/Common/ErrorHandler.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/Endian.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::support::endian;
using namespace llvm::ELF;
using namespace lld;
using namespace lld::elf;

namespace {
class OPEN8 final : public TargetInfo {
public:
  OPEN8();
  RelExpr getRelExpr(RelType type, const Symbol &s,
                     const uint8_t *loc) const override;
  void relocate(uint8_t *loc, const Relocation &rel,
                uint64_t val) const override;
};
} // namespace

OPEN8::OPEN8() { noneRel = R_OPEN8_NONE; }

RelExpr OPEN8::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  switch (type) {
  case R_OPEN8_PCREL:
    return R_PC;
  default:
    return R_ABS;
  }
}

static void writeLDI(uint8_t *loc, uint64_t val) {
  write16le(loc, read16le(loc) | val );
}

void OPEN8::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  switch (rel.type) {
  case R_OPEN8_8:
    checkUInt(loc, val, 8, rel);
    *loc = val;
    break;
  case R_OPEN8_16:
    // Note: this relocation is often used between code and data space, which
    // are 0x800000 apart in the output ELF file. The bitmask cuts off the high
    // bit.
    write16le(loc+1, val & 0xffff);
    break;
  case R_OPEN8_32:
    checkUInt(loc, val, 32, rel);
    write32le(loc, val);
    break;

  case R_OPEN8_LO8_LDI_NEG:
    writeLDI(loc, -val & 0xff);
    break;
  case R_OPEN8_LO8_LDI:
    writeLDI(loc, val & 0xff);
    break;
  case R_OPEN8_HI8_LDI_NEG:
    writeLDI(loc, (-val >> 8) & 0xff);
    break;
  case R_OPEN8_HI8_LDI:
    writeLDI(loc, (val >> 8) & 0xff);
    break;

  // Since every jump destination is word aligned we gain an extra bit
  case R_OPEN8_PCREL: {
    checkInt(loc, val, 8, rel);
    //uint16_t val8 = (val - 2 -(uint64_t)loc) & 0xff;
    val &= 0xFF;
    write16le(loc+1, read16le(loc+1)| val);
    break;
  }

  case R_OPEN8_CALL: {
    write16le(loc+1, val & 0xffff);
    break;
  }
  default:
    error(getErrorLocation(loc) + "unrecognized relocation " +
          toString(rel.type));
  }
}

TargetInfo *elf::getOPEN8TargetInfo() {
  static OPEN8 target;
  return &target;
}
