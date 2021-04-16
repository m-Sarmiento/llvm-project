//===--- OPEN8.cpp - Implement OPEN8 target feature support -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements OPEN8 TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "OPEN8.h"
#include "clang/Basic/MacroBuilder.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang;
using namespace clang::targets;

namespace clang {
namespace targets {

/// Information about a specific microcontroller.
struct LLVM_LIBRARY_VISIBILITY MCUInfo {
  const char *Name;
  const char *DefineName;
};

// This list should be kept up-to-date with OPEN8Devices.td in LLVM.
static MCUInfo OPEN8Mcus[] = {
    {"open8generic_0", "__OPEN8_GENERIC0__"},
    {"open8generic_8", "__OPEN8_GENERIC1__"},
};

} // namespace targets
} // namespace clang

static constexpr llvm::StringLiteral ValidFamilyNames[] = {
    "open80",      "open81"};

bool OPEN8TargetInfo::isValidCPUName(StringRef Name) const {
  bool IsFamily =
      llvm::find(ValidFamilyNames, Name) != std::end(ValidFamilyNames);

  bool IsMCU =
      llvm::find_if(OPEN8Mcus, [&](const MCUInfo &Info) {
        return Info.Name == Name;
      }) != std::end(OPEN8Mcus);
  return IsFamily || IsMCU;
}

void OPEN8TargetInfo::fillValidCPUList(SmallVectorImpl<StringRef> &Values) const {
  Values.append(std::begin(ValidFamilyNames), std::end(ValidFamilyNames));
  for (const MCUInfo &Info : OPEN8Mcus)
    Values.push_back(Info.Name);
}

void OPEN8TargetInfo::getTargetDefines(const LangOptions &Opts,
                                     MacroBuilder &Builder) const {
  Builder.defineMacro("OPEN8");
  Builder.defineMacro("__OPEN8");
  Builder.defineMacro("__OPEN8__");
  Builder.defineMacro("__ELF__");

  if (!this->CPU.empty()) {
    auto It = llvm::find_if(
        OPEN8Mcus, [&](const MCUInfo &Info) { return Info.Name == this->CPU; });

    if (It != std::end(OPEN8Mcus))
      Builder.defineMacro(It->DefineName);
  }
}
