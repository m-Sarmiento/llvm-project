#include "OPEN8ELFStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/FormattedStream.h"

#include "OPEN8MCTargetDesc.h"

namespace llvm {

static unsigned getEFlagsForFeatureSet(const FeatureBitset &Features) {
  unsigned EFlags = 0;

  // Set architecture
  if (Features[OPEN8::ELFArchOPEN81])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN81;
  return EFlags;
}

OPEN8ELFStreamer::OPEN8ELFStreamer(MCStreamer &S,
                               const MCSubtargetInfo &STI)
    : OPEN8TargetStreamer(S) {

  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned EFlags = MCA.getELFHeaderEFlags();

  EFlags |= getEFlagsForFeatureSet(STI.getFeatureBits());

  MCA.setELFHeaderEFlags(EFlags);
}

} // end namespace llvm
