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
  else if (Features[OPEN8::ELFArchOPEN82])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN82;
  else if (Features[OPEN8::ELFArchOPEN825])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN825;
  else if (Features[OPEN8::ELFArchOPEN83])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN83;
  else if (Features[OPEN8::ELFArchOPEN831])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN831;
  else if (Features[OPEN8::ELFArchOPEN835])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN835;
  else if (Features[OPEN8::ELFArchOPEN84])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN84;
  else if (Features[OPEN8::ELFArchOPEN85])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN85;
  else if (Features[OPEN8::ELFArchOPEN851])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN851;
  else if (Features[OPEN8::ELFArchOPEN86])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN86;
  else if (Features[OPEN8::ELFArchTiny])
    EFlags |= ELF::EF_OPEN8_ARCH_OPEN8TINY;
  else if (Features[OPEN8::ELFArchXMEGA1])
    EFlags |= ELF::EF_OPEN8_ARCH_XMEGA1;
  else if (Features[OPEN8::ELFArchXMEGA2])
    EFlags |= ELF::EF_OPEN8_ARCH_XMEGA2;
  else if (Features[OPEN8::ELFArchXMEGA3])
    EFlags |= ELF::EF_OPEN8_ARCH_XMEGA3;
  else if (Features[OPEN8::ELFArchXMEGA4])
    EFlags |= ELF::EF_OPEN8_ARCH_XMEGA4;
  else if (Features[OPEN8::ELFArchXMEGA5])
    EFlags |= ELF::EF_OPEN8_ARCH_XMEGA5;
  else if (Features[OPEN8::ELFArchXMEGA6])
    EFlags |= ELF::EF_OPEN8_ARCH_XMEGA6;
  else if (Features[OPEN8::ELFArchXMEGA7])
    EFlags |= ELF::EF_OPEN8_ARCH_XMEGA7;

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
