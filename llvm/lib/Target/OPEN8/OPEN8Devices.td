//===---------------------------------------------------------------------===//
// OPEN8 Device Definitions
//===---------------------------------------------------------------------===//

// :TODO: Implement the skip errata, see `gcc/config/open8/open8-arch.h` for details
// :TODO: We define all devices with SRAM to have all variants of LD/ST/LDD/STD.
//        In reality, open81 (no SRAM) has one variant each of `LD` and `ST`.
//        open82 (with SRAM) adds the rest of the variants.


// A feature set aggregates features, grouping them. We don't want to create a
// new member in OPEN8Subtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
  : SubtargetFeature<name, "m_FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, list<SubtargetFeature> i>
  : FeatureSet<name, !strconcat("The device is a part of the ",
               name, " family"), i>;

def FeatureNEW           : SubtargetFeature<"new", "m_hasNEW", "true",
                                  "The device has new features">;

// The device has SRAM, and supports the bare minimum of
// SRAM-relevant instructions.
//
// These are:
// LD - all 9 variants
// ST - all 9 variants
// LDD - two variants for Y and Z
// STD - two variants for Y and Z
// `LDS Rd, K`
// `STS k, Rr`
// `PUSH`/`POP`
def FeatureSRAM           : SubtargetFeature<"sram", "m_hasSRAM", "true",
                                  "The device has random access memory">;

// The device supports the `JMP k` and `CALL k` instructions.
def FeatureJMPCALL        : SubtargetFeature<"jmpcall", "m_hasJMPCALL", "true",
                                  "The device supports the `JMP` and "
                                  "`CALL` instructions">;


// The device supports the indirect branches `IJMP` and `ICALL`.
def FeatureIJMPCALL       : SubtargetFeature<"ijmpcall", "m_hasIJMPCALL",
                                  "true",
                                  "The device supports `IJMP`/`ICALL`"
                                  "instructions">;

// The device supports the extended indirect branches `EIJMP` and `EICALL`.
def FeatureEIJMPCALL      : SubtargetFeature<"eijmpcall", "m_hasEIJMPCALL",
                                  "true", "The device supports the "
                                  "`EIJMP`/`EICALL` instructions">;

// The device supports `ADDI Rd, K`, `SUBI Rd, K`.
def FeatureADDSUBIW       : SubtargetFeature<"addsubiw", "m_hasADDSUBIW",
                                  "true", "Enable 16-bit register-immediate "
                                  "addition and subtraction instructions">;

// The device has an 8-bit stack pointer (SP) register.
def FeatureSmallStack     : SubtargetFeature<"smallstack", "m_hasSmallStack",
                                  "true", "The device has an 8-bit "
                                  "stack pointer">;

// The device supports the 16-bit GPR pair MOVW instruction.
def FeatureMOVW           : SubtargetFeature<"movw", "m_hasMOVW", "true",
                                  "The device supports the 16-bit MOVW "
                                  "instruction">;

// The device supports the `LPM` instruction, with implied destination being r0.
def FeatureLPM            : SubtargetFeature<"lpm", "m_hasLPM", "true",
                                  "The device supports the `LPM` instruction">;

// The device supports the `LPM Rd, Z[+] instruction.
def FeatureLPMX           : SubtargetFeature<"lpmx", "m_hasLPMX", "true",
                                  "The device supports the `LPM Rd, Z[+]` "
                                  "instruction">;

// The device supports the `ELPM` instruction.
def FeatureELPM           : SubtargetFeature<"elpm", "m_hasELPM", "true",
                                  "The device supports the ELPM instruction">;

// The device supports the `ELPM Rd, Z[+]` instructions.
def FeatureELPMX          : SubtargetFeature<"elpmx", "m_hasELPMX", "true",
                                  "The device supports the `ELPM Rd, Z[+]` "
                                  "instructions">;

// The device supports the `SPM` instruction.
def FeatureSPM            : SubtargetFeature<"spm", "m_hasSPM", "true",
                                  "The device supports the `SPM` instruction">;

// The device supports the `SPM Z+` instruction.
def FeatureSPMX           : SubtargetFeature<"spmx", "m_hasSPMX", "true",
                                  "The device supports the `SPM Z+` "
                                  "instruction">;

// The device supports the `DES k` instruction.
def FeatureDES            : SubtargetFeature<"des", "m_hasDES", "true",
                                  "The device supports the `DES k` encryption "
                                  "instruction">;

// The device supports the Read-Write-Modify instructions
// XCH, LAS, LAC, and LAT.
def FeatureRMW            : SubtargetFeature<"rmw", "m_supportsRMW", "true",
                                  "The device supports the read-write-modify "
                                  "instructions: XCH, LAS, LAC, LAT">;

// The device supports the `[F]MUL[S][U]` family of instructions.
def FeatureMultiplication : SubtargetFeature<"mul", "m_supportsMultiplication",
                                  "true", "The device supports the "
                                  "multiplication instructions">;

// The device supports the `BREAK` instruction.
def FeatureBREAK          : SubtargetFeature<"break", "m_hasBREAK", "true",
                                  "The device supports the `BREAK` debugging "
                                  "instruction">;

// The device has instruction encodings specific to the Tiny core.
def FeatureTinyEncoding   : SubtargetFeature<"tinyencoding",
                                  "m_hasTinyEncoding", "true",
                                  "The device has Tiny core specific "
                                  "instruction encodings">;

// The device has CPU registers mapped in data address space
def FeatureMMR : SubtargetFeature<"memmappedregs", "m_hasMemMappedGPR",
                                  "true", "The device has CPU registers "
                                  "mapped in data address space">;

class ELFArch<string name>  : SubtargetFeature<"", "ELFArch",
                                    !strconcat("ELF::",name), "">;

// ELF e_flags architecture values
def ELFArchOPEN81    : ELFArch<"EF_OPEN8_ARCH_OPEN81">;

//===---------------------------------------------------------------------===//
// OPEN8 Families
//===---------------------------------------------------------------------===//

// The device has at least the bare minimum that **every** single OPEN8
// device should have.
def FamilyOPEN80           : Family<"open80", []>;

def FamilyOPEN81           : Family<"open81", [FeatureNEW, FeatureSRAM, FamilyOPEN80, FeatureLPM, FeatureMMR, FeatureSmallStack, ]>;

//===---------------------------------------------------------------------===//
// OPEN8 microcontrollers supported.
//===---------------------------------------------------------------------===//

class Device<string Name, Family Fam, ELFArch Arch,
             list<SubtargetFeature> ExtraFeatures = []>
  : Processor<Name, NoItineraries, !listconcat([Fam,Arch],ExtraFeatures)>;

// Generic MCUs
// Note that several versions of GCC has strange ELF architecture
// settings for backwards compatibility - see `gas/config/tc-open8.c`
// in OPEN8 binutils. We do not replicate this.
def : Device<"open80",      FamilyOPEN80,    ELFArchOPEN81>;
def : Device<"open81",      FamilyOPEN81,    ELFArchOPEN81>;

