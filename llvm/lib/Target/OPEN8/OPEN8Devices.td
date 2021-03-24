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
def ELFArchOPEN82    : ELFArch<"EF_OPEN8_ARCH_OPEN82">;
def ELFArchOPEN825   : ELFArch<"EF_OPEN8_ARCH_OPEN825">;
def ELFArchOPEN83    : ELFArch<"EF_OPEN8_ARCH_OPEN83">;
def ELFArchOPEN831   : ELFArch<"EF_OPEN8_ARCH_OPEN831">;
def ELFArchOPEN835   : ELFArch<"EF_OPEN8_ARCH_OPEN835">;
def ELFArchOPEN84    : ELFArch<"EF_OPEN8_ARCH_OPEN84">;
def ELFArchOPEN85    : ELFArch<"EF_OPEN8_ARCH_OPEN85">;
def ELFArchOPEN851   : ELFArch<"EF_OPEN8_ARCH_OPEN851">;
def ELFArchOPEN86    : ELFArch<"EF_OPEN8_ARCH_OPEN86">;
def ELFArchTiny    : ELFArch<"EF_OPEN8_ARCH_OPEN8TINY">;
def ELFArchXMEGA1  : ELFArch<"EF_OPEN8_ARCH_XMEGA1">;
def ELFArchXMEGA2  : ELFArch<"EF_OPEN8_ARCH_XMEGA2">;
def ELFArchXMEGA3  : ELFArch<"EF_OPEN8_ARCH_XMEGA3">;
def ELFArchXMEGA4  : ELFArch<"EF_OPEN8_ARCH_XMEGA4">;
def ELFArchXMEGA5  : ELFArch<"EF_OPEN8_ARCH_XMEGA5">;
def ELFArchXMEGA6  : ELFArch<"EF_OPEN8_ARCH_XMEGA6">;
def ELFArchXMEGA7  : ELFArch<"EF_OPEN8_ARCH_XMEGA7">;

//===---------------------------------------------------------------------===//
// OPEN8 Families
//===---------------------------------------------------------------------===//

// The device has at least the bare minimum that **every** single OPEN8
// device should have.
def FamilyOPEN80           : Family<"open80", []>;

def FamilyOPEN81           : Family<"open81", [FamilyOPEN80, FeatureLPM, FeatureMMR]>;

def FamilyOPEN82           : Family<"open82",
                                 [FamilyOPEN81, FeatureIJMPCALL, FeatureADDSUBIW,
                                  FeatureSRAM]>;

def FamilyOPEN825          : Family<"open825",
                                 [FamilyOPEN82, FeatureMOVW, FeatureLPMX,
                                  FeatureSPM, FeatureBREAK]>;

def FamilyOPEN83           : Family<"open83",
                                 [FamilyOPEN82, FeatureJMPCALL]>;

def FamilyOPEN831          : Family<"open831",
                                 [FamilyOPEN83, FeatureELPM]>;

def FamilyOPEN835          : Family<"open835",
                                 [FamilyOPEN83, FeatureMOVW, FeatureLPMX,
                                  FeatureSPM, FeatureBREAK]>;

def FamilyOPEN84           : Family<"open84",
                                 [FamilyOPEN82, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK]>;

def FamilyOPEN85           : Family<"open85",
                                 [FamilyOPEN83, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK]>;

def FamilyOPEN851          : Family<"open851",
                                 [FamilyOPEN85, FeatureELPM, FeatureELPMX]>;

def FamilyOPEN86           : Family<"open86",
                                 [FamilyOPEN851]>;

def FamilyTiny           : Family<"open8tiny",
                                 [FamilyOPEN80, FeatureBREAK, FeatureSRAM,
                                  FeatureTinyEncoding]>;

def FamilyXMEGA          : Family<"xmega",
                                 [FamilyOPEN80, FeatureLPM, FeatureIJMPCALL, FeatureADDSUBIW,
                                  FeatureSRAM, FeatureJMPCALL, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK, FeatureEIJMPCALL, FeatureSPMX,
                                  FeatureDES, FeatureELPM, FeatureELPMX]>;

def FamilyXMEGAU         : Family<"xmegau",
                                  [FamilyXMEGA, FeatureRMW]>;

def FeatureSetSpecial    : FeatureSet<"special",
                                      "Enable use of the entire instruction "
                                      "set - used for debugging",
                                      [FeatureSRAM, FeatureJMPCALL,
                                       FeatureIJMPCALL, FeatureEIJMPCALL,
                                       FeatureADDSUBIW, FeatureMOVW,
                                       FeatureLPM, FeatureLPMX, FeatureELPM,
                                       FeatureELPMX, FeatureSPM, FeatureSPMX,
                                       FeatureDES, FeatureRMW,
                                       FeatureMultiplication, FeatureBREAK, FeatureMMR]>;

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
def : Device<"open81",      FamilyOPEN81,    ELFArchOPEN81>;
def : Device<"open82",      FamilyOPEN82,    ELFArchOPEN82>;
def : Device<"open825",     FamilyOPEN825,   ELFArchOPEN825>;
def : Device<"open83",      FamilyOPEN83,    ELFArchOPEN83>;
def : Device<"open831",     FamilyOPEN831,   ELFArchOPEN831>;
def : Device<"open835",     FamilyOPEN835,   ELFArchOPEN835>;
def : Device<"open84",      FamilyOPEN84,    ELFArchOPEN84>;
def : Device<"open85",      FamilyOPEN85,    ELFArchOPEN85>;
def : Device<"open851",     FamilyOPEN851,   ELFArchOPEN851>;
def : Device<"open86",      FamilyOPEN86,    ELFArchOPEN86>;
def : Device<"open8xmega1", FamilyXMEGA,   ELFArchXMEGA1>;
def : Device<"open8xmega2", FamilyXMEGA,   ELFArchXMEGA2>;
def : Device<"open8xmega3", FamilyXMEGA,   ELFArchXMEGA3>;
def : Device<"open8xmega4", FamilyXMEGA,   ELFArchXMEGA4>;
def : Device<"open8xmega5", FamilyXMEGA,   ELFArchXMEGA5>;
def : Device<"open8xmega6", FamilyXMEGA,   ELFArchXMEGA6>;
def : Device<"open8xmega7", FamilyXMEGA,   ELFArchXMEGA7>;
def : Device<"open8tiny",   FamilyTiny,    ELFArchTiny>;

// Specific MCUs
def : Device<"at90s1200",          FamilyOPEN80, ELFArchOPEN81>;
def : Device<"attiny11",           FamilyOPEN81, ELFArchOPEN81>;
def : Device<"attiny12",           FamilyOPEN81, ELFArchOPEN81>;
def : Device<"attiny15",           FamilyOPEN81, ELFArchOPEN81>;
def : Device<"attiny28",           FamilyOPEN81, ELFArchOPEN81>;
def : Device<"at90s2313",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90s2323",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90s2333",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90s2343",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"attiny22",           FamilyOPEN82, ELFArchOPEN82>;
def : Device<"attiny26",           FamilyOPEN82, ELFArchOPEN82, [FeatureLPMX]>;
def : Device<"at86rf401",          FamilyOPEN82, ELFArchOPEN825,
             [FeatureMOVW, FeatureLPMX]>;
def : Device<"at90s4414",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90s4433",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90s4434",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90s8515",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90c8534",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"at90s8535",          FamilyOPEN82, ELFArchOPEN82>;
def : Device<"ata5272",            FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny13",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny13a",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny2313",         FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny2313a",        FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny24",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny24a",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny4313",         FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny44",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny44a",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny84",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny84a",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny25",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny45",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny85",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny261",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny261a",         FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny441",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny461",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny461a",         FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny841",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny861",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny861a",         FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny87",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny43u",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny48",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny88",           FamilyOPEN825, ELFArchOPEN825>;
def : Device<"attiny828",          FamilyOPEN825, ELFArchOPEN825>;
def : Device<"at43usb355",         FamilyOPEN83,  ELFArchOPEN83>;
def : Device<"at76c711",           FamilyOPEN83,  ELFArchOPEN83>;
def : Device<"atmega103",          FamilyOPEN831, ELFArchOPEN831>;
def : Device<"at43usb320",         FamilyOPEN831, ELFArchOPEN831>;
def : Device<"attiny167",          FamilyOPEN835, ELFArchOPEN835>;
def : Device<"at90usb82",          FamilyOPEN835, ELFArchOPEN835>;
def : Device<"at90usb162",         FamilyOPEN835, ELFArchOPEN835>;
def : Device<"ata5505",            FamilyOPEN835, ELFArchOPEN835>;
def : Device<"atmega8u2",          FamilyOPEN835, ELFArchOPEN835>;
def : Device<"atmega16u2",         FamilyOPEN835, ELFArchOPEN835>;
def : Device<"atmega32u2",         FamilyOPEN835, ELFArchOPEN835>;
def : Device<"attiny1634",         FamilyOPEN835, ELFArchOPEN835>;
def : Device<"atmega8",            FamilyOPEN82,  ELFArchOPEN84,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"ata6289",            FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega8a",           FamilyOPEN82,  ELFArchOPEN84,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"ata6285",            FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"ata6286",            FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega48",           FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega48a",          FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega48pa",         FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega48pb",         FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega48p",          FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega88",           FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega88a",          FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega88p",          FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega88pa",         FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega88pb",         FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"atmega8515",         FamilyOPEN82,  ELFArchOPEN84,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8535",         FamilyOPEN82,  ELFArchOPEN84,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8hva",         FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"at90pwm1",           FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"at90pwm2",           FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"at90pwm2b",          FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"at90pwm3",           FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"at90pwm3b",          FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"at90pwm81",          FamilyOPEN84,  ELFArchOPEN84>;
def : Device<"ata5790",            FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"ata5795",            FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16",           FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16a",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega161",          FamilyOPEN83,  ELFArchOPEN85,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega162",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega163",          FamilyOPEN83,  ELFArchOPEN85,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega164a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega164p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega164pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega165",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega165a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega165p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega165pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega168",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega168a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega168p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega168pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega168pb",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega169",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega169a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega169p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega169pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32",           FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32a",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega323",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega324a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega324p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega324pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega324pb",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega325",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega325a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega325p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega325pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3250",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3250a",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3250p",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3250pa",       FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega328",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega328p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega328pb",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega329",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega329a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega329p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega329pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3290",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3290a",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3290p",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega3290pa",       FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega406",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega64",           FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega64a",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega640",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega644",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega644a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega644p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega644pa",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega645",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega645a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega645p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega649",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega649a",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega649p",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega6450",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega6450a",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega6450p",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega6490",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega6490a",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega6490p",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega64rfr2",       FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega644rfr2",      FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16hva",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16hva2",       FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16hvb",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16hvbrevb",    FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32hvb",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32hvbrevb",    FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega64hve",        FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90can32",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90can64",          FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90pwm161",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90pwm216",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90pwm316",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32c1",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega64c1",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16m1",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32m1",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega64m1",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega16u4",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32u4",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega32u6",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90usb646",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90usb647",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at90scr100",         FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"at94k",              FamilyOPEN83,  ELFArchOPEN85,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX]>;
def : Device<"m3000",              FamilyOPEN85,  ELFArchOPEN85>;
def : Device<"atmega128",          FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega128a",         FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega1280",         FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega1281",         FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega1284",         FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega1284p",        FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega128rfa1",      FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega128rfr2",      FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega1284rfr2",     FamilyOPEN851, ELFArchOPEN851>;
def : Device<"at90can128",         FamilyOPEN851, ELFArchOPEN851>;
def : Device<"at90usb1286",        FamilyOPEN851, ELFArchOPEN851>;
def : Device<"at90usb1287",        FamilyOPEN851, ELFArchOPEN851>;
def : Device<"atmega2560",         FamilyOPEN86,  ELFArchOPEN86>;
def : Device<"atmega2561",         FamilyOPEN86,  ELFArchOPEN86>;
def : Device<"atmega256rfr2",      FamilyOPEN86,  ELFArchOPEN86>;
def : Device<"atmega2564rfr2",     FamilyOPEN86,  ELFArchOPEN86>;
def : Device<"atxmega16a4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega16a4u",       FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16c4",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16d4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4u",       FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32c4",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32d4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32e5",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16e5",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega8e5",         FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32x1",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega64a3",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a3u",       FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64a4u",       FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b1",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b3",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64c3",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64d3",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64d4",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a1",        FamilyXMEGA, ELFArchXMEGA5>;
def : Device<"atxmega64a1u",       FamilyXMEGAU, ELFArchXMEGA5>;
def : Device<"atxmega128a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b1",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128d4",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256a3b",      FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3bu",     FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega384c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega384d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a1",       FamilyXMEGA, ELFArchXMEGA7>;
def : Device<"atxmega128a1u",      FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"atxmega128a4u",      FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"attiny4",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny5",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny9",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny10",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny20",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny40",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny102",          FamilyTiny, ELFArchTiny>;
def : Device<"attiny104",          FamilyTiny, ELFArchTiny>;

