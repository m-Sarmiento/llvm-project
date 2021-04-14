//===-- OPEN8RegisterInfo.td - OPEN8 Register defs -------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the OPEN8 register file
//===----------------------------------------------------------------------===//

// 8-bit General purpose register definition.
class OPEN8Reg<bits<16> num,
             string name,
             list<Register> subregs = [],
             list<string> altNames = []>
  : RegisterWithSubRegs<name, subregs>
{
  field bits<16> Num = num;

  let HWEncoding = num;
  let Namespace = "OPEN8";
  let SubRegs = subregs;
  let AltNames = altNames;
}

// Subregister indices.
let Namespace = "OPEN8" in
{
  def sub_lo : SubRegIndex<8>;
  def sub_hi : SubRegIndex<8, 8>;
}

let Namespace = "OPEN8" in {
  def ptr : RegAltNameIndex;
}


//===----------------------------------------------------------------------===//
//  8-bit general purpose registers
//===----------------------------------------------------------------------===//

def R0  : OPEN8Reg<0,  "r0">,  DwarfRegNum<[0]>;
def R1  : OPEN8Reg<1,  "r1">,  DwarfRegNum<[1]>;
def R2  : OPEN8Reg<2,  "r2">,  DwarfRegNum<[2]>;
def R3  : OPEN8Reg<3,  "r3">,  DwarfRegNum<[3]>;
def R4  : OPEN8Reg<4,  "r4">,  DwarfRegNum<[4]>;
def R5  : OPEN8Reg<5,  "r5">,  DwarfRegNum<[5]>;
def R6  : OPEN8Reg<6,  "r6">,  DwarfRegNum<[6]>;
def R7  : OPEN8Reg<7,  "r7">,  DwarfRegNum<[7]>;
def SPL : OPEN8Reg<32, "SPL">, DwarfRegNum<[32]>;
def SPH : OPEN8Reg<33, "SPH">, DwarfRegNum<[33]>;

let SubRegIndices = [sub_lo, sub_hi],
CoveredBySubRegs = 1 in
{
  // 16 bit GPR pairs.
  def SP     : OPEN8Reg<32, "SP",      [SPL, SPH]>, DwarfRegNum<[32]>;

  // The pointer registers (X,Y,Z) are a special case because they
  // are printed as a `high:low` pair when a DREG is expected,
  // but printed using `X`, `Y`, `Z` when a pointer register is expected.
  def R7R6   : OPEN8Reg<6,  "r7:r6",   [R6, R7]>,   DwarfRegNum<[6]>;
  def R5R4   : OPEN8Reg<4,  "r5:r4",   [R4, R5]>,   DwarfRegNum<[4]>;
  def R3R2   : OPEN8Reg<2,  "r3:r2",   [R2, R3]>,   DwarfRegNum<[2]>;
  def R1R0   : OPEN8Reg<0,  "r1:r0",   [R0, R1]>,   DwarfRegNum<[0]>;

  // Pseudo registers for unaligned i16
  /*def R6R5 : OPEN8Reg<15, "r16:r15", [R5, R6]>, DwarfRegNum<[15]>;
  def R4R3 : OPEN8Reg<13, "r14:r13", [R3, R4]>, DwarfRegNum<[13]>;
  def R2R1 : OPEN8Reg<11, "r12:r11", [R1, R2]>, DwarfRegNum<[11]>;*/
}

//===----------------------------------------------------------------------===//
// Register Classes
//===----------------------------------------------------------------------===//

// Main 8-bit register class.
def GPR8 : RegisterClass<"OPEN8", [i8], 8,
  (
    add R7, R6, R5, R4, R3, R2, R1, R0
  )>;

// Simple lower registers r0..r15
def GPR8lo : RegisterClass<"OPEN8", [i8], 8,
  (
    add R7, R6, R5, R4, R3, R2, R1, R0
  )>;

// 8-bit register class for instructions which take immediates.
def LD8 : RegisterClass<"OPEN8", [i8], 8,
  (
    add R7, R6, R5, R4, R3, R2, R1, R0
  )>;

// Simple lower registers r16..r23
def LD8lo : RegisterClass<"OPEN8", [i8], 8,
  (
    add R7, R6, R5, R4, R3, R2, R1, R0
  )>;

// Main 16-bit pair register class.
def DREGS : RegisterClass<"OPEN8", [i16], 8,
  (
    add R7R6, R5R4, R3R2, R1R0
    //PSEUDO
    /*,R2R1
    ,R4R3
    ,R6R5*/
  )>;

// 16-bit pair register class for movw
def DREGSMOVW : RegisterClass<"OPEN8", [i16], 8,
  (
    add R7R6, R5R4, R3R2, R1R0
  )>;

// The 16-bit DREGS register class, excluding the Z pointer register.
//
// This is used by instructions which cause high pointer register
// contention which leads to an assertion in the register allocator.
//
// There is no technical reason why instructions that use this class
// cannot use Z; it's simply a workaround a regalloc bug.
//
// More information can be found in PR39553.
def DREGS_WITHOUT_YZ_WORKAROUND : RegisterClass<"OPEN8", [i16], 8,
  (
    add R7R6, R5R4, R3R2, R1R0
  )>;

// 16-bit register class for the adiw/sbiw instructions.
def IWREGS : RegisterClass<"OPEN8", [i16], 8,
  (
    add R7R6, R5R4, R3R2, R1R0
  )>;

// 16-bit register class for the ld and st instructions.
// AKA X,Y, and Z
def PTRREGS : RegisterClass<"OPEN8", [i16], 8,
  (
    add R7R6, R5R4, R3R2, R1R0
  ), ptr>;

// 16-bit register class for the ldd and std instructions.
// AKA Y and Z.
def PTRDISPREGS : RegisterClass<"OPEN8", [i16], 8,
  (
    add R7R6, R5R4, R3R2, R1R0
  ), ptr>;

// We have a bunch of instructions with an explicit Z register argument. We
// model this using a register class containing only the Z register.
//def ZREG : RegisterClass<"OPEN8", [i16], 8, (add R31R30)>;

// Register class used for the stack read pseudo instruction.
def GPRSP: RegisterClass<"OPEN8", [i16], 8, (add SP)>;

// Status register.
def SREG : OPEN8Reg<14, "PSR">, DwarfRegNum<[88]>;
def CCR : RegisterClass<"OPEN8", [i8], 8, (add SREG)>
{
  let CopyCost = -1;      // Don't allow copying of status registers
}

