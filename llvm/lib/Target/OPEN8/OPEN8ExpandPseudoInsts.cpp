//===-- OPEN8ExpandPseudoInsts.cpp - Expand pseudo instructions -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "OPEN8.h"
#include "OPEN8InstrInfo.h"
#include "OPEN8TargetMachine.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define OPEN8_EXPAND_PSEUDO_NAME "OPEN8 pseudo instruction expansion pass"

namespace {

/// Expands "placeholder" instructions marked as pseudo into
/// actual OPEN8 instructions.
class OPEN8ExpandPseudo : public MachineFunctionPass {
public:
  static char ID;

  OPEN8ExpandPseudo() : MachineFunctionPass(ID) {
    initializeOPEN8ExpandPseudoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return OPEN8_EXPAND_PSEUDO_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const OPEN8RegisterInfo *TRI;
  const TargetInstrInfo *TII;

  /// The register to be used for temporary storage.
  const unsigned SCRATCH_REGISTER = OPEN8::R0;

  bool expandMBB(Block &MBB);
  bool expandMI(Block &MBB, BlockIt MBBI);
  template <unsigned OP> bool expand(Block &MBB, BlockIt MBBI);

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode));
  }

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode,
                              Register DstReg) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode), DstReg);
  }

  MachineRegisterInfo &getRegInfo(Block &MBB) { return MBB.getParent()->getRegInfo(); }

  bool expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI);
  bool expandLogic(unsigned Op, Block &MBB, BlockIt MBBI);
  //bool expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI);
  bool expandMoveAcc(unsigned Op, Block &MBB, BlockIt MBBI);
  //bool isLogicImmOpRedundant(unsigned Op, unsigned ImmVal) const;

/*  template<typename Func>
  bool expandAtomic(Block &MBB, BlockIt MBBI, Func f);

  template<typename Func>
  bool expandAtomicBinaryOp(unsigned Opcode, Block &MBB, BlockIt MBBI, Func f);

  bool expandAtomicBinaryOp(unsigned Opcode, Block &MBB, BlockIt MBBI);

  bool expandAtomicArithmeticOp(unsigned MemOpcode,
                                unsigned ArithOpcode,
                                Block &MBB,
                                BlockIt MBBI);
*/
  /// Scavenges a free GPR8 register for use.
  Register scavengeGPR8(MachineInstr &MI);
};

char OPEN8ExpandPseudo::ID = 0;

bool OPEN8ExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool OPEN8ExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  TRI = STI.getRegisterInfo();
  TII = STI.getInstrInfo();

  // We need to track liveness in order to use register scavenging.
  MF.getProperties().set(MachineFunctionProperties::Property::TracksLiveness);

  for (Block &MBB : MF) {
    bool ContinueExpanding = true;
    unsigned ExpandCount = 0;

    // Continue expanding the block until all pseudos are expanded.
    do {
      assert(ExpandCount < 10 && "pseudo expand limit reached");

      bool BlockModified = expandMBB(MBB);
      Modified |= BlockModified;
      ExpandCount++;

      ContinueExpanding = BlockModified;
    } while (ContinueExpanding);
  }

  return Modified;
}

bool OPEN8ExpandPseudo::
expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

bool OPEN8ExpandPseudo::
expandLogic(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, Op)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(3).setIsDead();

  auto MIBHI = buildMI(MBB, MBBI, Op)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool OPEN8ExpandPseudo::
expandMoveAcc(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  //bool ImpIsDead = MI.getOperand(3).isDead();

  if(DstReg != OPEN8::R0  & SrcReg != OPEN8::R0){
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, Op).addReg(SrcReg,getKillRegState(SrcIsKill));
    buildMI(MBB, MBBI, OPEN8::T0X).addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));
  } else if (DstReg == OPEN8::R0 & SrcReg != OPEN8::R0){
    buildMI(MBB, MBBI, Op).addReg(DstReg,getKillRegState(SrcIsKill));
  } else if (DstReg != OPEN8::R0 & SrcReg == OPEN8::R0){
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R0);
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(DstReg);
    buildMI(MBB, MBBI, Op).addReg(DstReg,getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::T0X).addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));
  } else {
    buildMI(MBB, MBBI, Op).addReg(DstReg,getKillRegState(DstIsKill));
  }

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ADDWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(OPEN8::ADDRdRr, OPEN8::ADCRdRr, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ADCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(OPEN8::ADCRdRr, OPEN8::ADCRdRr, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SUBWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(OPEN8::SUBRdRr, OPEN8::SBCRdRr, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SBCIRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  unsigned imm = MI.getOperand(2).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  //bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsDead = MI.getOperand(1).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned Op0 = OPEN8::TX0;
  unsigned Op1 = OPEN8::LDIRdk;
  unsigned Op2 = OPEN8::SBC;
  unsigned Op3 = OPEN8::T0X;

  if(SrcReg != OPEN8::R0 ){
    buildMI(MBB, MBBI, Op0).addReg(SrcReg, RegState::Define | getDeadRegState(SrcIsDead));  
  } else{
    llvm_unreachable("src can't be r0");
  }
  buildMI(MBB, MBBI, Op1).addReg(SrcReg).addImm(imm);

  buildMI(MBB, MBBI, Op2).addReg(SrcReg,getKillRegState(SrcIsKill));

  if(DstReg != OPEN8::R0 ){
    buildMI(MBB, MBBI, Op3).addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));  
  }
  
  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SUBIRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  unsigned imm = MI.getOperand(2).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  //bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsDead = MI.getOperand(1).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned Op0 = OPEN8::TX0;
  unsigned Op1 = OPEN8::LDIRdk;
  unsigned Op2 = OPEN8::CLP;
  unsigned Op3 = OPEN8::SBC;
  unsigned Op4 = OPEN8::T0X;

  if(SrcReg != OPEN8::R0 ){
    buildMI(MBB, MBBI, Op0).addReg(SrcReg, RegState::Define | getDeadRegState(SrcIsDead));  
  } else{
    llvm_unreachable("src can't be r0");
  }
  buildMI(MBB, MBBI, Op1).addReg(SrcReg).addImm(imm);
  buildMI(MBB, MBBI, Op2).addImm(1);
  buildMI(MBB, MBBI, Op3).addReg(SrcReg,getKillRegState(SrcIsKill));

  if(DstReg != OPEN8::R0 ){
    buildMI(MBB, MBBI, Op4).addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));  
  }
  
  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SUBIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);
  auto MIBLO = buildMI(MBB, MBBI, OPEN8::SUBIRdK)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OPEN8::SBCIRdK)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(SrcIsKill));

  switch (MI.getOperand(2).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    llvm_unreachable("MachineOperand::MO_GlobalAddress");
    break;
    /*const GlobalValue *GV = MI.getOperand(2).getGlobal();
    int64_t Offs = MI.getOperand(2).getOffset();
    unsigned TF = MI.getOperand(2).getTargetFlags();
    MIBLO.addGlobalAddress(GV, Offs, TF | AVRII::MO_NEG | AVRII::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | AVRII::MO_NEG | AVRII::MO_HI);
    break;*/
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(2).getImm();
    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SBCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(OPEN8::SBCRdRr, OPEN8::SBCRdRr, MBB, MBBI);
}
/*
template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SBCIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  unsigned Imm = MI.getOperand(2).getImm();
  unsigned Lo8 = Imm & 0xff;
  unsigned Hi8 = (Imm >> 8) & 0xff;
  unsigned OpLo = OPEN8::SBCIRdK;
  unsigned OpHi = OPEN8::SBCIRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(SrcIsKill))
    .addImm(Lo8);

  // SREG is always implicitly killed
  MIBLO->getOperand(4).setIsKill();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(SrcIsKill))
    .addImm(Hi8);

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}
*/
template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ANDWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(OPEN8::ANDRdRr, MBB, MBBI);
}

/*template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ANDIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(OPEN8::ANDIRdK, MBB, MBBI);
}*/

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(OPEN8::ORRdRr, MBB, MBBI);
}

/*template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ORIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(OPEN8::ORIRdK, MBB, MBBI);
}*/

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::EORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(OPEN8::EORRdRr, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::EORRdRr>(Block &MBB, BlockIt MBBI) {
  return expandMoveAcc(OPEN8::XOR, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ORRdRr>(Block &MBB, BlockIt MBBI) {
  return expandMoveAcc(OPEN8::OR, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ANDRdRr>(Block &MBB, BlockIt MBBI) {
  return expandMoveAcc(OPEN8::AND, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ADDRdRr>(Block &MBB, BlockIt MBBI) {
  return expandMoveAcc(OPEN8::ADD, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ADCRdRr>(Block &MBB, BlockIt MBBI) {
  return expandMoveAcc(OPEN8::ADC, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SUBRdRr>(Block &MBB, BlockIt MBBI) {
  //clear carry flag TODO: carry flag lost?
  buildMI(MBB, MBBI, OPEN8::CLP).addImm(0x1);
  return expandMoveAcc(OPEN8::SBC, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SBCRdRr>(Block &MBB, BlockIt MBBI) {
  return expandMoveAcc(OPEN8::SBC, MBB, MBBI);
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::MULRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Op0, Op1;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  /*unsigned DstReg = MI.getOperand(0).getReg();
  unsigned SrcReg = MI.getOperand(2).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();*/
  Op0 = OPEN8::TX0;
  Op1 = OPEN8::MUL;
  if(SrcReg != OPEN8::R0 ){
    /*auto MI1 = */buildMI(MBB, MBBI, Op0)
    .addReg(SrcReg, getKillRegState(SrcIsKill));
  }
  /*auto MI2 = */buildMI(MBB, MBBI, Op1)
    .addReg(DstReg, getKillRegState(DstIsKill));

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::MULSRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Op0, Op1;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  /*unsigned DstReg = MI.getOperand(0).getReg();
  unsigned SrcReg = MI.getOperand(2).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();*/
  Op0 = OPEN8::TX0;
  Op1 = OPEN8::MUL;
  if(SrcReg != OPEN8::R0 ){
    /*auto MI1 = */buildMI(MBB, MBBI, Op0)
    .addReg(SrcReg, getKillRegState(SrcIsKill));
  }
  /*auto MI2 = */buildMI(MBB, MBBI, Op1)
    .addReg(DstReg, getKillRegState(DstIsKill));

  MI.eraseFromParent();
  return true;
}

/*template <>
bool OPEN8ExpandPseudo::expand<OPEN8::COMWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = OPEN8::COMRd;
  unsigned OpHi = OPEN8::COMRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(2).setIsDead();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  MI.eraseFromParent();
  return true;
}*/

/*template <>
bool OPEN8ExpandPseudo::expand<OPEN8::NEGWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Do NEG on the upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, OPEN8::NEGRd)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  // SREG is always implicitly dead
  MIBHI->getOperand(2).setIsDead();

  // Do NEG on the lower byte.
  buildMI(MBB, MBBI, OPEN8::NEGRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // Do an extra SBC.
  auto MISBCI =
      buildMI(MBB, MBBI, OPEN8::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(OPEN8::R0);
          //.addReg(ZERO_REGISTER);
  if (ImpIsDead)
    MISBCI->getOperand(3).setIsDead();
  // SREG is always implicitly killed
  MISBCI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}*/

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::CPWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = OPEN8::CPRdRr;
  unsigned OpHi = OPEN8::CPCRdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::CPCWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = OPEN8::CPCRdRr;
  unsigned OpHi = OPEN8::CPCRdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDIWRdk>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = OPEN8::LDIRdk;
  unsigned OpHi = OPEN8::LDIRdk;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF | OPEN8II::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | OPEN8II::MO_HI);
    break;
  }
  case MachineOperand::MO_BlockAddress: {
    const BlockAddress *BA = MI.getOperand(1).getBlockAddress();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.add(MachineOperand::CreateBA(BA, TF | OPEN8II::MO_LO));
    MIBHI.add(MachineOperand::CreateBA(BA, TF | OPEN8II::MO_HI));
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDAWRdk>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = OPEN8::LDARdk;
  unsigned OpHi = OPEN8::LDARdk;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF);
    MIBHI.addGlobalAddress(GV, Offs + 1, TF);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm);
    MIBHI.addImm(Imm + 1);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDDRdQ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register sp = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();
  Register Reg = MI.getOperand(0).getReg();
  unsigned spIsKill = MI.getOperand(1).isKill();
  unsigned RegIsKill = MI.getOperand(0).isKill();

  unsigned Op0 = OPEN8::LDO;
  unsigned Op1 = OPEN8::T0X;

  /*auto MIBHI = */buildMI(MBB, MBBI, Op0)
    .addReg(sp, getKillRegState(spIsKill))
    .addImm(Imm);
  if (Reg != OPEN8::R0)
  /*auto MIBLO = */buildMI(MBB, MBBI, Op1)
    .addReg(Reg, getKillRegState(RegIsKill));

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = OPEN8::LDRd;
  unsigned OpHi = OPEN8::LDDRdQ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(CurDstLoReg, RegState::Define)
                   .addReg(SrcReg, RegState::Define);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(CurDstHiReg, RegState::Define)
    .addReg(SrcReg, getKillRegState(SrcIsKill))
    .addImm(1);

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, OPEN8::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, OPEN8::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDWRdPi>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsDead = MI.getOperand(1).isKill();
  unsigned OpLo = OPEN8::LDRdPi;
  unsigned OpHi = OPEN8::LDRdPi;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(SrcReg, RegState::Define)
    .addReg(SrcReg, RegState::Kill);

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(SrcReg, RegState::Define | getDeadRegState(SrcIsDead))
    .addReg(SrcReg, RegState::Kill);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}


template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDDWRdQ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = OPEN8::LDDRdQ;
  unsigned OpHi = OPEN8::LDDRdQ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 254 && "Offset is out of range");

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(CurDstLoReg, RegState::Define)
    .addReg(SrcReg)
    .addImm(Imm);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(CurDstHiReg, RegState::Define)
    .addReg(SrcReg, getKillRegState(SrcIsKill))
    .addImm(Imm + 1);

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, OPEN8::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, OPEN8::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

Register OPEN8ExpandPseudo::scavengeGPR8(MachineInstr &MI) {
  MachineBasicBlock &MBB = *MI.getParent();
  RegScavenger RS;

  RS.enterBasicBlock(MBB);
  RS.forward(MI);

  BitVector Candidates =
      TRI->getAllocatableSet
      (*MBB.getParent(), &OPEN8::GPR8RegClass);

  // Exclude all the registers being used by the instruction.
  for (MachineOperand &MO : MI.operands()) {
    if (MO.isReg() && MO.getReg() != 0 && !MO.isDef() &&
        !Register::isVirtualRegister(MO.getReg()))
      Candidates.reset(MO.getReg());
  }

  BitVector Available = RS.getRegsAvailable(&OPEN8::GPR8RegClass);
  Available &= Candidates;

  signed Reg = Available.find_first();
  assert(Reg != -1 && "ran out of registers");
  return Reg;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::STAWKRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = OPEN8::STAKRr;
  unsigned OpHi = OPEN8::STAKRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Write the high byte first in case this address belongs to a special
  // I/O address with a special temporary register.
  auto MIBHI = buildMI(MBB, MBBI, OpHi);
  auto MIBLO = buildMI(MBB, MBBI, OpLo);

  switch (MI.getOperand(0).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(0).getGlobal();
    int64_t Offs = MI.getOperand(0).getOffset();
    unsigned TF = MI.getOperand(0).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF);
    MIBHI.addGlobalAddress(GV, Offs + 1, TF);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(0).getImm();

    MIBLO.addImm(Imm);
    MIBHI.addImm(Imm + 1);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MIBLO.addReg(SrcLoReg, getKillRegState(SrcIsKill));
  MIBHI.addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::STDQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register sp = MI.getOperand(0).getReg();
  unsigned Imm = MI.getOperand(1).getImm();
  Register Reg = MI.getOperand(2).getReg();
  unsigned spIsKill = MI.getOperand(0).isKill();
  unsigned RegIsKill = MI.getOperand(2).isKill();

  unsigned Op0 = OPEN8::TX0;
  unsigned Op1 = OPEN8::STO;

  if (Reg != OPEN8::R0)
  buildMI(MBB, MBBI, Op0)
    .addReg(Reg, getKillRegState(RegIsKill));

  buildMI(MBB, MBBI, Op1)
    .addReg(sp, getKillRegState(spIsKill))
    .addImm(Imm);

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::STWRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsUndef = MI.getOperand(0).isUndef();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = OPEN8::STRr;
  unsigned OpHi = OPEN8::STDQRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  //:TODO: need to reverse this order like inw and stsw?
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg, getUndefRegState(DstIsUndef))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, getUndefRegState(DstIsUndef))
    .addImm(1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::STWPiRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(3).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = OPEN8::STPiRr;
  unsigned OpHi = OPEN8::STPiRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg, RegState::Define)
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::STDWQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(1).getImm();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = OPEN8::STDQRr;
  unsigned OpHi = OPEN8::STDQRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 255 && "Offset is out of range");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg)
    .addImm(Imm)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, getKillRegState(DstIsKill))
    .addImm(Imm + 1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

/*
template <>
bool OPEN8ExpandPseudo::expand<OPEN8::INWRdA>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  unsigned Imm = MI.getOperand(1).getImm();
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = OPEN8::INRdA;
  unsigned OpHi = OPEN8::INRdA;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);
  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(Imm);
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(Imm + 1);
  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());
  MI.eraseFromParent();
  return true;
}
template <>
bool OPEN8ExpandPseudo::expand<OPEN8::OUTWARr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  unsigned Imm = MI.getOperand(0).getImm();
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = OPEN8::OUTARr;
  unsigned OpHi = OPEN8::OUTARr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");
  // 16 bit I/O writes need the high byte first
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addImm(Imm + 1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addImm(Imm)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));
  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());
  MI.eraseFromParent();
  return true;
}
*/
template <>
bool OPEN8ExpandPseudo::expand<OPEN8::PUSHWRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(0).isKill();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = OPEN8::PUSHRr;
  unsigned OpHi = OPEN8::PUSHRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::POPWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = OPEN8::POPRd;
  unsigned OpHi = OPEN8::POPRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpHi, DstHiReg).setMIFlags(Flags); // High
  buildMI(MBB, MBBI, OpLo, DstLoReg).setMIFlags(Flags); // Low

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ROLBRd>(Block &MBB, BlockIt MBBI) {
  // In OPEN8, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  unsigned OpShift, OpCarry;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  OpShift = OPEN8::ADDRdRr;
  OpCarry = OPEN8::ADCRdRr;

  // add r16, r16
  // adc r16, r1

  // Shift part
  buildMI(MBB, MBBI, OpShift)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(DstReg);

  // Add the carry bit
  auto MIB = buildMI(MBB, MBBI, OpCarry)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(OPEN8::R1);
    //ZERO_REGISTER

  // SREG is always implicitly killed
  MIB->getOperand(2).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::RORBRd>(Block &MBB, BlockIt MBBI) {
  // In OPEN8, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  unsigned OpShiftOut, OpLoad, OpShiftIn, OpAdd;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  OpShiftOut = OPEN8::LSRRd;
  OpLoad = OPEN8::LDIRdk;
  OpShiftIn = OPEN8::RORRd;
  OpAdd = OPEN8::ORRdRr;

  // lsr r16
  // ldi r0, 0
  // ror r0
  // or r16, r17

  // Shift out
  buildMI(MBB, MBBI, OpShiftOut)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg);

  // Put 0 in temporary register
  buildMI(MBB, MBBI, OpLoad)
    .addReg(SCRATCH_REGISTER, RegState::Define | getDeadRegState(true))
    .addImm(0x00);

  // Shift in
  buildMI(MBB, MBBI, OpShiftIn)
    .addReg(SCRATCH_REGISTER, RegState::Define | getDeadRegState(true))
    .addReg(SCRATCH_REGISTER);

  // Add the results together using an or-instruction
  auto MIB = buildMI(MBB, MBBI, OpAdd)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(SCRATCH_REGISTER);

  // SREG is always implicitly killed
  MIB->getOperand(2).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSLWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = OPEN8::ADDRdRr; // ADD Rd, Rd <==> LSL Rd
  unsigned OpHi = OPEN8::ADCRdRr; // ADC Rd, Rd <==> ROL Rd
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg)
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg)
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSLW4Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // swap Rh
  // swap Rl
  buildMI(MBB, MBBI, OPEN8::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));
  buildMI(MBB, MBBI, OPEN8::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // andi Rh, 0xf0
  buildMI(MBB, MBBI, OPEN8::LDIRdk)
                            .addReg(OPEN8::R1)
                            .addImm(0xf0);

  /*auto MI0 =*/
      buildMI(MBB, MBBI, OPEN8::ANDRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  // SREG is implicitly dead.
  //MI0->getOperand(3).setIsDead();

  // eor Rh, Rl
  /*auto MI1 =*/
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg);
  // SREG is implicitly dead.
  //MI1->getOperand(3).setIsDead();

  // andi Rl, 0xf0
  buildMI(MBB, MBBI, OPEN8::LDIRdk)
                            .addReg(OPEN8::R1)
                            .addImm(0xf0);
  auto MI2 =
      buildMI(MBB, MBBI, OPEN8::ANDRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill));
          //.addImm(0xf0);
  // SREG is implicitly dead.
  MI2->getOperand(3).setIsDead();

  // eor Rh, Rl
  auto MI3 =
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg);
  if (ImpIsDead)
    MI3->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSLW8Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // mov Rh, Rl
  buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg);

  // clr Rl
  auto MIBLO =
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBLO->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSLW12Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // mov Rh, Rl
  buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg);

  // swap Rh
  buildMI(MBB, MBBI, OPEN8::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));
/*
  // andi Rh, 0xf0
  auto MI0 =
      buildMI(MBB, MBBI, OPEN8::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addImm(0xf0);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();
*/
  // clr Rl
  auto MI1 =
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MI1->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = OPEN8::RORRd;
  unsigned OpHi = OPEN8::LSRRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSRW4Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // swap Rh
  // swap Rl
  buildMI(MBB, MBBI, OPEN8::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));
  buildMI(MBB, MBBI, OPEN8::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));
/*
  // andi Rl, 0xf
  auto MI0 =
      buildMI(MBB, MBBI, OPEN8::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();
*/
  // eor Rl, Rh
  auto MI1 =
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg);
  // SREG is implicitly dead.
  MI1->getOperand(3).setIsDead();
/*
  // andi Rh, 0xf
  auto MI2 =
      buildMI(MBB, MBBI, OPEN8::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI2->getOperand(3).setIsDead();
*/
  // eor Rl, Rh
  auto MI3 =
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg);
  if (ImpIsDead)
    MI3->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSRW8Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // Clear upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSRW12Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // swap Rl
  buildMI(MBB, MBBI, OPEN8::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));
/*
  // andi Rl, 0xf
  auto MI0 =
      buildMI(MBB, MBBI, OPEN8::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();
*/
  // Clear upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, OPEN8::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::RORWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("RORW unimplemented");
  return false;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ROLWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("ROLW unimplemented");
  return false;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ASRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpHi = OPEN8::ASRRd;
  unsigned OpLo = OPEN8::RORRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ASRW8Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // Move the sign bit to the C flag.
  buildMI(MBB, MBBI, OPEN8::ADDRdRr)
      .addReg(DstHiReg, RegState::Define, getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill) | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));

  // Set upper byte to 0 or -1.
  auto MIBHI =
      buildMI(MBB, MBBI, OPEN8::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSLB7Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();

  // ror r24
  // clr r24
  // ror r24

  buildMI(MBB, MBBI, OPEN8::RORRd)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      ->getOperand(3).setIsUndef(true);

  buildMI(MBB, MBBI, OPEN8::EORRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  auto MIRRC =
      buildMI(MBB, MBBI, OPEN8::RORRd)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSRB7Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();

  // rol r24
  // clr r24
  // rol r24

  buildMI(MBB, MBBI, OPEN8::ADCRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill))
      ->getOperand(4).setIsUndef(true);

  buildMI(MBB, MBBI, OPEN8::EORRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  auto MIRRC =
      buildMI(MBB, MBBI, OPEN8::ADCRdRr)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, getKillRegState(DstIsKill))
          .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ASRB7Rd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();

  // lsl r24
  // sbc r24, r24

  buildMI(MBB, MBBI, OPEN8::ADDRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  auto MIRRC = buildMI(MBB, MBBI, OPEN8::SBCRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <> bool OPEN8ExpandPseudo::expand<OPEN8::SEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  // sext R17:R16, R17
  // mov     r16, r17
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R13
  // mov     r16, r13
  // mov     r17, r13
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R16
  // mov     r17, r16
  // lsl     r17
  // sbc     r17, r17
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    auto MOV = buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(SrcReg);

    if (SrcReg == DstHiReg) {
      MOV->getOperand(1).setIsKill();
    }
  }

  if (SrcReg != DstHiReg) {
    buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstHiReg, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  buildMI(MBB, MBBI, OPEN8::ADDRdRr) // LSL Rd <==> ADD Rd, Rr
    .addReg(DstHiReg, RegState::Define)
    .addReg(DstHiReg)
    .addReg(DstHiReg, RegState::Kill);

  auto SBC = buildMI(MBB, MBBI, OPEN8::SBCRdRr)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, RegState::Kill)
    .addReg(DstHiReg, RegState::Kill);

  if (ImpIsDead)
    SBC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  SBC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <> bool OPEN8ExpandPseudo::expand<OPEN8::ZEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  // zext R25:R24, R20
  // mov      R24, R20
  // eor      R25, R25
  // zext R25:R24, R24
  // eor      R25, R25
  // zext R25:R24, R25
  // mov      R24, R25
  // eor      R25, R25
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    buildMI(MBB, MBBI, OPEN8::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  auto EOR = buildMI(MBB, MBBI, OPEN8::EORRdRr)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, RegState::Kill | RegState::Undef)
    .addReg(DstHiReg, RegState::Kill | RegState::Undef);

  if (ImpIsDead)
    EOR->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SPREAD>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  //Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  
  //clear psr_gp4 flag
  buildMI(MBB, MBBI, OPEN8::CLP)
    .addImm(0x7);

  //retrive stack pointer
  buildMI(MBB, MBBI, OPEN8::RSP);

  //move r1r0 to SP
  buildMI(MBB, MBBI, OPEN8::MOVWRdRr)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(OPEN8::R1R0,RegState::Kill);

  /*
  unsigned Flags = MI.getFlags();
  unsigned OpLo = OPEN8::INRdA;
  unsigned OpHi = OPEN8::INRdA;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(0x3d)
    .setMIFlags(Flags);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(0x3e)
    .setMIFlags(Flags);
*/
  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SPWRITE>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  //Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  //unsigned Flags = MI.getFlags(); 
  
  //set psr_gp4 flag
  buildMI(MBB, MBBI, OPEN8::STP)
    .addImm(0x7);

  //move r7r6 to r1r
  buildMI(MBB, MBBI, OPEN8::MOVWRdRr)
    .addReg(OPEN8::R1R0) //add regstate ?
    .addReg(SrcReg, getKillRegState(SrcIsKill));

  //relocate stack pointer
  buildMI(MBB, MBBI, OPEN8::RSP);

  //clear psr_gp4 flag
  buildMI(MBB, MBBI, OPEN8::CLP)
    .addImm(0x7);
  /*
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  buildMI(MBB, MBBI, OPEN8::INRdA)
    .addReg(OPEN8::R0, RegState::Define)
    .addImm(SREG_ADDR)
    .setMIFlags(Flags);

  buildMI(MBB, MBBI, OPEN8::BCLRs).addImm(0x07).setMIFlags(Flags);

  buildMI(MBB, MBBI, OPEN8::OUTARr)
    .addImm(0x3e)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  buildMI(MBB, MBBI, OPEN8::OUTARr)
    .addImm(SREG_ADDR)
    .addReg(OPEN8::R0, RegState::Kill)
    .setMIFlags(Flags);

  buildMI(MBB, MBBI, OPEN8::OUTARr)
    .addImm(0x3d)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);
  */
  MI.eraseFromParent();
  return true;
}

/*template <>
bool OPEN8ExpandPseudo::expand<OPEN8::COMRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  //bool ImpIsDead = MI.getOperand(2).isDead();
  if(DstReg != OPEN8::R0 ){
    buildMI(MBB, MBBI, OPEN8::LDIRdk).addReg(OPEN8::R0).addImm(0xFF);
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(DstReg,getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::T0X).addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));
  } else {
    //TODO: value R1 will lost?
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R1);
    buildMI(MBB, MBBI, OPEN8::T0X).addReg(OPEN8::R1);
    buildMI(MBB, MBBI, OPEN8::LDIRdk).addReg(OPEN8::R0).addImm(0xFF);
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(OPEN8::R1,getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(OPEN8::R1);
  }
  MI.eraseFromParent();
  return true;
}*/

/*template <>
bool OPEN8ExpandPseudo::expand<OPEN8::NEGRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  //bool ImpIsDead = MI.getOperand(2).isDead();
  if(DstReg != OPEN8::R0 ){
    buildMI(MBB, MBBI, OPEN8::LDIRdk).addReg(OPEN8::R0).addImm(0x00);
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(DstReg,getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::T0X).addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));
  } else {
    //TODO: value R1 will lost?
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R1);
    buildMI(MBB, MBBI, OPEN8::T0X).addReg(OPEN8::R1);
    buildMI(MBB, MBBI, OPEN8::LDIRdk).addReg(OPEN8::R0).addImm(0x00);
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(OPEN8::R1,getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(OPEN8::R1);
  }
  MI.eraseFromParent();
  return true;
}*/

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::CPCRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  //bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  if(DstReg != OPEN8::R0  & SrcReg != OPEN8::R0){
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(SrcReg,getKillRegState(SrcIsKill));
  } else if (DstReg == OPEN8::R0 & SrcReg != OPEN8::R0){
    //push and pop r0
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R0);
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(DstReg,getKillRegState(SrcIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(OPEN8::R0);
  } else if (DstReg != OPEN8::R0 & SrcReg == OPEN8::R0){
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R0);
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R0);
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(DstReg);
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(DstReg,getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(DstReg);
  } else {
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(DstReg);
    buildMI(MBB, MBBI, OPEN8::SBC).addReg(DstReg,getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(DstReg);
  }
  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::CPRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  //bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  //carry will lost??
  buildMI(MBB, MBBI, OPEN8::CLP).addImm(0x1);
  if(DstReg != OPEN8::R0  & SrcReg != OPEN8::R0){
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::CMP).addReg(SrcReg,getKillRegState(SrcIsKill));
  } else if (DstReg == OPEN8::R0 & SrcReg != OPEN8::R0){
    buildMI(MBB, MBBI, OPEN8::CMP).addReg(DstReg,getKillRegState(SrcIsKill));
  } else if (DstReg != OPEN8::R0 & SrcReg == OPEN8::R0){
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R0);
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(DstReg);
    buildMI(MBB, MBBI, OPEN8::CMP).addReg(DstReg,getKillRegState(DstIsKill));
    //restart value to the registers
    buildMI(MBB, MBBI, OPEN8::PUSHRr).addReg(OPEN8::R0);
    buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg, getKillRegState(DstIsKill));
    buildMI(MBB, MBBI, OPEN8::POPRd).addReg(DstReg);
  } else {
    buildMI(MBB, MBBI, OPEN8::CMP).addReg(DstReg,getKillRegState(DstIsKill));
  }
  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::MOVRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  //bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  //bool ImpIsDead = MI.getOperand(3).isDead();

  if(SrcReg != OPEN8::R0 )
  buildMI(MBB, MBBI, OPEN8::TX0).addReg(SrcReg, getKillRegState(SrcIsKill));
  if(DstReg != OPEN8::R0 )
  buildMI(MBB, MBBI, OPEN8::T0X).addReg(DstReg, getKillRegState(DstIsKill));

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::MOVWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  //bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  //bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  unsigned Op0 = OPEN8::TX0;
  unsigned Op1 = OPEN8::T0X;

  //to avoid problems with movw r7r6 r1r0 or movw r1r0 r7r6
  if(DstReg == OPEN8::R1R0){
    //high 
    if(SrcHiReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op0).addReg(SrcHiReg, getKillRegState(SrcIsKill));
    if(DstHiReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op1).addReg(DstHiReg, getKillRegState(DstIsKill));
    //low part
    if(SrcLoReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op0).addReg(SrcLoReg, getKillRegState(SrcIsKill));
    if(DstLoReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op1).addReg(DstLoReg, getKillRegState(DstIsKill));
  }
  else{
    //low part
    if(SrcLoReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op0).addReg(SrcLoReg, getKillRegState(SrcIsKill));
    if(DstLoReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op1).addReg(DstLoReg, getKillRegState(DstIsKill));
    //high 
    if(SrcHiReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op0).addReg(SrcHiReg, getKillRegState(SrcIsKill));
    if(DstHiReg != OPEN8::R0 )
      buildMI(MBB, MBBI, Op1).addReg(DstHiReg, getKillRegState(DstIsKill));
  }

  MI.eraseFromParent();
  return true;
}

//TODO

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register sp = MI.getOperand(1).getReg();
  Register Reg = MI.getOperand(0).getReg();
  unsigned spIsKill = MI.getOperand(1).isKill();
  unsigned RegIsKill = MI.getOperand(0).isKill();

  unsigned Op0 = OPEN8::LDX;
  unsigned Op1 = OPEN8::T0X;

  /*auto MIBHI = */buildMI(MBB, MBBI, Op0)
    .addReg(sp, getKillRegState(spIsKill));
  if (Reg != OPEN8::R0)
  /*auto MIBLO = */buildMI(MBB, MBBI, Op1)
    .addReg(Reg, getKillRegState(RegIsKill));

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LDRdPi>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register sp = MI.getOperand(1).getReg();
  Register Reg = MI.getOperand(0).getReg();
  unsigned spIsKill = MI.getOperand(1).isKill();
  unsigned RegIsKill = MI.getOperand(0).isKill();

  unsigned Op0 = OPEN8::LDXinc;
  unsigned Op1 = OPEN8::T0X;

  /*auto MIBHI = */buildMI(MBB, MBBI, Op0)
    .addReg(sp, getKillRegState(spIsKill));
  if (Reg != OPEN8::R0)
  /*auto MIBLO = */buildMI(MBB, MBBI, Op1)
    .addReg(Reg, getKillRegState(RegIsKill));
  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::STRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register sp = MI.getOperand(0).getReg();
  Register Reg = MI.getOperand(1).getReg();
  unsigned spIsKill = MI.getOperand(0).isKill();
  unsigned RegIsKill = MI.getOperand(1).isKill();

  unsigned Op0 = OPEN8::TX0;
  unsigned Op1 = OPEN8::STX;

  if (Reg != OPEN8::R0)
  buildMI(MBB, MBBI, Op0)
    .addReg(Reg, getKillRegState(RegIsKill));

  buildMI(MBB, MBBI, Op1)
    .addReg(sp, getKillRegState(spIsKill));

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::STPiRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register sp = MI.getOperand(0).getReg();
  Register Reg = MI.getOperand(2).getReg();
  unsigned spIsKill = MI.getOperand(0).isKill();
  unsigned RegIsKill = MI.getOperand(2).isKill();

  unsigned Op0 = OPEN8::TX0;
  unsigned Op1 = OPEN8::STXinc;

  if (Reg != OPEN8::R0)
  buildMI(MBB, MBBI, Op0)
    .addReg(Reg, getKillRegState(RegIsKill));

  buildMI(MBB, MBBI, Op1)
    .addReg(sp, getKillRegState(spIsKill));

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::LSRRd>(Block &MBB, BlockIt MBBI){
  MachineInstr &MI = *MBBI;
  unsigned Op1, Op2;
  unsigned DstReg = MI.getOperand(0).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  //bool DstIsDead = MI.getOperand(0).isDead();

  Op1 = OPEN8::CLP;
  Op2 = OPEN8::RORRd;

  buildMI(MBB, MBBI, Op1).addImm(0x1); //clear caarry

  buildMI(MBB, MBBI, Op2)
    .addReg(DstReg, getKillRegState(DstIsKill));

  MI.eraseFromParent();
  return true;
}

//TODO: COMPLETE ASRRD
template <>
bool OPEN8ExpandPseudo::expand<OPEN8::ASRRd>(Block &MBB, BlockIt MBBI){
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  //bool DstIsDead = MI.getOperand(0).isDead();

  buildMI(MBB, MBBI, OPEN8::CLP).addImm(0x1);
  buildMI(MBB, MBBI, OPEN8::TX0).addReg(DstReg);
  buildMI(MBB, MBBI, OPEN8::BTT).addImm(0x7);
  buildMI(MBB, MBBI, OPEN8::BR1).addImm(0).addImm(3); //BRANCH ZERO
  buildMI(MBB, MBBI, OPEN8::STP).addImm(0x1);
  buildMI(MBB, MBBI, OPEN8::RORRd)
    .addReg(DstReg, getKillRegState(DstIsKill));

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::SWAPRd>(Block &MBB, BlockIt MBBI){
  MachineInstr &MI = *MBBI;
 
  Register DstReg = MI.getOperand(0).getReg();
  //bool DstIsKill = MI.getOperand(0).isKill();
  //bool DstIsDead = MI.getOperand(0).isDead();

  buildMI(MBB, MBBI, OPEN8::LDIRdk).addReg(OPEN8::R0).addImm(0x0F);
  buildMI(MBB, MBBI, OPEN8::AND).addReg(DstReg);
  buildMI(MBB, MBBI, OPEN8::T0X).addReg(OPEN8::R1);
  buildMI(MBB, MBBI, OPEN8::LDIRdk).addReg(OPEN8::R0).addImm(0xF0);
  buildMI(MBB, MBBI, OPEN8::AND).addReg(DstReg);
  buildMI(MBB, MBBI, OPEN8::ADD).addReg(OPEN8::R0);
  buildMI(MBB, MBBI, OPEN8::T0X).addReg(DstReg);
  //  .addReg(DstReg, getKillRegState(DstIsKill));

 /*buildMI(MBB, MBBI, OPEN8::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));
      */
  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::JMPZ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  buildMI(MBB, MBBI, OPEN8::BR0).addImm(0).addImm(5); //BRANCH
  //buildMI(MBB, MBBI, OPEN8::BRNZ).addImm(5); //BRANCH
  buildMI(MBB, MBBI, OPEN8::JMPk).addMBB(MI.getOperand(0).getMBB()); //JMP

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::JMPNZ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  buildMI(MBB, MBBI, OPEN8::BR1).addImm(0).addImm(5); //BRANCH
  //buildMI(MBB, MBBI, OPEN8::BRZ).addImm(5); //BRANCH
  buildMI(MBB, MBBI, OPEN8::JMPk).addMBB(MI.getOperand(0).getMBB()); //JMP

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::JMPLZ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  buildMI(MBB, MBBI, OPEN8::BR0).addImm(2).addImm(5); //BRANCH
  //buildMI(MBB, MBBI, OPEN8::BRC).addImm(5); //BRANCH
  buildMI(MBB, MBBI, OPEN8::JMPk).addMBB(MI.getOperand(0).getMBB()); //JMP

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::JMPGEZ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  buildMI(MBB, MBBI, OPEN8::BR1).addImm(2).addImm(5); //BRANCH
  //buildMI(MBB, MBBI, OPEN8::BRLZ).addImm(5); //BRANCH
  buildMI(MBB, MBBI, OPEN8::JMPk).addMBB(MI.getOperand(0).getMBB()); //JMP

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::JMPC>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  buildMI(MBB, MBBI, OPEN8::BR0).addImm(0).addImm(5); //BRANCH
  //buildMI(MBB, MBBI, OPEN8::BRNZ).addImm(5); //BRANCH
  buildMI(MBB, MBBI, OPEN8::JMPk).addMBB(MI.getOperand(0).getMBB()); //JMP

  MI.eraseFromParent();
  return true;
}

template <>
bool OPEN8ExpandPseudo::expand<OPEN8::JMPNC>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  buildMI(MBB, MBBI, OPEN8::BR1).addImm(1).addImm(5); //BRANCH
  //buildMI(MBB, MBBI, OPEN8::BRC).addImm(5); //BRANCH
  buildMI(MBB, MBBI, OPEN8::JMPk).addMBB(MI.getOperand(0).getMBB()); //JMP

  MI.eraseFromParent();
  return true;
}

bool OPEN8ExpandPseudo::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)               \
  case Op:                       \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    EXPAND(OPEN8::ADDWRdRr);
    EXPAND(OPEN8::ADCWRdRr);
    EXPAND(OPEN8::SUBWRdRr);
    EXPAND(OPEN8::SUBIWRdK);
    EXPAND(OPEN8::SBCWRdRr);
    EXPAND(OPEN8::ANDWRdRr);
    EXPAND(OPEN8::ORWRdRr);
    EXPAND(OPEN8::EORWRdRr);
    //EXPAND(OPEN8::COMWRd);
    //EXPAND(OPEN8::NEGWRd);
    EXPAND(OPEN8::CPWRdRr);
    EXPAND(OPEN8::CPCWRdRr);
    EXPAND(OPEN8::LDIWRdk);
    EXPAND(OPEN8::LDAWRdk);
    EXPAND(OPEN8::LDWRd);
    EXPAND(OPEN8::LDWRdPi);
    EXPAND(OPEN8::ADDRdRr);
    EXPAND(OPEN8::ADCRdRr);
    EXPAND(OPEN8::SBCIRdK);
    EXPAND(OPEN8::SUBIRdK);
    EXPAND(OPEN8::SUBRdRr);
    EXPAND(OPEN8::SBCRdRr);
    EXPAND(OPEN8::ANDRdRr);
    EXPAND(OPEN8::ORRdRr);
    EXPAND(OPEN8::EORRdRr);
    EXPAND(OPEN8::MULRdRr);
    EXPAND(OPEN8::MULSRdRr);
    //EXPAND(OPEN8::COMRd);
    //EXPAND(OPEN8::NEGRd);
    EXPAND(OPEN8::CPCRdRr);
    EXPAND(OPEN8::CPRdRr);
    EXPAND(OPEN8::MOVRdRr);
    EXPAND(OPEN8::MOVWRdRr);
    EXPAND(OPEN8::LDRd);
    EXPAND(OPEN8::LDRdPi);
    EXPAND(OPEN8::ASRRd);
    EXPAND(OPEN8::LSRRd);
    EXPAND(OPEN8::LDDRdQ);
    EXPAND(OPEN8::STDQRr);
    EXPAND(OPEN8::STRr);
    //EXPAND(OPEN8::RORRd);
    EXPAND(OPEN8::SWAPRd);
  //case OPEN8::LDDWRdYQ: //:FIXME: remove this once PR13375 gets fixed
    EXPAND(OPEN8::LDDWRdQ);
    EXPAND(OPEN8::STAWKRr);
    EXPAND(OPEN8::STWRr);
    EXPAND(OPEN8::STWPiRr);
    EXPAND(OPEN8::STDWQRr);
    EXPAND(OPEN8::PUSHWRr);
    EXPAND(OPEN8::POPWRd);
    EXPAND(OPEN8::ROLBRd);
    EXPAND(OPEN8::RORBRd);
    EXPAND(OPEN8::LSLWRd);
    EXPAND(OPEN8::LSLW4Rd);
    EXPAND(OPEN8::LSLW8Rd);
    //EXPAND(OPEN8::LSLW12Rd);
    EXPAND(OPEN8::LSRWRd);
    //EXPAND(OPEN8::LSRW4Rd);
    EXPAND(OPEN8::LSRW8Rd);
    //EXPAND(OPEN8::LSRW12Rd);
    EXPAND(OPEN8::RORWRd);
    EXPAND(OPEN8::ROLWRd);
    EXPAND(OPEN8::ASRWRd);
    EXPAND(OPEN8::ASRW8Rd);
    //EXPAND(OPEN8::LSLB7Rd);
    //EXPAND(OPEN8::LSRB7Rd);
    EXPAND(OPEN8::ASRB7Rd);
    EXPAND(OPEN8::SEXT);
    EXPAND(OPEN8::ZEXT);
    EXPAND(OPEN8::SPREAD);
    EXPAND(OPEN8::SPWRITE);
    EXPAND(OPEN8::JMPZ);
    EXPAND(OPEN8::JMPNZ);
    EXPAND(OPEN8::JMPLZ);
    EXPAND(OPEN8::JMPGEZ);
    EXPAND(OPEN8::JMPC);
    EXPAND(OPEN8::JMPNC);
  }
#undef EXPAND
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(OPEN8ExpandPseudo, "open8-expand-pseudo",
                OPEN8_EXPAND_PSEUDO_NAME, false, false)
namespace llvm {

FunctionPass *createOPEN8ExpandPseudoPass() { return new OPEN8ExpandPseudo(); }

} // end of namespace llvm
