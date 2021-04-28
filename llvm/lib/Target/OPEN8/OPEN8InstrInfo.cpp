//===-- OPEN8InstrInfo.cpp - OPEN8 Instruction Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the OPEN8 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "OPEN8InstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#include "OPEN8.h"
#include "OPEN8MachineFunctionInfo.h"
#include "OPEN8RegisterInfo.h"
#include "OPEN8TargetMachine.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "OPEN8GenInstrInfo.inc"

namespace llvm {

OPEN8InstrInfo::OPEN8InstrInfo()
    : OPEN8GenInstrInfo(OPEN8::ADJCALLSTACKDOWN, OPEN8::ADJCALLSTACKUP), RI() {}

void OPEN8InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc) const {
  //const OPEN8Subtarget &STI = MBB.getParent()->getSubtarget<OPEN8Subtarget>();
  //const OPEN8RegisterInfo &TRI = *STI.getRegisterInfo();
  unsigned Opc;

  // Not all OPEN8 devices support the 16-bit `MOVW` instruction.
  if (OPEN8::DREGSRegClass.contains(DestReg, SrcReg)) {
    
      BuildMI(MBB, MI, DL, get(OPEN8::MOVWRdRr), DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));

  } else {
    if (OPEN8::GPR8RegClass.contains(DestReg, SrcReg)) {
      Opc = OPEN8::MOVRdRr;
    } else if (SrcReg == OPEN8::SP && OPEN8::DREGSRegClass.contains(DestReg)) {
      Opc = OPEN8::SPREAD;
    } else if (DestReg == OPEN8::SP && OPEN8::DREGSRegClass.contains(SrcReg)) {
      Opc = OPEN8::SPWRITE;
    } else {
      llvm_unreachable("Impossible reg-to-reg copy");
    }

    BuildMI(MBB, MI, DL, get(Opc), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
  }
}

unsigned OPEN8InstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case OPEN8::LDDRdQ:
  case OPEN8::LDDWRdYQ: { //:FIXME: remove this once PR13375 gets fixed
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

unsigned OPEN8InstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case OPEN8::STDQRr:
  case OPEN8::STDWQRr: {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

void OPEN8InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       Register SrcReg, bool isKill,
                                       int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
  MachineFunction &MF = *MBB.getParent();
  OPEN8MachineFunctionInfo *AFI = MF.getInfo<OPEN8MachineFunctionInfo>();

  AFI->setHasSpills(true);

  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = OPEN8::STDQRr;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = OPEN8::STDWQRr;
  } else {
    llvm_unreachable("Cannot store this register into a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode))
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addReg(SrcReg, getKillRegState(isKill))
      .addMemOperand(MMO);
}

void OPEN8InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  MachineFunction &MF = *MBB.getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = OPEN8::LDDRdQ;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    // Opcode = OPEN8::LDDWRdQ;
    //:FIXME: remove this once PR13375 gets fixed
    Opcode = OPEN8::LDDWRdYQ;
  } else {
    llvm_unreachable("Cannot load this register from a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode), DestReg)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);
}

const MCInstrDesc &OPEN8InstrInfo::getBrCond(OPEN8CC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case OPEN8CC::COND_EQ:
    return get(OPEN8::BRZ);
  case OPEN8CC::COND_NE:
    return get(OPEN8::BRNZ);
  case OPEN8CC::COND_GE: //TODO: open8 dont support signed branch
    return get(OPEN8::BRGE);
  case OPEN8CC::COND_LT:
    return get(OPEN8::BRLT);
  case OPEN8CC::COND_SH:
    return get(OPEN8::BRNC);
  case OPEN8CC::COND_LO:
    return get(OPEN8::BRC);
  case OPEN8CC::COND_MI:
    return get(OPEN8::BRLZ);
  case OPEN8CC::COND_PL:
    return get(OPEN8::BRGEZ);
  }
}

OPEN8CC::CondCodes OPEN8InstrInfo::getCondFromBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:
    return OPEN8CC::COND_INVALID;
  case OPEN8::BRZ:
    return OPEN8CC::COND_EQ;
  case OPEN8::BRNZ:
    return OPEN8CC::COND_NE;
  case OPEN8::BRNC:
    return OPEN8CC::COND_SH;
  case OPEN8::BRC:
    return OPEN8CC::COND_LO;
  case OPEN8::BRLZ:
    return OPEN8CC::COND_MI;
  case OPEN8::BRGEZ:
    return OPEN8CC::COND_PL;
  case OPEN8::BRGE:
    return OPEN8CC::COND_GE;
  case OPEN8::BRLT:
    return OPEN8CC::COND_LT;
  }
}

OPEN8CC::CondCodes OPEN8InstrInfo::getOppositeCondition(OPEN8CC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Invalid condition!");
  case OPEN8CC::COND_EQ:
    return OPEN8CC::COND_NE;
  case OPEN8CC::COND_NE:
    return OPEN8CC::COND_EQ;
  case OPEN8CC::COND_SH:
    return OPEN8CC::COND_LO;
  case OPEN8CC::COND_LO:
    return OPEN8CC::COND_SH;
  case OPEN8CC::COND_GE:
    return OPEN8CC::COND_LT;
  case OPEN8CC::COND_LT:
    return OPEN8CC::COND_GE;
  case OPEN8CC::COND_MI:
    return OPEN8CC::COND_PL;
  case OPEN8CC::COND_PL:
    return OPEN8CC::COND_MI;
  }
}

bool OPEN8InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end();
  MachineBasicBlock::iterator UnCondBrIter = MBB.end();

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }

    // Working from the bottom, when we see a non-terminator
    // instruction, we're done.
    if (!isUnpredicatedTerminator(*I)) {
      break;
    }

    // A terminator that isn't a branch can't easily be handled
    // by this analysis.
    if (!I->getDesc().isBranch()) {
      return true;
    }

    // Handle unconditional branches.
    //:TODO: add here jmp
    if (I->getOpcode() == OPEN8::JMPk) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a JMP, delete them.
      while (std::next(I) != MBB.end()) {
        std::next(I)->eraseFromParent();
      }

      Cond.clear();
      FBB = 0;

      // Delete the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = 0;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = MBB.end();
        continue;
      }

      // TBB is used to indicate the unconditinal destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    OPEN8CC::CondCodes BranchCode = getCondFromBranchOpc(I->getOpcode());
    if (BranchCode == OPEN8CC::COND_INVALID) {
      return true; // Can't handle indirect branch.
    }

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(0).getMBB();
      if (AllowModify && UnCondBrIter != MBB.end() &&
          MBB.isLayoutSuccessor(TargetBB)) {
        // If we can modify the code and it ends in something like:
        //
        //     jCC L1
        //     jmp L2
        //   L1:
        //     ...
        //   L2:
        //
        // Then we can change this to:
        //
        //     jnCC L2
        //   L1:
        //     ...
        //   L2:
        //
        // Which is a bit more efficient.
        // We conditionally jump to the fall-through block.
        BranchCode = getOppositeCondition(BranchCode);
        unsigned JNCC = getBrCond(BranchCode).getOpcode();
        MachineBasicBlock::iterator OldInst = I;

        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(JNCC))
            .addMBB(UnCondBrIter->getOperand(0).getMBB());
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(OPEN8::JMPk))
            .addMBB(TargetBB);

        OldInst->eraseFromParent();
        UnCondBrIter->eraseFromParent();

        // Restart the analysis.
        UnCondBrIter = MBB.end();
        I = MBB.end();
        continue;
      }

      FBB = TBB;
      TBB = I->getOperand(0).getMBB();
      Cond.push_back(MachineOperand::CreateImm(BranchCode));
      continue;
    }

    // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination.
    assert(Cond.size() == 1);
    assert(TBB);

    // Only handle the case where all conditional branches branch to
    // the same destination.
    if (TBB != I->getOperand(0).getMBB()) {
      return true;
    }

    OPEN8CC::CondCodes OldBranchCode = (OPEN8CC::CondCodes)Cond[0].getImm();
    // If the conditions are the same, we can leave them alone.
    if (OldBranchCode == BranchCode) {
      continue;
    }

    return true;
  }

  return false;
}

unsigned OPEN8InstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL,
                                    int *BytesAdded) const {
  if (BytesAdded) *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "OPEN8 branch conditions have one component!");

  if (Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    auto &MI = *BuildMI(&MBB, DL, get(OPEN8::JMPk)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Conditional branch.
  unsigned Count = 0;
  OPEN8CC::CondCodes CC = (OPEN8CC::CondCodes)Cond[0].getImm();
  auto &CondMI = *BuildMI(&MBB, DL, getBrCond(CC)).addMBB(TBB);

  if (BytesAdded) *BytesAdded += getInstSizeInBytes(CondMI);
  ++Count;

  if (FBB) {
    // Two-way Conditional branch. Insert the second branch.
    auto &MI = *BuildMI(&MBB, DL, get(OPEN8::JMPk)).addMBB(FBB);
    if (BytesAdded) *BytesAdded += getInstSizeInBytes(MI);
    ++Count;
  }

  return Count;
}

unsigned OPEN8InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  if (BytesRemoved) *BytesRemoved = 0;

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }
    //:TODO: add here the missing jmp instructions once they are implemented
    // like jmp, {e}ijmp, and other cond branches, ...
    if (I->getOpcode() != OPEN8::JMPk &&
        getCondFromBranchOpc(I->getOpcode()) == OPEN8CC::COND_INVALID) {
      break;
    }

    // Remove the branch.
    if (BytesRemoved) *BytesRemoved += getInstSizeInBytes(*I);
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

bool OPEN8InstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid OPEN8 branch condition!");

  OPEN8CC::CondCodes CC = static_cast<OPEN8CC::CondCodes>(Cond[0].getImm());
  Cond[0].setImm(getOppositeCondition(CC));

  return false;
}

unsigned OPEN8InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
  // A regular instruction
  default: {
    const MCInstrDesc &Desc = get(Opcode);
    return Desc.getSize();
  }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const OPEN8TargetMachine &TM = static_cast<const OPEN8TargetMachine&>(MF.getTarget());
    const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();

    return TII.getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                                  *TM.getMCAsmInfo());
  }
  }
}

MachineBasicBlock *
OPEN8InstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("unexpected opcode!");
  case OPEN8::JMPk:
  case OPEN8::JSRk:
  case OPEN8::BRZ:
  case OPEN8::BRNZ:
  case OPEN8::BRC:
  case OPEN8::BRNC:
  case OPEN8::BRLZ:
  case OPEN8::BRGEZ:
  case OPEN8::BRGE:
  case OPEN8::BRLT:
    return MI.getOperand(0).getMBB();
  case OPEN8::BR1:
  case OPEN8::BR0:
    return MI.getOperand(1).getMBB();
  }
}

bool OPEN8InstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                         int64_t BrOffset) const {

  switch (BranchOp) {
  default:
    llvm_unreachable("unexpected opcode!");
  case OPEN8::JMPk:
  case OPEN8::JSRk:
    return isIntN(16, BrOffset);
  case OPEN8::BR1:
  case OPEN8::BR0:
  case OPEN8::BRZ:
  case OPEN8::BRNZ:
  case OPEN8::BRNC:
  case OPEN8::BRC:
  case OPEN8::BRLZ:
  case OPEN8::BRGEZ:
  case OPEN8::BRGE:
  case OPEN8::BRLT:
    return isIntN(8, BrOffset);
  }
}

unsigned OPEN8InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                            MachineBasicBlock &NewDestBB,
                                            const DebugLoc &DL,
                                            int64_t BrOffset,
                                            RegScavenger *RS) const {
    // This method inserts a *direct* branch (JMP), despite its name.
    // LLVM calls this method to fixup unconditional branches; it never calls
    // insertBranch or some hypothetical "insertDirectBranch".
    // See lib/CodeGen/RegisterRelaxation.cpp for details.
    // We end up here when a jump is too long for a RJMP instruction.
    auto &MI = *BuildMI(&MBB, DL, get(OPEN8::JMPk)).addMBB(&NewDestBB);

    return getInstSizeInBytes(MI);
}

} // end of namespace llvm

