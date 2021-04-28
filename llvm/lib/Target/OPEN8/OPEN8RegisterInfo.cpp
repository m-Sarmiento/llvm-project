//===-- OPEN8RegisterInfo.cpp - OPEN8 Register Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the OPEN8 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "OPEN8RegisterInfo.h"

#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

#include "OPEN8.h"
#include "OPEN8InstrInfo.h"
#include "OPEN8MachineFunctionInfo.h"
#include "OPEN8TargetMachine.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"

#define GET_REGINFO_TARGET_DESC
#include "OPEN8GenRegisterInfo.inc"

namespace llvm {

OPEN8RegisterInfo::OPEN8RegisterInfo() : OPEN8GenRegisterInfo(0) {}

const uint16_t *
OPEN8RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const OPEN8MachineFunctionInfo *AFI = MF->getInfo<OPEN8MachineFunctionInfo>();

  return AFI->isInterruptOrSignalHandler()
              ? CSR_Interrupts_SaveList
              : CSR_Normal_SaveList;
}

const uint32_t *
OPEN8RegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                      CallingConv::ID CC) const {
  const OPEN8MachineFunctionInfo *AFI = MF.getInfo<OPEN8MachineFunctionInfo>();

  return AFI->isInterruptOrSignalHandler()
              ? CSR_Interrupts_RegMask
              : CSR_Normal_RegMask;
}

BitVector OPEN8RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // Reserve the intermediate result registers r1 and r2
  // The result of instructions like 'mul' is always stored here.
  Reserved.set(OPEN8::R0);
  Reserved.set(OPEN8::R1);
  Reserved.set(OPEN8::R1R0);

  //  Reserve the stack pointer.
  Reserved.set(OPEN8::SPL);
  Reserved.set(OPEN8::SPH);
  Reserved.set(OPEN8::SP);

  // We tenatively reserve the frame pointer register r29:r28 because the
  // function may require one, but we cannot tell until register allocation
  // is complete, which can be too late.
  //
  // Instead we just unconditionally reserve the Y register.
  //
  // TODO: Write a pass to enumerate functions which reserved the Y register
  //       but didn't end up needing a frame pointer. In these, we can
  //       convert one or two of the spills inside to use the Y register.
  Reserved.set(OPEN8::R6);
  Reserved.set(OPEN8::R7);
  Reserved.set(OPEN8::R7R6);

  return Reserved;
}

const TargetRegisterClass *
OPEN8RegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                           const MachineFunction &MF) const {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    return &OPEN8::DREGSRegClass;
  }

  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    return &OPEN8::GPR8RegClass;
  }

  llvm_unreachable("Invalid register size");
}

/// Fold a frame offset shared between two add instructions into a single one.
static void foldFrameOffset(MachineBasicBlock::iterator &II, int &Offset,
                            Register DstReg) {
  return;
  
  /*MachineInstr &MI = *II;
  int Opcode = MI.getOpcode();

  // Don't bother trying if the next instruction is not an add or a sub.
  //if ((Opcode != OPEN8::SUBIWRdK) && (Opcode != OPEN8::ADIWRdK)) {
    return;
  //}

  // Check that DstReg matches with next instruction, otherwise the instruction
  // is not related to stack address manipulation.
  if (DstReg != MI.getOperand(0).getReg()) {
    return;
  }

  // Add the offset in the next instruction to our offset.
  switch (Opcode) {
  case OPEN8::SUBIWRdK:
    Offset += -MI.getOperand(2).getImm();
    break;
  }

  // Finally remove the instruction.
  II++;
  MI.eraseFromParent();*/
}

void OPEN8RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected SPAdj value");

  MachineInstr &MI = *II;
  DebugLoc dl = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction &MF = *MBB.getParent();
  const OPEN8TargetMachine &TM = (const OPEN8TargetMachine &)MF.getTarget();
  const TargetInstrInfo &TII = *TM.getSubtargetImpl()->getInstrInfo();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFI = TM.getSubtargetImpl()->getFrameLowering();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  int Offset = MFI.getObjectOffset(FrameIndex);

  // Add one to the offset because SP points to an empty slot.//
  Offset += MFI.getStackSize() - TFI->getOffsetOfLocalArea()+1;
  // Fold incoming offset.
  Offset += MI.getOperand(FIOperandNum + 1).getImm();

  // This is actually "load effective address" of the stack slot
  // instruction. We have only two-address instructions, thus we need to
  // expand it into move + add.
  if (MI.getOpcode() == OPEN8::FRMIDX) {
    MI.setDesc(TII.get(OPEN8::MOVWRdRr));
    MI.getOperand(FIOperandNum).ChangeToRegister(OPEN8::R7R6, false);
    MI.RemoveOperand(2);

    assert(Offset > 0 && "Invalid offset");

    Register DstReg = MI.getOperand(0).getReg();
    assert(DstReg != OPEN8::R7R6 && "Dest reg cannot be the frame pointer");

    II++; // Skip over the FRMIDX (and now MOVW) instruction.

    // Generally, to load a frame address two add instructions are emitted that
    // could get folded into a single one:
    //  movw    r31:r30, r29:r28
    //  adiw    r31:r30, 29
    //  adiw    r31:r30, 16
    // to:
    //  movw    r31:r30, r29:r28
    //  adiw    r31:r30, 45
    if (II != MBB.end())
      foldFrameOffset(II, Offset, DstReg);

    /*Offset = -Offset;
    MachineInstr *New = BuildMI(MBB, II, dl, TII.get(OPEN8::SUBIWRdK), DstReg)
                            .addReg(DstReg, RegState::Kill)
                            .addImm(Offset);
    New->getOperand(3).setIsDead();*/
    MachineInstr *New = BuildMI(MBB, II, dl, TII.get(OPEN8::ADIWRdK), DstReg)
                            .addReg(DstReg, RegState::Kill)
                            .addImm(Offset);
    New->getOperand(3).setIsDead();
    return;
  }

  // If the offset is too big we have to adjust and restore the frame pointer
  // to materialize a valid load/store with displacement.
  //:TODO: consider using only one adiw/sbiw chain for more than one frame index
  //// Always case For huge offsets where adiw/sbiw cannot be used use a pair of subi/sbci.
  if (Offset > 126) {
    assert(true && "psr posible lost");
    unsigned SubOpc = OPEN8::SUBIWRdK;
    unsigned AddOpc = OPEN8::ADIWRdK;
    int AddOffset = (Offset - 127 + 1);
    // It is possible that the spiller places this frame instruction in between
    // a compare and branch, invalidating the contents of SREG set by the
    // compare instruction because of the add/sub pairs. Conservatively save and
    // restore SREG before and after each add/sub pair.
    // OPEN8 dont support save or restore PSR, maybe will lost it
    //BuildMI(MBB, II, dl, TII.get(OPEN8::INRdA), OPEN8::R0).addImm(0x3f);
    MachineInstr *New = BuildMI(MBB, II, dl, TII.get(AddOpc), OPEN8::R7R6)
                            .addReg(OPEN8::R7R6, RegState::Kill)
                            .addImm(AddOffset);
    New->getOperand(3).setIsDead();
    // Restore SREG.
    //BuildMI(MBB, std::next(II), dl, TII.get(OPEN8::OUTARr))
    //    .addImm(0x3f)
    //    .addReg(OPEN8::R0, RegState::Kill);
    // No need to set SREG as dead here otherwise if the next instruction is a
    // cond branch it will be using a dead register.
    BuildMI(MBB, std::next(II), dl, TII.get(SubOpc), OPEN8::R7R6)
        .addReg(OPEN8::R7R6, RegState::Kill)
        .addImm(Offset - 127 + 1);
    Offset = 126;
  }

  MI.getOperand(FIOperandNum).ChangeToRegister(OPEN8::R7R6, false);
  assert(isUInt<7>(Offset) && "Offset is out of range");
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
}

Register OPEN8RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  if (TFI->hasFP(MF)) {
    // The Y pointer register
    return OPEN8::R6;
  }

  return OPEN8::SP;
}

const TargetRegisterClass *
OPEN8RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind) const {
  // FIXME: Currently we're using open8-gcc as reference, so we restrict
  // ptrs to Y and Z regs. Though open8-gcc has buggy implementation
  // of memory constraint, so we can fix it and bit open8-gcc here ;-)
  return &OPEN8::PTRDISPREGSRegClass;
}

void OPEN8RegisterInfo::splitReg(Register Reg, Register &LoReg,
                               Register &HiReg) const {
  assert(OPEN8::DREGSRegClass.contains(Reg) && "can only split 16-bit registers");

  LoReg = getSubReg(Reg, OPEN8::sub_lo);
  HiReg = getSubReg(Reg, OPEN8::sub_hi);
}

bool OPEN8RegisterInfo::shouldCoalesce(MachineInstr *MI,
                                     const TargetRegisterClass *SrcRC,
                                     unsigned SubReg,
                                     const TargetRegisterClass *DstRC,
                                     unsigned DstSubReg,
                                     const TargetRegisterClass *NewRC,
                                     LiveIntervals &LIS) const {
  if(this->getRegClass(OPEN8::PTRDISPREGSRegClassID)->hasSubClassEq(NewRC)) {
    return false;
  }

  return TargetRegisterInfo::shouldCoalesce(MI, SrcRC, SubReg, DstRC, DstSubReg, NewRC, LIS);
}

} // end of namespace llvm
