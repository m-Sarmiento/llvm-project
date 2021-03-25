//===-- OPEN8FrameLowering.cpp - OPEN8 Frame Information ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the OPEN8 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "OPEN8FrameLowering.h"

#include "OPEN8.h"
#include "OPEN8InstrInfo.h"
#include "OPEN8MachineFunctionInfo.h"
#include "OPEN8TargetMachine.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"

#include <vector>

namespace llvm {

OPEN8FrameLowering::OPEN8FrameLowering()
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, Align(1), -2) {}

bool OPEN8FrameLowering::canSimplifyCallFramePseudos(
    const MachineFunction &MF) const {
  // Always simplify call frame pseudo instructions, even when
  // hasReservedCallFrame is false.
  return true;
}

bool OPEN8FrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  // Reserve call frame memory in function prologue under the following
  // conditions:
  // - Y pointer is reserved to be the frame pointer.
  // - The function does not contain variable sized objects.

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return hasFP(MF) && !MFI.hasVarSizedObjects();
}

void OPEN8FrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = (MBBI != MBB.end()) ? MBBI->getDebugLoc() : DebugLoc();
  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  const OPEN8InstrInfo &TII = *STI.getInstrInfo();
  const OPEN8MachineFunctionInfo *AFI = MF.getInfo<OPEN8MachineFunctionInfo>();
  bool HasFP = hasFP(MF);

  // Interrupt handlers re-enable interrupts in function entry.
  if (AFI->isInterruptHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::BSETs))
        .addImm(0x07)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Emit special prologue code to save R1, R0 and SREG in interrupt/signal
  // handlers before saving any other registers.
  if (AFI->isInterruptOrSignalHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::PUSHWRr))
        .addReg(OPEN8::R1R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);

    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::INRdA), OPEN8::R0)
        .addImm(0x3f)
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::PUSHRr))
        .addReg(OPEN8::R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::EORRdRr))
        .addReg(OPEN8::R0, RegState::Define)
        .addReg(OPEN8::R0, RegState::Kill)
        .addReg(OPEN8::R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Early exit if the frame pointer is not needed in this function.
  if (!HasFP) {
    return;
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();

  // Skip the callee-saved push instructions.
  while (
      (MBBI != MBB.end()) && MBBI->getFlag(MachineInstr::FrameSetup) &&
      (MBBI->getOpcode() == OPEN8::PUSHRr || MBBI->getOpcode() == OPEN8::PUSHWRr)) {
    ++MBBI;
  }

  // Update Y with the new base value.
  BuildMI(MBB, MBBI, DL, TII.get(OPEN8::SPREAD), OPEN8::R7R6)
      .addReg(OPEN8::SP)
      .setMIFlag(MachineInstr::FrameSetup);

  // Mark the FramePtr as live-in in every block except the entry.
  for (MachineFunction::iterator I = std::next(MF.begin()), E = MF.end();
       I != E; ++I) {
    I->addLiveIn(OPEN8::R7R6);
  }

  if (!FrameSize) {
    return;
  }

  // Reserve the necessary frame memory by doing FP -= <size>.
  unsigned Opcode = (isUInt<6>(FrameSize)) ? OPEN8::SBIWRdK : OPEN8::SUBIWRdK;

  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Opcode), OPEN8::R7R6)
                         .addReg(OPEN8::R7R6, RegState::Kill)
                         .addImm(FrameSize)
                         .setMIFlag(MachineInstr::FrameSetup);
  // The SREG implicit def is dead.
  MI->getOperand(3).setIsDead();

  // Write back R7R6 to SP and temporarily disable interrupts.
  BuildMI(MBB, MBBI, DL, TII.get(OPEN8::SPWRITE), OPEN8::SP)
      .addReg(OPEN8::R7R6)
      .setMIFlag(MachineInstr::FrameSetup);
}

static void restoreStatusRegister(MachineFunction &MF, MachineBasicBlock &MBB) {
  const OPEN8MachineFunctionInfo *AFI = MF.getInfo<OPEN8MachineFunctionInfo>();

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();

  DebugLoc DL = MBBI->getDebugLoc();
  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  const OPEN8InstrInfo &TII = *STI.getInstrInfo();

  // Emit special epilogue code to restore R1, R0 and SREG in interrupt/signal
  // handlers at the very end of the function, just before reti.
  if (AFI->isInterruptOrSignalHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::POPRd), OPEN8::R0);
    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::OUTARr))
        .addImm(0x3f)
        .addReg(OPEN8::R0, RegState::Kill);
    BuildMI(MBB, MBBI, DL, TII.get(OPEN8::POPWRd), OPEN8::R1R0);
  }
}

void OPEN8FrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const OPEN8MachineFunctionInfo *AFI = MF.getInfo<OPEN8MachineFunctionInfo>();

  // Early exit if the frame pointer is not needed in this function except for
  // signal/interrupt handlers where special code generation is required.
  if (!hasFP(MF) && !AFI->isInterruptOrSignalHandler()) {
    return;
  }

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  assert(MBBI->getDesc().isReturn() &&
         "Can only insert epilog into returning blocks");

  DebugLoc DL = MBBI->getDebugLoc();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();
  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  const OPEN8InstrInfo &TII = *STI.getInstrInfo();

  // Early exit if there is no need to restore the frame pointer.
  if (!FrameSize) {
    restoreStatusRegister(MF, MBB);
    return;
  }

  // Skip the callee-saved pop instructions.
  while (MBBI != MBB.begin()) {
    MachineBasicBlock::iterator PI = std::prev(MBBI);
    int Opc = PI->getOpcode();

    if (Opc != OPEN8::POPRd && Opc != OPEN8::POPWRd && !PI->isTerminator()) {
      break;
    }

    --MBBI;
  }

  unsigned Opcode;

  // Select the optimal opcode depending on how big it is.
  if (isUInt<6>(FrameSize)) {
    Opcode = OPEN8::ADIWRdK;
  } else {
    Opcode = OPEN8::SUBIWRdK;
    FrameSize = -FrameSize;
  }

  // Restore the frame pointer by doing FP += <size>.
  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Opcode), OPEN8::R7R6)
                         .addReg(OPEN8::R7R6, RegState::Kill)
                         .addImm(FrameSize);
  // The SREG implicit def is dead.
  MI->getOperand(3).setIsDead();

  // Write back R7R6 to SP and temporarily disable interrupts.
  BuildMI(MBB, MBBI, DL, TII.get(OPEN8::SPWRITE), OPEN8::SP)
      .addReg(OPEN8::R7R6, RegState::Kill);

  restoreStatusRegister(MF, MBB);
}

// Return true if the specified function should have a dedicated frame
// pointer register. This is true if the function meets any of the following
// conditions:
//  - a register has been spilled
//  - has allocas
//  - input arguments are passed using the stack
//
// Notice that strictly this is not a frame pointer because it contains SP after
// frame allocation instead of having the original SP in function entry.
bool OPEN8FrameLowering::hasFP(const MachineFunction &MF) const {
  const OPEN8MachineFunctionInfo *FuncInfo = MF.getInfo<OPEN8MachineFunctionInfo>();

  return (FuncInfo->getHasSpills() || FuncInfo->getHasAllocas() ||
          FuncInfo->getHasStackArgs());
}

bool OPEN8FrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  unsigned CalleeFrameSize = 0;
  DebugLoc DL = MBB.findDebugLoc(MI);
  MachineFunction &MF = *MBB.getParent();
  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  OPEN8MachineFunctionInfo *OPEN8FI = MF.getInfo<OPEN8MachineFunctionInfo>();

  for (unsigned i = CSI.size(); i != 0; --i) {
    unsigned Reg = CSI[i - 1].getReg();
    bool IsNotLiveIn = !MBB.isLiveIn(Reg);

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 8 &&
           "Invalid register size");

    // Add the callee-saved register as live-in only if it is not already a
    // live-in register, this usually happens with arguments that are passed
    // through callee-saved registers.
    if (IsNotLiveIn) {
      MBB.addLiveIn(Reg);
    }

    // Do not kill the register when it is an input argument.
    BuildMI(MBB, MI, DL, TII.get(OPEN8::PUSHRr))
        .addReg(Reg, getKillRegState(IsNotLiveIn))
        .setMIFlag(MachineInstr::FrameSetup);
    ++CalleeFrameSize;
  }

  OPEN8FI->setCalleeSavedFrameSize(CalleeFrameSize);

  return true;
}

bool OPEN8FrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  DebugLoc DL = MBB.findDebugLoc(MI);
  const MachineFunction &MF = *MBB.getParent();
  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  for (const CalleeSavedInfo &CCSI : CSI) {
    unsigned Reg = CCSI.getReg();

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 8 &&
           "Invalid register size");

    BuildMI(MBB, MI, DL, TII.get(OPEN8::POPRd), Reg);
  }

  return true;
}

/// Replace pseudo store instructions that pass arguments through the stack with
/// real instructions.
static void fixStackStores(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI,
                           const TargetInstrInfo &TII, Register FP) {
  // Iterate through the BB until we hit a call instruction or we reach the end.
  for (auto I = MI, E = MBB.end(); I != E && !I->isCall();) {
    MachineBasicBlock::iterator NextMI = std::next(I);
    MachineInstr &MI = *I;
    unsigned Opcode = I->getOpcode();

    // Only care of pseudo store instructions where SP is the base pointer.
    if (Opcode != OPEN8::STDSPQRr && Opcode != OPEN8::STDWSPQRr) {
      I = NextMI;
      continue;
    }

    assert(MI.getOperand(0).getReg() == OPEN8::SP &&
           "Invalid register, should be SP!");

    // Replace this instruction with a regular store. Use Y as the base
    // pointer since it is guaranteed to contain a copy of SP.
    unsigned STOpc =
        (Opcode == OPEN8::STDWSPQRr) ? OPEN8::STDWPtrQRr : OPEN8::STDPtrQRr;

    MI.setDesc(TII.get(STOpc));
    MI.getOperand(0).setReg(FP);

    I = NextMI;
  }
}

MachineBasicBlock::iterator OPEN8FrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  const OPEN8InstrInfo &TII = *STI.getInstrInfo();

  // There is nothing to insert when the call frame memory is allocated during
  // function entry. Delete the call frame pseudo and replace all pseudo stores
  // with real store instructions.
  if (hasReservedCallFrame(MF)) {
    fixStackStores(MBB, MI, TII, OPEN8::R7R6);
    return MBB.erase(MI);
  }

  DebugLoc DL = MI->getDebugLoc();
  unsigned int Opcode = MI->getOpcode();
  int Amount = TII.getFrameSize(*MI);

  // ADJCALLSTACKUP and ADJCALLSTACKDOWN are converted to adiw/subi
  // instructions to read and write the stack pointer in I/O space.
  if (Amount != 0) {
    assert(getStackAlign() == Align(1) && "Unsupported stack alignment");

    if (Opcode == TII.getCallFrameSetupOpcode()) {
      // Update the stack pointer.
      // In many cases this can be done far more efficiently by pushing the
      // relevant values directly to the stack. However, doing that correctly
      // (in the right order, possibly skipping some empty space for undef
      // values, etc) is tricky and thus left to be optimized in the future.
      BuildMI(MBB, MI, DL, TII.get(OPEN8::SPREAD), OPEN8::R7R6).addReg(OPEN8::SP);

      MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(OPEN8::SUBIWRdK), OPEN8::R7R6)
                              .addReg(OPEN8::R7R6, RegState::Kill)
                              .addImm(Amount);
      New->getOperand(3).setIsDead();

      BuildMI(MBB, MI, DL, TII.get(OPEN8::SPWRITE), OPEN8::SP)
          .addReg(OPEN8::R7R6);

      // Make sure the remaining stack stores are converted to real store
      // instructions.
      fixStackStores(MBB, MI, TII, OPEN8::R7R6);
    } else {
      assert(Opcode == TII.getCallFrameDestroyOpcode());

      // Note that small stack changes could be implemented more efficiently
      // with a few pop instructions instead of the 8-9 instructions now
      // required.

      // Select the best opcode to adjust SP based on the offset size.
      unsigned addOpcode;
      if (isUInt<6>(Amount)) {
        addOpcode = OPEN8::ADIWRdK;
      } else {
        addOpcode = OPEN8::SUBIWRdK;
        Amount = -Amount;
      }

      // Build the instruction sequence.
      BuildMI(MBB, MI, DL, TII.get(OPEN8::SPREAD), OPEN8::R7R6).addReg(OPEN8::SP);

      MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(addOpcode), OPEN8::R7R6)
                              .addReg(OPEN8::R7R6, RegState::Kill)
                              .addImm(Amount);
      New->getOperand(3).setIsDead();

      BuildMI(MBB, MI, DL, TII.get(OPEN8::SPWRITE), OPEN8::SP)
          .addReg(OPEN8::R7R6, RegState::Kill);
    }
  }

  return MBB.erase(MI);
}

void OPEN8FrameLowering::determineCalleeSaves(MachineFunction &MF,
                                            BitVector &SavedRegs,
                                            RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  // If we have a frame pointer, the Y register needs to be saved as well.
  if (hasFP(MF)) {
    SavedRegs.set(OPEN8::R7);
    SavedRegs.set(OPEN8::R6);
  }
}
/// The frame analyzer pass.
///
/// Scans the function for allocas and used arguments
/// that are passed through the stack.
struct OPEN8FrameAnalyzer : public MachineFunctionPass {
  static char ID;
  OPEN8FrameAnalyzer() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override {
    const MachineFrameInfo &MFI = MF.getFrameInfo();
    OPEN8MachineFunctionInfo *FuncInfo = MF.getInfo<OPEN8MachineFunctionInfo>();

    // If there are no fixed frame indexes during this stage it means there
    // are allocas present in the function.
    if (MFI.getNumObjects() != MFI.getNumFixedObjects()) {
      // Check for the type of allocas present in the function. We only care
      // about fixed size allocas so do not give false positives if only
      // variable sized allocas are present.
      for (unsigned i = 0, e = MFI.getObjectIndexEnd(); i != e; ++i) {
        // Variable sized objects have size 0.
        if (MFI.getObjectSize(i)) {
          FuncInfo->setHasAllocas(true);
          break;
        }
      }
    }

    // If there are fixed frame indexes present, scan the function to see if
    // they are really being used.
    if (MFI.getNumFixedObjects() == 0) {
      return false;
    }

    // Ok fixed frame indexes present, now scan the function to see if they
    // are really being used, otherwise we can ignore them.
    for (const MachineBasicBlock &BB : MF) {
      for (const MachineInstr &MI : BB) {
        int Opcode = MI.getOpcode();

        if ((Opcode != OPEN8::LDDRdPtrQ) && (Opcode != OPEN8::LDDWRdPtrQ) &&
            (Opcode != OPEN8::STDPtrQRr) && (Opcode != OPEN8::STDWPtrQRr)) {
          continue;
        }

        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isFI()) {
            continue;
          }

          if (MFI.isFixedObjectIndex(MO.getIndex())) {
            FuncInfo->setHasStackArgs(true);
            return false;
          }
        }
      }
    }

    return false;
  }

  StringRef getPassName() const override { return "OPEN8 Frame Analyzer"; }
};

char OPEN8FrameAnalyzer::ID = 0;

/// Creates instance of the frame analyzer pass.
FunctionPass *createOPEN8FrameAnalyzerPass() { return new OPEN8FrameAnalyzer(); }

/// Create the Dynalloca Stack Pointer Save/Restore pass.
/// Insert a copy of SP before allocating the dynamic stack memory and restore
/// it in function exit to restore the original SP state. This avoids the need
/// of reserving a register pair for a frame pointer.
struct OPEN8DynAllocaSR : public MachineFunctionPass {
  static char ID;
  OPEN8DynAllocaSR() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override {
    // Early exit when there are no variable sized objects in the function.
    if (!MF.getFrameInfo().hasVarSizedObjects()) {
      return false;
    }

    const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();
    MachineBasicBlock &EntryMBB = MF.front();
    MachineBasicBlock::iterator MBBI = EntryMBB.begin();
    DebugLoc DL = EntryMBB.findDebugLoc(MBBI);

    Register SPCopy =
        MF.getRegInfo().createVirtualRegister(&OPEN8::DREGSRegClass);

    // Create a copy of SP in function entry before any dynallocas are
    // inserted.
    BuildMI(EntryMBB, MBBI, DL, TII.get(OPEN8::COPY), SPCopy).addReg(OPEN8::SP);

    // Restore SP in all exit basic blocks.
    for (MachineBasicBlock &MBB : MF) {
      // If last instruction is a return instruction, add a restore copy.
      if (!MBB.empty() && MBB.back().isReturn()) {
        MBBI = MBB.getLastNonDebugInstr();
        DL = MBBI->getDebugLoc();
        BuildMI(MBB, MBBI, DL, TII.get(OPEN8::COPY), OPEN8::SP)
            .addReg(SPCopy, RegState::Kill);
      }
    }

    return true;
  }

  StringRef getPassName() const override {
    return "OPEN8 dynalloca stack pointer save/restore";
  }
};

char OPEN8DynAllocaSR::ID = 0;

/// createOPEN8DynAllocaSRPass - returns an instance of the dynalloca stack
/// pointer save/restore pass.
FunctionPass *createOPEN8DynAllocaSRPass() { return new OPEN8DynAllocaSR(); }

} // end of namespace llvm

