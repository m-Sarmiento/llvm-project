//===-- OPEN8RelaxMemOperations.cpp - Relax out of range loads/stores -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass which relaxes out of range memory operations into
// equivalent operations which handle bigger addresses.
//
//===----------------------------------------------------------------------===//

#include "OPEN8.h"
#include "OPEN8InstrInfo.h"
#include "OPEN8TargetMachine.h"
#include "MCTargetDesc/OPEN8MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define OPEN8_RELAX_MEM_OPS_NAME "OPEN8 memory operation relaxation pass"

namespace {

class OPEN8RelaxMem : public MachineFunctionPass {
public:
  static char ID;

  OPEN8RelaxMem() : MachineFunctionPass(ID) {
    initializeOPEN8RelaxMemPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return OPEN8_RELAX_MEM_OPS_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const TargetInstrInfo *TII;

  template <unsigned OP> bool relax(Block &MBB, BlockIt MBBI);

  bool runOnBasicBlock(Block &MBB);
  bool runOnInstruction(Block &MBB, BlockIt MBBI);

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode));
  }
};

char OPEN8RelaxMem::ID = 0;

bool OPEN8RelaxMem::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const OPEN8Subtarget &STI = MF.getSubtarget<OPEN8Subtarget>();
  TII = STI.getInstrInfo();

  for (Block &MBB : MF) {
    bool BlockModified = runOnBasicBlock(MBB);
    Modified |= BlockModified;
  }

  return Modified;
}

bool OPEN8RelaxMem::runOnBasicBlock(Block &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);
    Modified |= runOnInstruction(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

template <>
bool OPEN8RelaxMem::relax<OPEN8::STDWPtrQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  MachineOperand &Ptr = MI.getOperand(0);
  MachineOperand &Src = MI.getOperand(2);
  int64_t Imm = MI.getOperand(1).getImm();

  // We can definitely optimise this better.
  if (Imm > 63) {
    // Push the previous state of the pointer register.
    // This instruction must preserve the value.
    buildMI(MBB, MBBI, OPEN8::PUSHWRr)
      .addReg(Ptr.getReg());

    // Add the immediate to the pointer register.
    buildMI(MBB, MBBI, OPEN8::SBCIWRdK)
      .addReg(Ptr.getReg(), RegState::Define)
      .addReg(Ptr.getReg())
      .addImm(-Imm);

    // Store the value in the source register to the address
    // pointed to by the pointer register.
    buildMI(MBB, MBBI, OPEN8::STWPtrRr)
      .addReg(Ptr.getReg())
      .addReg(Src.getReg(), getKillRegState(Src.isKill()));

    // Pop the original state of the pointer register.
    buildMI(MBB, MBBI, OPEN8::POPWRd)
      .addDef(Ptr.getReg(), getKillRegState(Ptr.isKill()));

    MI.removeFromParent();
  }

  return false;
}

bool OPEN8RelaxMem::runOnInstruction(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define RELAX(Op)                \
  case Op:                       \
    return relax<Op>(MBB, MI)

  switch (Opcode) {
    RELAX(OPEN8::STDWPtrQRr);
  }
#undef RELAX
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(OPEN8RelaxMem, "open8-relax-mem",
                OPEN8_RELAX_MEM_OPS_NAME, false, false)

namespace llvm {

FunctionPass *createOPEN8RelaxMemPass() { return new OPEN8RelaxMem(); }

} // end of namespace llvm
