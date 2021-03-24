add_llvm_component_group(OPEN8)

set(LLVM_TARGET_DEFINITIONS OPEN8.td)

tablegen(LLVM OPEN8GenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM OPEN8GenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM OPEN8GenCallingConv.inc -gen-callingconv)
tablegen(LLVM OPEN8GenDAGISel.inc -gen-dag-isel)
tablegen(LLVM OPEN8GenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM OPEN8GenInstrInfo.inc -gen-instr-info)
tablegen(LLVM OPEN8GenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM OPEN8GenRegisterInfo.inc -gen-register-info)
tablegen(LLVM OPEN8GenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(OPEN8CommonTableGen)

add_llvm_target(OPEN8CodeGen
  OPEN8AsmPrinter.cpp
  OPEN8ExpandPseudoInsts.cpp
  OPEN8FrameLowering.cpp
  OPEN8InstrInfo.cpp
  OPEN8ISelDAGToDAG.cpp
  OPEN8ISelLowering.cpp
  OPEN8MCInstLower.cpp
  OPEN8RelaxMemOperations.cpp
  OPEN8RegisterInfo.cpp
  OPEN8Subtarget.cpp
  OPEN8TargetMachine.cpp
  OPEN8TargetObjectFile.cpp

  DEPENDS
  intrinsics_gen

  LINK_COMPONENTS
  AsmPrinter
  CodeGen
  Core
  MC
  OPEN8Desc
  OPEN8Info
  SelectionDAG
  Support
  Target

  ADD_TO_COMPONENT
  OPEN8
)

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
