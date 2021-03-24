add_llvm_component_library(LLVMOPEN8Disassembler
  OPEN8Disassembler.cpp

  LINK_COMPONENTS
  MCDisassembler
  OPEN8Info
  Support

  ADD_TO_COMPONENT
  OPEN8
)

