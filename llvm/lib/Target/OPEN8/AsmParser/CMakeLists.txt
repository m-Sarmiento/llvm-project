add_llvm_component_library(LLVMOPEN8AsmParser
  OPEN8AsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  OPEN8Desc
  OPEN8Info
  Support

  ADD_TO_COMPONENT
  OPEN8
)
