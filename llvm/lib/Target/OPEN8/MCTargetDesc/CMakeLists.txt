add_llvm_component_library(LLVMOPEN8Desc
  OPEN8AsmBackend.cpp
  OPEN8ELFObjectWriter.cpp
  OPEN8ELFStreamer.cpp
  OPEN8InstPrinter.cpp
  OPEN8MCAsmInfo.cpp
  OPEN8MCCodeEmitter.cpp
  OPEN8MCELFStreamer.cpp
  OPEN8MCExpr.cpp
  OPEN8MCTargetDesc.cpp
  OPEN8TargetStreamer.cpp

  LINK_COMPONENTS
  MC
  OPEN8Info
  Support

  ADD_TO_COMPONENT
  OPEN8
)

add_dependencies(LLVMOPEN8Desc OPEN8CommonTableGen)

