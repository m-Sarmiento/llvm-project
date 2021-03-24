include_directories( ${CMAKE_CURRENT_BINARY_DIR}/..
                     ${CMAKE_CURRENT_SOURCE_DIR}/.. )

add_llvm_component_library(LLVMOPEN8Info
  OPEN8TargetInfo.cpp

  LINK_COMPONENTS
  Support

  ADD_TO_COMPONENT
  OPEN8
)

add_dependencies(LLVMOPEN8Info OPEN8CommonTableGen)

