//===-- OPEN8CallingConv.td - Calling Conventions for OPEN8 ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for OPEN8 architecture.
// Normal functions use a special calling convention, solved in code.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// OPEN8 Return Value Calling Convention
//===----------------------------------------------------------------------===//

// Special return value calling convention for runtime functions.
def RetCC_OPEN8_BUILTIN : CallingConv
<[
  CCIfType<[i8], CCAssignToReg<[R4,R5]>>,
  CCIfType<[i16], CCAssignToReg<[R5R4, R3R2]>>
]>;

//===----------------------------------------------------------------------===//
// OPEN8 Argument Calling Conventions
//===----------------------------------------------------------------------===//

// The calling conventions are implemented in custom C++ code

// Calling convention for variadic functions.
def ArgCC_OPEN8_Vararg : CallingConv
<[
  // i16 are always passed through the stack with an alignment of 1.
  CCAssignToStack<2, 1>
]>;

//===----------------------------------------------------------------------===//
// Callee-saved register lists.
//===----------------------------------------------------------------------===//

def CSR_Normal : CalleeSavedRegs<(add R7, R6)>;//, (sequence "R%u", 5, 2))>;
def CSR_Interrupts : CalleeSavedRegs<(add (sequence "R%u", 7, 0))>;
