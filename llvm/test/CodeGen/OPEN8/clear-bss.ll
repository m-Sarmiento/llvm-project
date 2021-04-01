; RUN: llc < %s -march=open8 `| FileCheck %s`

; CHECK: .globl __do_clear_bss
@zeroed = internal constant [3 x i8] zeroinitializer

