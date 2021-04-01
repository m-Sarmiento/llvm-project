; RUN: llc < %s -march=open8 `| FileCheck %s`

; CHECK: .globl __do_copy_data
@str = internal global [3 x i8] c"foo"

