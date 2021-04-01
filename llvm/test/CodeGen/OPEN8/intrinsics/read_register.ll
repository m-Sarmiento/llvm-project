; RUN: llc -O0 < %s -march=open8 | FileCheck %s

; CHECK-LABEL: foo
define void @foo() {
entry:
  %val1 = call i16 @llvm.read_register.i16(metadata !0)
  %val2 = call i16 @llvm.read_register.i16(metadata !1)
  %val3 = call i8 @llvm.read_register.i8(metadata !2)
  ret void
}

declare i8 @llvm.read_register.i8(metadata)
declare i16 @llvm.read_register.i16(metadata)

!0 = !{!"r2"}
!1 = !{!"r6"}
!2 = !{!"r0"}
