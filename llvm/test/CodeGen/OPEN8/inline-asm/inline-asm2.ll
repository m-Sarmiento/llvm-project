; RUN: llc < %s -march=open8 -no-integrated-as `| FileCheck %s`

; CHECK-LABEL: foo
define void @foo(i16 %a) {
  call void asm sideeffect "add $0, $0", "r"(i16 %a) nounwind
  ret void
}

