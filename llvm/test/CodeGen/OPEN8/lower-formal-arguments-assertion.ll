; RUN: llc < %s -march=open8 `| FileCheck %s`

define void @foo(i1) {
; CHECK-LABEL: foo:
; CHECK: ret
  ret void
}
