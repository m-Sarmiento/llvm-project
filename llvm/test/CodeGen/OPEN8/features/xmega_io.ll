
; RUN: llc -O0 < %s -march=open8 `| FileCheck %s`

define i8 @read8_low_io() {
; CHECK-LABEL: read8_low_io
; XMEGA: in r24, 8
; AVR: lds r24, 8
  %1 = load i8, i8* inttoptr (i16 8 to i8*)
  ret i8 %1
}

define i8 @read8_hi_io() {
; CHECK-LABEL: read8_hi_io
; XMEGA: in r24, 40
; AVR: in r24, 8
  %1 = load i8, i8* inttoptr (i16 40 to i8*)
  ret i8 %1
}

define i8 @read8_maybe_io() {
; CHECK-LABEL: read8_maybe_io
; XMEGA: lds r24, 80
; AVR: in r24, 48
  %1 = load i8, i8* inttoptr (i16 80 to i8*)
  ret i8 %1
}

define i8 @read8_not_io(){
; CHECK-LABEL: read8_not_io
; XMEGA: lds r24, 160
; AVR: lds r24, 160
  %1 = load i8, i8* inttoptr (i16 160 to i8*)
  ret i8 %1
}
