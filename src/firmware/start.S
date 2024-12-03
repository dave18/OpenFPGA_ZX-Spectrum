//#include "custom_ops.S"

.section .text
.globl _start
_start:
    la    sp, __stacktop
	call main
	ebreak
	
