
	; TEST PROGRAM

  	; setting the value of n1
;	MOV 30H, #67H
;	MOV 31H, #2FH
;	MOV 32H, #56H
;	MOV 33H, #7DH

	; setting the value of n2
;	MOV 34H, #0A3H
;	MOV 35H, #71H
;	MOV 36H, #15H
;	MOV 37H, #33H

;	CALL mul32bit

;	JMP $			; finished - do nothing

	; END OF TEST PROGRAM

USING 0
LongMult_code_seg SEGMENT CODE 	; code segment for this function
RSEG LongMult_code_seg 			; switch to this code segment

; This subroutine multiples two 32-bit numbers.
; inputs:	a 32-bit number (n1) stored in locations 30H (high byte) to 33H
;		  	a 32-bit number (n2) stored in locations 34H (high byte) to 37H
; output:	a 64-bit number (r) stored in locations 38H (high byte) to 3FH
PUBLIC mul32bit
mul32bit:
	PUSH PSW
	PUSH ACC
	PUSH B
	PUSH AR0
	PUSH AR1

	; clear RAM locations where result will be stored
 	CLR A
	MOV R0, #38H
 	MOV R1, #8
clrNext:
 	MOV @R0, A
 	INC R0
 	DJNZ R1, clrNext


	MOV B, 37H			; store low byte (byte 0) of n2 in B 
	MOV R1, #3FH		; store location of low byte (byte 0) of r (the result) in R1
	CALL byteMul		; multiply n1 by byte 0 of n2
	MOV B, 36H	   		; store byte 1 of n2 in B 
	DEC R1				; point R1 at byte 1 of r
	CALL byteMul		; multiply n1 by byte 1 of n2
	MOV B, 35H	  		; store byte 2 of n2 in B
	DEC R1				; point R1 at byte 2 of r
	CALL byteMul		; multiply n1 by byte 2 of n2
	MOV B, 34H			; store byte 3 of n2 in B
	DEC R1				; multiply n1 by byte 3 of n2
	CALL byteMul		; multiply n1 by byte 3 of n2


	POP AR1
	POP AR0
	POP B
	POP ACC
	POP PSW
	RET
; end of mul32bit subroutine


; This subroutine multiples all four bytes of n1 by one byte of n2.
; inputs:	the current byte of n2, stored in B
;			a pointer to the corresponding byte of r (the result), stored in R1
; output:	the results of the multiplications are added to the corresponding bytes of r
byteMul:
	PUSH PSW
	PUSH ACC
	PUSH B
	PUSH AR0
	PUSH AR1
	PUSH AR2
	PUSH AR3
	PUSH AR4

	MOV R3, B			; move value in B (current byte of n2) to R3 for later use
	MOV R0, #33H		; store location of low byte (byte 0) of n1 in R0
	MOV R2, #4			; 4 bytes need to be multiplied - R2 will be used to keep track

loop:
	MOV B, R3			; retrieve original B value from R3
	MOV A, @R0			; move current byte of n1 to A
	MUL AB				; multiply it by current byte of n2
	ADD A, @R1			; add the low byte of the result (in A) to current byte of r
	MOV @R1, A			; move the result to current byte of r
	DEC R1				; point R1 to next byte of r				
	MOV A, B			; move the high byte of result of MUL AB to A
	ADDC A, @R1			; and add it (plus any carry from the previous addition) to current byte of r
	MOV @R1, A			; move the result to current byte of r
	MOV R4, AR1			; move value in R1 to R4 for later use
	JNC endPropCarry	; if no carry, no need to propagate it

propCarry:
	DEC R1				; if there is a carry, point to next byte of r
	CJNE R1, #37H, skip	; if R1 is still pointing to a byte of r, skip next instruction
	JMP endPropCarry	; if R1 is now pointing beyond r, end of carry propagation has been reached
skip:
	MOV A, @R1			; move current byte of r to A
	INC A				; then increment it (ie: adding the carry)
	MOV @R1, A 			; and move the result back to the current byte or f
	JNZ endPropCarry	; if A is not zero, no need to propagate the carry any further
	JMP propCarry		; if it is, the carry must be added to the next byte of r
endPropCarry:
	MOV R1, AR4			; move the value that was in R1 before the carry propagation back to R1

	DEC R0				; point R0 at next byte of n1
	DJNZ R2, loop		; decrement R2 - if it's still not zero, then the four bytes of n1 have
						;					not yet been multiplied by the current byte of n2, therefore
						;					loop back

	POP AR4
	POP AR3
	POP AR2
	POP AR1
	POP AR0
	POP B
	POP ACC
	POP PSW 
	RET
; end of byteMul subroutine


END