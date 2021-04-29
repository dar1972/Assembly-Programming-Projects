  TTL CMPE Exercise03
;****************************************************************
;Descriptive comment header goes here.
;Computes arithmetic operations using memory and branching
;Name:  Dhruv Rajpurohit
;Date:  23 January 2020
;Class:  CMPE-250
;Section: 01L1, Thursday, 2pm to 3:50pm
;---------------------------------------------------------------
;Keil Simulator Template for KL46
;R. W. Melton
;January 5, 2018
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates
;Standard data masks
BYTE_MASK         EQU  0xFF
NIBBLE_MASK       EQU  0x0F
;Standard data sizes (in bits)
BYTE_BITS         EQU  8
NIBBLE_BITS       EQU  4
;Architecture data sizes (in bytes)
WORD_SIZE         EQU  4  ;Cortex-M0+
HALFWORD_SIZE     EQU  2  ;Cortex-M0+
;Architecture data masks
HALFWORD_MASK     EQU  0xFFFF
;Return                 
RET_ADDR_T_MASK   EQU  1  ;Bit 0 of ret. addr. must be
                          ;set for BX, BLX, or POP
                          ;mask in thumb mode
;---------------------------------------------------------------
;Vectors
VECTOR_TABLE_SIZE EQU 0x000000C0  ;KL46
VECTOR_SIZE       EQU 4           ;Bytes per vector
;---------------------------------------------------------------
;CPU CONTROL:  Control register
;31-2:(reserved)
;   1:SPSEL=current stack pointer select
;           0=MSP (main stack pointer) (reset value)
;           1=PSP (process stack pointer)
;   0:nPRIV=not privileged
;        0=privileged (Freescale/NXP "supervisor") (reset value)
;        1=not privileged (Freescale/NXP "user")
CONTROL_SPSEL_MASK   EQU  2
CONTROL_SPSEL_SHIFT  EQU  1
CONTROL_nPRIV_MASK   EQU  1
CONTROL_nPRIV_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PRIMASK:  Interrupt mask register
;31-1:(reserved)
;   0:PM=prioritizable interrupt mask:
;        0=all interrupts unmasked (reset value)
;          (value after CPSIE I instruction)
;        1=prioritizable interrrupts masked
;          (value after CPSID I instruction)
PRIMASK_PM_MASK   EQU  1
PRIMASK_PM_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PSR:  Program status register
;Combined APSR, EPSR, and IPSR
;----------------------------------------------------------
;CPU APSR:  Application Program Status Register
;31  :N=negative flag
;30  :Z=zero flag
;29  :C=carry flag
;28  :V=overflow flag
;27-0:(reserved)
APSR_MASK     EQU  0xF0000000
APSR_SHIFT    EQU  28
APSR_N_MASK   EQU  0x80000000
APSR_N_SHIFT  EQU  31
APSR_Z_MASK   EQU  0x40000000
APSR_Z_SHIFT  EQU  30
APSR_C_MASK   EQU  0x20000000
APSR_C_SHIFT  EQU  29
APSR_V_MASK   EQU  0x10000000
APSR_V_SHIFT  EQU  28
;----------------------------------------------------------
;CPU EPSR
;31-25:(reserved)
;   24:T=Thumb state bit
;23- 0:(reserved)
EPSR_MASK     EQU  0x01000000
EPSR_SHIFT    EQU  24
EPSR_T_MASK   EQU  0x01000000
EPSR_T_SHIFT  EQU  24
;----------------------------------------------------------
;CPU IPSR
;31-6:(reserved)
; 5-0:Exception number=number of current exception
;      0=thread mode
;      1:(reserved)
;      2=NMI
;      3=hard fault
;      4-10:(reserved)
;     11=SVCall
;     12-13:(reserved)
;     14=PendSV
;     15=SysTick
;     16=IRQ0
;     16-47:IRQ(Exception number - 16)
;     47=IRQ31
;     48-63:(reserved)
IPSR_MASK             EQU  0x0000003F
IPSR_SHIFT            EQU  0
IPSR_EXCEPTION_MASK   EQU  0x0000003F
IPSR_EXCEPTION_SHIFT  EQU  0
;----------------------------------------------------------
PSR_N_MASK           EQU  APSR_N_MASK
PSR_N_SHIFT          EQU  APSR_N_SHIFT
PSR_Z_MASK           EQU  APSR_Z_MASK
PSR_Z_SHIFT          EQU  APSR_Z_SHIFT
PSR_C_MASK           EQU  APSR_C_MASK
PSR_C_SHIFT          EQU  APSR_C_SHIFT
PSR_V_MASK           EQU  APSR_V_MASK
PSR_V_SHIFT          EQU  APSR_V_SHIFT
PSR_T_MASK           EQU  EPSR_T_MASK
PSR_T_SHIFT          EQU  EPSR_T_SHIFT
PSR_EXCEPTION_MASK   EQU  IPSR_EXCEPTION_MASK
PSR_EXCEPTION_SHIFT  EQU  IPSR_EXCEPTION_SHIFT
MUL2				 EQU  1
MUL4			     EQU  2
NUMBER1				 EQU  127
NUMBER2				 EQU  128
;----------------------------------------------------------
;Stack
SSTACK_SIZE EQU  0x00000100
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
Reset_Handler  PROC {}
main
;---------------------------------------------------------------
;Initialize registers R0-R12
            BL      RegInit
;>>>>> begin main program code <<<<<
; Loading registers P and Q
			LDR 	R1,=P
			LDR 	R1,[R1,#0]
			LDR		R2,=Q
			LDR		R2,[R2,#0]
;R3<- P
			MOVS	R3,R1
			MOVS	R6,#NUMBER1
			CMP		R3,R6
			BGT		overflowF1
			MOVS	R7,#NUMBER2
			CMN		R3,R7
			BLT		overflowF1
			B		moveF1

moveF1
			
							
;R3<- P + 3*Q
			LSLS	R4,R2,#MUL2  	;2*Q
			CMP		R4,R6
			BGT		overflowF2
			CMN		R4,R7
			BLT		overflowF2
			ADDS    R4,R4,R2		;3*Q
			CMP		R4,R6
			BGT		overflowF2
			CMN		R4,R7
			BLT		overflowF2
			B		moveF2
moveF2
			ADDS	R3,R1,R4	  	;P + 3*Q
			CMP		R3,R6
			BGT		overflowF3
			CMN		R3,R7
			BLT		overflowF3
			B		moveF3
moveF3
;R3<- P + 3*Q + 65
			LDR		R4,=const_F
			LDR		R4,[R4,#0]
			ADDS    R3,R3,R4 	;P + 3*Q + 65
			CMP		R3,R6
			BGT		overflowF4
			CMN		R3,R7
			BLT		overflowF4
			B		moveF4

overflowF1
			SUBS	R3,R3,R3
overflowF2
			SUBS	R4,R4,R4
overflowF3
			SUBS	R3,R3,R3
overflowF4
			SUBS	R3,R3,R3
moveF4
			LDR		R0,=F
			STR		R3,[R0,#0]
;Calculating G Value
;R5<- 3*P
			LSLS	R5,R1,#MUL2 	;2*P
			ADDS	R5,R5,R1		;3*P
			CMP		R5,R6
			BGT		overflowG1
			CMN		R5,R7
			BLT		overflowG1
			B		moveG1
moveG1
;R5<- 3*P - 2*Q
			LSLS	R4,R2,#MUL2		; 2*Q
			CMP		R4,R6
			BGT		overflowG2
			CMN		R4,R7
			BLT		overflowG2
			B		moveG2
moveG2
;R5<- 3*P - 2*Q + 12	
			LSLS	R5,R5,#24
			LSLS	R4,R4,#24
			SUBS	R5,R5,R4		;3*P - 2*Q@(MSB)
			BVS		overflowG3
			B		moveG3
moveG3
;R5<- 3*P - 2*Q + 12
			LDR		R0,=const_G
			LDR		R0,[R0,#0]
			LSLS	R0,R0,#24
			ADDS	R5,R5,R0		;3*P - 2*Q + 12
			BVS		overflowG4
			B		moveG4

overflowG1
			SUBS	R5,R5,R5
overflowG2
			SUBS	R4,R4,R4
overflowG3
			SUBS	R5,R5,R5
overflowG4
			SUBS	R5,R5,R5
moveG4
			ASRS	R5,R5,#24
			LDR		R1,=G
			STR		R5,[R1,#0]
	
;Result<- F + G
			ADDS	R7,R3,R5	; Result = F + G
			NOP
			CMP		R7,#127
			BGT		overflowresult
			MOVS	R1,#128
			CMN		R7,R1
			B		moveresult
overflowresult
			SUBS	R7,R7,R7
moveresult
			LDR		R2,=Result
			STR		R7,[R2,#0]
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;---------------------------------------------------------------
RegInit     PROC  {}
;****************************************************************
;Initializes register n to value 0xnnnnnnnn, for n in 
;{0x0-0xC,0xE}
;****************************************************************
;Put return on stack
            PUSH    {LR}
;Initialize registers
            LDR     R1,=0x11111111
            ADDS    R2,R1,R1
            ADDS    R3,R2,R1
            ADDS    R4,R3,R1
            ADDS    R5,R4,R1
            ADDS    R6,R5,R1
            ADDS    R7,R6,R1
            ADDS    R0,R7,R1
            MOV     R8,R0
            ADDS    R0,R0,R1
            MOV     R9,R0
            ADDS    R0,R0,R1
            MOV     R10,R0
            ADDS    R0,R0,R1
            MOV     R11,R0
            ADDS    R0,R0,R1
            MOV     R12,R0
            ADDS    R0,R0,R1
            ADDS    R0,R0,R1
            MOV     R14,R0
            MOVS    R0,#0
            POP     {PC}
            ENDP
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;reset vector
            SPACE  (VECTOR_TABLE_SIZE - (2 * VECTOR_SIZE))
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
const_F DCD 65
const_G DCD 12
;>>>>>   end constants here <<<<<
;****************************************************************
            AREA    |.ARM.__at_0x1FFFE000|,DATA,READWRITE,ALIGN=3
            EXPORT  __initial_sp
;Allocate system stack
            IF      :LNOT::DEF:SSTACK_SIZE
SSTACK_SIZE EQU     0x00000100
            ENDIF
Stack_Mem   SPACE   SSTACK_SIZE
__initial_sp
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
P 	   SPACE 4
Q 	   SPACE 4
F 	   SPACE 4
G 	   SPACE 4
Result SPACE 4
;>>>>>   end variables here <<<<<
            END