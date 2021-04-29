            TTL Serial I/O driver implementing Circular FIFO
;****************************************************************
; It uses Interrupts to interact between terminal and the CPU
; It implements UART0_ISR and UART0_IRQ to service the interrupts to perform execution
;Name:  Dhruv Rajpurohit
;Date:  2 April 2020
;Class:  CMPE-250
;Section:  01L1, Thursday 2pm-3:50pm
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;February 5, 2018
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
		
; Equates for Queue Management Structure
MAX_BUFFER	EQU	    80
Safebuffer	EQU		79
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17
	
;Queue Structure Sizes
Q_BUF_SZ	EQU	    4	
Q_REC_SZ	EQU		18
    
;general
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
;---------------------------------------------------------------
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
			BL		Init_UART0_IRQ     
            CPSIE   I
            LDR     R0,=QBuffer			; Load Address of Queue Buffer in R0
            LDR     R1,=QRecord			; Load Address of Record Structure Management
			MOVS	R2,#Q_BUF_SZ
            BL      InitQueue			; Initialize the Record Structure to be used with buffer  later
Step3       LDR     R0,=String1			; Address of String1
			MOVS	R1,#Safebuffer
            BL      PutStringSB			;"Type a queue command (D,E,H,P,S):"
     
branchback	BL		GetChar				; Input a Characte
;If Its a lowercase Character// according to ASCII tables
; It will branch if any Lower case character is input
			CMP		R0,#100
			BEQ		BackD
			CMP		R0,#101
			BEQ		BackE
			CMP		R0,#104
			BEQ		BackH
			CMP		R0,#112
			BEQ		BackP
            CMP     R0,#115
            BEQ     BackS
;If its a uppercase Character// according to ASCII tables
; It will branch if any Uppercase Character is input
			CMP		R0,#68
			BEQ		D
			CMP		R0,#69
			BEQ		E
			CMP		R0,#72
			BEQ		H
			CMP		R0,#80
			BEQ		P
            CMP     R0,#83
            BEQ     S
			B		branchback			;It will branch back if these characters are not input
; Converting the uppercase characters into lowercase characters
; branches back to check at the lowercase branching
D			MOVS	R0,#100
			B		BackD
E			MOVS	R0,#101
			B		BackE
H			MOVS	R0,#104
			B 		BackH
P			MOVS	R0,#112
			B		BackP
S           MOVS    R0,#115
            B       BackS
BackD
            BL      PutChar			;Print d in the terminal
            BL      NewLine	
            LDR     R1,=QRecord
            BL      DeQueue			; DeQueues an Element in the buffer using outpointer inside subroutine
            BCS     unsuccessful	; If carrt flag is set branches to unsuccessful deQueue
            BL      PutChar			; PutChar the DeQueued Character
            MOVS    R0,#58			; print :
            BL      PutChar
            B       Step8
unsuccessful
            LDR     R0,=String2		; Address of String 2 in R0
            BL      PutStringSB		;Print   "Failure:"
            B       Step8			; Branch to give status
; If E or e has been typed in the terminal
; Enqueue Command
BackE
            BL      PutChar			; Print e in terminal
            BL      NewLine
            LDR     R0,=String3		; Address of String3 in terminal
            BL      PutStringSB		; print "Character to enqueue:"
            BL      GetChar			; Take Character to enqueue ( ASCII )
            BL      PutChar			; Print that character into terminal
            BL      NewLine
; Ro contains character
            LDR     R1,=QRecord		; pointer to find Pointers in Queue
            BL      EnQueue			; Enqueue character from R0
            BCS     unsuccessful2	; If carry flag is set, unsuccesful
            LDR     R0,=String4     ; Address of String4 in R0
            BL      PutStringSB		; Prints "Success:"
            B       Step8			; Status
unsuccessful2
            LDR     R0,=String2		; 
            BL      PutStringSB		; Prints Failure on the terminal screen
            B       Step8

;IF H or h has been typed into terminal
BackH       BL      PutChar			;prints h on the terminal screen
            BL      NewLine
            LDR     R0,=String5		;Address of string5 in R0
            BL      PutStringSB		;Prints "d (dequeue), e (enqueue), h (help), p (print), s (status)"
            BL      NewLine
            B       Step3			; Go to Step 3 for new Command

; If P or p has been typed into the terminal
BackP       BL      PutChar			; Prints p into terminal
            BL      NewLine
            MOVS    R0,#62			
            BL      PutChar			; Prints > onto the terminal
            LDR     R1,=QRecord		; Queue Record Structure into R1
            LDR     R2,[R1,#OUT_PTR]; Load Out Pointer address in R2
            MOVS    R4,#0			; Start Counter for number of elements	
            LDRB    R3,[R1,#NUM_ENQD] ; Number Enqueued in register
qp          LDRB    R0,[R2,#0]		; Load character from Outpointer ( first to dequeue)
            CMP     R4,R3           ; Compare with max 4 elements in queue
            BEQ     pfin			; If 4 printed, finish and exit
			BL      PutChar			; PutChar the element
            ADDS    R2,R2,#1		; Increase OutPointer
            LDR     R5,[R1,#BUF_PAST]; Load Buffer Past address in R5
            ADDS    R4,R4,#1		; Increase counter by 1
            CMP     R2,R5			;Compare outpointer and Buffer past address
            BHS     bstart     		; If R2-R5 is higher or same than zero, breanch to bstart
            B       qp				; branchback to qp for printing next element in queue
bstart      LDR     R2,[R1,#BUF_STRT]; Store Buffer start address as Outpointer to move to queue start for any element remaining
            B       qp				; Branch back to qp
; all characters are printed
pfin
            MOVS    R0,#60			; print < onto terminal
            BL      PutChar		
            BL      NewLine
            B       Step3			; branch back to step3 for new command

;If S or s has been typed into terminal
BackS       BL		PutChar
			BL		NewLine
			LDR     R0,=String6
            BL      PutStringSB		; print onto terminal "Status:"
            B       Step8			; move to step8
            
Step8
            LDR     R0,=String7 	; Print "IN=0x"
            BL      PutStringSB
            LDR     R1,=QRecord
            LDR     R0,[R1,#IN_PTR]	; in pointer address in R0
            BL      PutNumHex		; Print text decimal representation from R0
            MOVS    R0,#32			; Space ASCII print 
            BL      PutChar			
            MOVS    R0,#32
            BL      PutChar
            LDR     R0,=String8	    ; Print "Out=0x"
            BL      PutStringSB
            LDR     R0,[R1,#OUT_PTR] ; Out pointer address in R0
            BL      PutNumHex		 ; Prints text decimal represenation from R0
            MOVS    R0,#32			
            BL      PutChar
            MOVS    R0,#32			; prints Space ASCII 
            BL      PutChar
            LDR     R0,=String9		; Print "Num="
            BL      PutStringSB	
            LDRB    R0,[R1,#NUM_ENQD]	; Loads R0 with Number of Characters Enqueued ( Byte )
            BL      PutNumUB			; Use PutNumUB to print text decimal represenation of unsigned byte ( Number Enqueued )
            BL      NewLine
            B       Step3				; Branch back to Step 3
            
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<


InitQueue   PROC	{R3-R14}
            PUSH    {R0-R2}
            STR        R0,[R1,#IN_PTR]      ;Store Inpointer
            STR        R0,[R1,#OUT_PTR]     ;Store Outpointer Qbuffer
            STR        R0,[R1,#BUF_STRT]    ;Store Buffer Start
            MOVS    R2,#Q_BUF_SZ            ;Moving buffer size to R0
            ADDS    R0,R0,R2
            STR     R0,[R1,#BUF_PAST]       ;Store buffer past
            MOVS    R0,#0
            STRB    R0,[R1,#NUM_ENQD]       ;Store register byte of number enqued
            POP     {R0-R2}
            BX        LR
            ENDP
            

EnQueue     PROC    {R0-R14}
; Character is enqueued in Qbuffer using pointers from Qrecord
        PUSH    {R0-R5}                    ; Push into stack the registers being used
        LDRB     R2,[R1,#NUM_ENQD]        ;Loading Number Enqueued from Number Queue pointer
        MOVS    R5,#Q_BUF_SZ            ; move buffer size
        CMP        R2,R5                     ;Comparing with buffer size
        BHS        exit                    ;If higher or same, exit
        LDR     R4,[R1,#IN_PTR]            ; Load address of INpointer
        STRB    R0,[R4,#0]              ; Storing character in queue at inpointer
        ADDS    R2,R2,#1                   ; Incrementing NUM enqueued
        STRB    R2,[R1,#NUM_ENQD]        ; Storing incremented Value
        ADDS    R4,R4,#1                ;incrementing pointer value
        STR     R4,[R1,#IN_PTR]            ;Storing in memory with offset
        LDR     R3,[R1,#BUF_PAST]        ; Loading bufferpast
        CMP        R4,R3                    ; Comparing with Pointer
        BHS        branch                    ; Branch if higer or same, In pointer - Buffer Past
; Clear flag and return to indicate successful enqueue
carryclear
        MRS        R0,APSR
        MOVS    R1,#0x20
        LSLS    R1,R1,#24
        BICS    R0,R0,R1
        MSR        APSR,R0
        B        return
branch
; Circular FIFO
        LDR     R4,[R1,#BUF_STRT]    ; Initialise inpointer as buffer start
        STR     R4,[R1,#IN_PTR]        ;Storing bufferstart in inpointer
        B        carryclear            ; branch to carry clear
; Set Carry flag to denote failure in Enqueueing the character
exit
        MRS        R0,APSR
        MOVS    R1,#0x20
        LSLS    R1,R1,#24
        ORRS    R0,R0,R1
        MSR        APSR,R0
; Return from subroutine with either Carry flag set or cleared
return
        POP        {R0-R5}        ; Empty stack
        BX        LR            ; Return
        ENDP


DeQueue     PROC    {R1-R14}
;Removes characters in queue in accordance with First in First out option

        PUSH    {R1-R5}
        LDRB    R2,[R1,#NUM_ENQD]        ;Number enqueued value
        CMP        R2,#0                    ; Compared to zero to check if queue empty
        BLS        empty                    ; this means Queue is empty
        LDR     R3,[R1,#OUT_PTR]        ; get out pointer
        LDRB    R0,[R3,#0]                ; Get Character at Outpointer
        MOVS    R5,#0                    ; move zero to R5
        STRB    R5,[R3,#0]                ; Store zero in OutPointer
        SUBS    R2,R2,#1                ; Decrement NUmber enqueued
        STRB    R2,[R1,#NUM_ENQD]        ; Store decremented value
        ADDS    R3,R3,#1                ;Incremenet    out pointer
        STR     R3,[R1,#OUT_PTR]        ; Store out pointer
        LDR     R4,[R1,#BUF_PAST]        ; load buffer past // last of queue
        CMP        R3,R4                    ; Outpointer - Buffer Past
        BLO        clearflag                ; if lower than zero, clear flag
        LDR     R3,[R1,#BUF_STRT]        ; Initialize inpointer to buffer start
        STR     R3,[R1,#OUT_PTR]        ; Storing buffer start as outpointer
clearflag
; Successful dequeue// clear flag
        PUSH    {R0}
        MRS        R0,APSR
        MOVS    R1,#0x20
        LSLS    R1,R1,#24
        BICS    R0,R0,R1
        MSR        APSR,R0
        POP     {R0}
        B        return1
empty
;SetFlag since no element to be dequeued // or Queue Empty
        MRS        R0,APSR
        MOVS    R1,#0x20
        LSLS    R1,R1,#24
        ORRS    R0,R0,R1
        MSR        APSR,R0
return1
; Return with C falg set or Cleared
        POP        {R1-R5}
        BX        LR
        ENDP

                
GetStringSB         PROC {R0-R13},{}

                    PUSH  {R0-R3,LR}
                    MOVS  R3,#0
                    MOVS  R4,R0
                    SUBS  R1,R1,#1
                    BL    GetChar
While
                    CMP   R0,#CR
                    BEQ   nextline
                    ADDS  R3,R3,#1
                    CMP   R3,R1
                    BHI   EndWhile
                    BL    PutChar
                    STRB  R0,[R4,#0]
                    ADDS  R4,R4,#1
                    BL    GetChar
                    B     While
EndWhile
                    BL    GetChar
                    CMP   R0,#CR
                    BEQ   nextline
                    B     EndWhile
nextline
                    MOVS  R0,#NULL
                    BL    PutChar
                    STRB  R0,[R4,#0]
                    MOVS  R0,#CR
                    BL    PutChar
                    MOVS  R0,#LF
                    BL    PutChar
                    POP   {R0-R3,PC}
                    BX LR
                    ENDP

PutStringSB         PROC {R0-R13},{}

                    PUSH  {R1-R4,LR}
                    MOVS  R4,#0
Whileloop           LDRB  R2,[R0,#0]
                    CMP   R2,#NULL
                    BEQ   EndWhileLoop
                    ADDS  R4,R4,#1
                    CMP   R4,R1
                    BHS   EndWhileLoop
                    PUSH  {R0}
                    MOVS  R0,R2
                    BL    PutChar
                    POP   {R0}
                    ADDS  R0,R0,#1
                    B     Whileloop
EndWhileLoop
                    POP   {R1-R4,PC}
                    BX LR
                    ENDP
                    
PutNumU		PROC {R0-R15}
; It prints to the terminal screen the text decimal representation of unsigned word


			PUSH	{R0-R4,LR}		; Push into Stack to preserve
			MOVS	R4,#0			;Counter initialization 
WhileDIVU
			MOVS	R1,R0			; Move R0 into R1
			MOVS	R0,#10			;Divizor is 10
			BL 		DIVU			; Divide by 10
			PUSH	{R1}			; Push remainder into stack ( continously ) 
			ADDS	R4,R4,#1		; increase counter
			CMP		R0,#0			; Compare qoutient with 0 
			BEQ		print			; if equal move to print
			B		WhileDIVU		; branch back for DIVU again


print		
			CMP		R4,#0			; Compare if Counter is zero
			BEQ		out2			; If equal move out
			POP		{R1}			; Pop Remaindera value ( all values )
			ADDS	R1,R1,#0x30		; Convert Word into text ( ASCII )
			MOVS	R0,R1			; Move the convert into R0 for display
			BL		PutChar
			SUBS	R4,R4,#1		;Decrement Counter
			B		print			; move back to print
out2				
			POP		{R0-R4,PC}		; Empty Stack
			BX		LR				; Return
			ENDP	
			
DIVU	PROC  {R2,R14}
; Computes unsigned Integer Division of Dividend in R1 by
				 
			    PUSH	{R2}			; push in stack for use
				MOVS	R2,#0			; Qoutient is zero		
				CMP		R0,#0			; Comparing Divisor
				BEQ		DivisorZero		;If divisor is zero,branch to DivisorZero
				CMP		R1,#0			;Comparing Dividend to zero
				BEQ		DividendZero	;If Dividend is zero,branch to DividendZero
WHILE			CMP		R1,R0			;Comparison to find if Dividend is less
				BLO		Bitclear		;If dividend is lessthan divisor
				SUBS	R1,R1,R0		;Dividend-Divisor/Remainder
				ADDS	R2,R2,#1		;Qoutient+1
				B		WHILE			;Jump back to WHILE
				
;If Dividend is less than Divisor, Clearing C flag
Bitclear				
				MOVS	R0,R2			;Qoutient to R0
				PUSH	{R0,R1}			;Push R1 and R0 into stack
				MRS		R0,APSR			;Copy contents of ASPR into R0
				MOVS	R1,#0x20		;moving x20 into R1
				LSLS	R1,R1,#24		;Left shift in R1
				BICS	R0,R0,R1		;BIT clear using R0 and R1
				MSR		APSR,R0			;Moving cleared APSR back 
				POP		{R0,R1}			;Pop out of stack to restore values
				B		exit2
;If the Divizor is zero// subsequent Bit set
DivisorZero		PUSH	{R0,R1}
				MRS		R0,APSR			;Moving APSR into R0
				MOVS 	R1,#0x20		;Mask by moving x20 into R1
				LSLS	R1,R1,#24		;Shifting to get MSBs 
				ORRS	R0,R0,R1		;Using OR with mask to set bits
				MSR		APSR,R0			;Set C flag into APSR 
				POP		{R1}			;POP out of stack
				POP		{R0}			;POP out of stack
				B		exit2			;return with flag set divisor_zero
;If the Dividend is zero//subsequent flag clear
DividendZero
				MOVS	R0,R2			;Move R2 into R0//Qoutient in R0
				B		Bitclear		;Go to Bit clear and exit
exit2			POP		{R2}			;pop R2 out
				BX		LR				;exit Subroutine
				ENDP

NewLine      PROC  {R0-R15}
;Moves the cursor to the next line in terminal
            PUSH    {R0,LR}
            MOVS    R0,#CR
            BL      PutChar
            MOVS    R0,#LF
            BL      PutChar
            POP     {R0,PC}
            BX      LR
            ENDP
 

PutNumHex   PROC   {R0-R15}
; Prints the text decimal represenation of unsigned word value in R0

            PUSH   {R1-R3,LR}
            MOVS    R3,#0            ;initalize counter
            MOVS    R2,#0x0000000F    ; prepare mask
            MOVS    R1,R0            ; move word in R1
            MOVS    R4,R0            ; move Word in R4
shift
            CMP     R3,#8            ; Since we know 8 character will be there
            BEQ     printhex        ; compare if 8 Characters are printed
            ANDS    R4,R4,R2        ; AND Word with mask
            MOVS    R0,R4            ; Move character in R0
            ADDS    R3,R3,#1        ; incrememnt Counter
            CMP     R0,#9            ; Compare if character is a number
            BGT     alpha           ; otehrwise alpha
            ADDS    R0,R0,#48        ; Convert character into Number ASCII
            B       pushstack        ; Push the chracter into Stack
alpha       ADDS    R0,R0,#55        ; Convert the character into alphabet ( ASCII )
            B       pushstack        ; Push into Stack
pushstack
            PUSH{R0}                ; Push into stack to be used later
            LSRS    R1,R1,#4        ; Left shift the word by 4bits to find next character
            MOVS    R4,R1            ; Move shift address into R4
            B       shift            ; go back to shift again with new R4
; Prints the characters in stack
printhex
            CMP     R3,#0            ; Check for number of characters printed
            BEQ     printfin        ; if all printed, return
            POP     {R0}            ; pop the last character stacked
            BL      PutChar            ; print this charcter
            SUBS    R3,R3,#1        ; decrement character
            B       printhex        ; branch back to print next character
printfin
            POP     {R1-R3,PC}        ; empty stack to return
            BX      LR
            ENDP
            


PutNumUB    PROC    {R0-R15}
; Prints to the terminal screen the text decimal representation of unsigned byte value in R0

            PUSH    {R0-R2,LR}
            MOVS    R1,R0            ;Move Byte in R1 from R0 // precautionary
            MOVS    R2,#0x0000000F    ; Move mask in R2
            ANDS    R1,R1,R2        ; AND operation to preserve the least byte of word
            MOVS    R0,R1            ; Move Least bye in R0
            BL      PutNumU            ; Call PutNumU
            POP     {R0-R2,PC}        ; Empty Stack
            BX      LR                ; return
            ENDP



Init_UART0_IRQ    PROC  {R1-R14}
            PUSH    {R0-R5,LR}
            LDR     R0,=RxQBuffer            ; Load Queue Buffer address in R0
            LDR     R1,=RxRecord             ; Record structure is received
            MOVS    R2,#MAX_BUFFER           ; 80characters
            BL      InitQueue                ; Initialize the Record Structure to be used with buffer  later
            
            LDR     R0,=TxQBuffer            ; Load Address of Queue Buffer in R0
            LDR     R1,=TxRecord             ; Transmit record structure
            MOVS    R2,#MAX_BUFFER           ;R2 characters
            BL      InitQueue                ;Initialize the Record Structure to be used with buffer  later
            
;Select MCGPLLCLK / 2 as UART0 clock source
            LDR R1,=SIM_SOPT2
            LDR R2,=SIM_SOPT2_UART0SRC_MASK
            LDR R3,[R1,#0]
            BICS R3,R3,R2
            LDR R2,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
            ORRS R3,R3,R2
            STR R3,[R1,#0]
;Enable external connection for UART0
            LDR R1,=SIM_SOPT5
            LDR R2,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
            LDR R3,[R1,#0]
            BICS R3,R3,R2
            STR R3,[R1,#0]
;Enable clock for UART0 module
            LDR R1,=SIM_SCGC4
            LDR R2,=SIM_SCGC4_UART0_MASK
            LDR R3,[R1,#0]
            ORRS R3,R3,R2
            STR R3,[R1,#0]
;Enable clock for Port A module
            LDR R1,=SIM_SCGC5
            LDR R2,= SIM_SCGC5_PORTA_MASK
            LDR R3,[R1,#0]
            ORRS R3,R3,R2
            STR R3,[R1,#0]
;UART0 Rx and UART0 Tx signals available through Port A to the OpenSDA connector
            LDR R1,=PORTA_PCR1
            LDR R2,=PORT_PCR_SET_PTA1_UART0_RX
            STR R2,[R1,#0]
;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
            LDR R1,=PORTA_PCR2
            LDR R2,=PORT_PCR_SET_PTA2_UART0_TX
            STR R2,[R1,#0]
 

;Disable UART0 receiver and transmitter
            LDR R1,=UART0_BASE
            MOVS R2,#UART0_C2_T_R
            LDRB R3,[R1,#UART0_C2_OFFSET]
            BICS R3,R3,R2
            STRB R3,[R1,#UART0_C2_OFFSET]

;Set UART0 IRQ priority
            LDR     R0,=UART0_IPR
            LDR     R1,=NVIC_IPR_UART0_MASK
            LDR     R2,=NVIC_IPR_UART0_PRI_3
            LDR     R3,[R0,#0]
            BICS R3,R3,R1
            ORRS R3,R3,R2
            STR     R3,[R0,#0]

;Clear any pending UART0 interrupts
            LDR     R0,=NVIC_ICPR
            LDR     R1,=NVIC_ICPR_UART0_MASK
            STR  R1,[R0,#0]

;Unmask UART0 interrupts
            LDR  R0,=NVIC_ISER
            LDR  R1,=NVIC_ISER_UART0_MASK
            STR     R1,[R0,#0]
            
;Set UART0 for 9600 baud, 8N1 protocol
            LDR  R1,=UART0_BASE
            MOVS R2,#UART0_BDH_9600
            STRB R2,[R1,#UART0_BDH_OFFSET]
            MOVS R2,#UART0_BDL_9600
            STRB R2,[R1,#UART0_BDL_OFFSET]
            MOVS R2,#UART0_C1_8N1
            STRB R2,[R1,#UART0_C1_OFFSET]
            MOVS R2,#UART0_C3_NO_TXINV
            STRB R2,[R1,#UART0_C3_OFFSET]
            MOVS R2,#UART0_C4_NO_MATCH_OSR_16
            STRB R2,[R1,#UART0_C4_OFFSET]
            MOVS R2,#UART0_C5_NO_DMA_SSR_SYNC
            STRB R2,[R1,#UART0_C5_OFFSET]
            MOVS R2,#UART0_S1_CLEAR_FLAGS
            STRB R2,[R1,#UART0_S1_OFFSET]
            MOVS R2, \
                 #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
            STRB R2,[R1,#UART0_S2_OFFSET]

;Enable receive interrupt
            MOVS R2,#UART0_C2_T_RI
            STRB R2,[R1,#UART0_C2_OFFSET]
            POP  {R0-R5,PC}
            ENDP
            LTORG



UART0_ISR    PROC    {R0-R14}
;Initialising Inetrrupt Service routine
            CPSID    I
            PUSH    {R4-R7,LR}

            LDR        R1,=UART0_BASE
            MOVS    R2,#UART0_C2_TIE_MASK
            LDRB     R3,[R1,#UART0_C2_OFFSET]
            ANDS    R2,R2,R3
            BEQ     Disabled
            LDRB     R2,[R1,#UART0_S1_OFFSET]
            MOVS    R4,#UART0_S1_TDRE_MASK
            ANDS    R2,R2,R4
            BEQ     Disabled
            LDR        R1,=TxRecord
            BL        DeQueue
            BCS        Disable
            LDR     R1,=UART0_BASE
            STRB    R0,[R1,#UART0_D_OFFSET]
            B       Disabled
Disable
            LDR     R1,=UART0_BASE
            MOVS    R5,#UART0_C2_T_RI
            STRB     R5,[R1,#UART0_C2_OFFSET]
            
Disabled
            LDRB     R2,[R1,#UART0_S1_OFFSET]
            MOVS    R4,#UART0_S1_RDRF_MASK
            ANDS    R2,R2,R4
            BEQ     final
            LDRB    R0,[R1,#UART0_D_OFFSET]
            LDR        R1,=RxRecord
            BL        EnQueue
final
            POP        {R4-R7,PC}
            CPSIE    I
            ENDP
                
GetChar        PROC    {R1-R14}
;DeQueues Chracter from RxQueue
            PUSH {R1,LR}
LoopGet
            CPSID I
            LDR      R1,=RxRecord
            BL      DeQueue
            CPSIE I
            BCS      LoopGet
            POP   {R1,PC}
            ENDP


PutChar        PROC    {R0-R14}
;EnQueues a character

            PUSH    {R0-R2,LR}
LoopPut
            CPSID    I
            LDR        R1,=TxRecord
            BL        EnQueue
            CPSIE    I
            BCS        LoopPut
            LDR     R1,=UART0_BASE
            MOVS R2,#UART0_C2_TI_RI
            STRB R2,[R1,#UART0_C2_OFFSET]
            POP        {R0-R2,PC}
            ENDP
				
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR	      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
String1       DCB    "Type a queue command (D,E,H,P,S):",NULL
String2       DCB    "Failure:",NULL
String3       DCB    "Character to enqueue:",NULL
String4       DCB    "Success:",NULL
String5       DCB    "d (dequeue), e (enqueue), h (help), p (print), s (status)",NULL
String6       DCB    "Status:",NULL
String7       DCB    "IN=0x",NULL
String8       DCB    "Out=0x",NULL
String9       DCB    "Num=",NULL
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<< 
RxQBuffer	SPACE	MAX_BUFFER
            ALIGN
RxRecord	SPACE	Q_REC_SZ
TxQBuffer	SPACE	MAX_BUFFER
            ALIGN
TxRecord	SPACE	Q_REC_SZ
QBuffer     SPACE   Q_BUF_SZ
            ALIGN
QRecord     SPACE   Q_REC_SZ
;>>>>>   end variables here <<<<<
            ALIGN
            END