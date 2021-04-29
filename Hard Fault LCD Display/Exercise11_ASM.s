            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  Dhruv Rajpurohit
;Date:  04/16/2020
;Class:  CMPE-250
;Section:  01L1 Thursday 2pm to 3:50pm
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;November 13, 2017
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;****************************************************************
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

; Equates for this lab
PWM_2ms     EQU     TPM_CnV_PWM_DUTY_2ms
PWM_1ms     EQU     TPM_CnV_PWM_DUTY_1ms 
TPM_CnV_PWM_DUTY_2ms    EQU     6700
TPM_CnV_PWM_DUTY_1ms    EQU     3000   
DAC0_STEPS  EQU 4096
SERVO_POSITIONS EQU 5
    
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
    
;****************************************************************
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY                
            EXPORT  Init_UART0_IRQ
            EXPORT  UART0_IRQHandler    
            EXPORT  GetChar
            EXPORT  PutChar
            EXPORT  GetStringSB
            EXPORT  PutStringSB
            EXPORT  PutNumUB
            EXPORT  PutNumHex               
;>>>>> begin subroutine code <<<<<
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


UART0_IRQHandler
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


InitQueue   PROC    {R3-R14}
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
                    


PutNumU             PROC {R0-R15}

                    PUSH  {R0-R4,LR}
                    MOVS  R4,#0
Loop_DIVU
                    MOVS  R1,R0
                    MOVS  R0,#10
                    BL    DIVU
                    PUSH  {R1}
                    ADDS  R4,R4,#1
                    CMP   R0,#0
                    BEQ   print
                    B     Loop_DIVU


print
                    CMP   R4,#0
                    BEQ   out
                    POP   {R1}
                    ADDS  R1,R1,#0x30
                    MOVS  R0,R1
                    BL    PutChar
                    SUBS  R4,R4,#1
                    B     print
out
                    POP   {R0-R4,PC}
                    BX LR
                    ENDP

			
DIVU                PROC {R3-R14}
                    PUSH {R2}
                    CMP R0, #0
                    BEQ DIV_BY_Zero
                    
                    
                    CMP R1, #0
                    BEQ SMALL_Div
                                
                    MOVS R2, #0
                    CMP R1, R0
                    BEQ Equal

SUB_LOOP            CMP R1, R0
                    BLO ENDW
                    SUBS R1, R1, R0
                    ADDS R2, R2, #1
                    B   SUB_LOOP

SMALL_Div
                    MRS   R0,APSR
                    MOVS  R1,#0x20
                    LSLS  R1,R1,#24
                    BICS  R0,R0,R1
                    MSR   APSR,R0
                    
                    MOVS R0, #0
                    MOVS R1, #0
                    B  THEEND
DIV_BY_Zero
        
                    MRS   R0, APSR
                    MOVS  R1,#0x20
                    LSLS  R1,R1,#24
                    ORRS R0,R0,R1
                    MSR  APSR,R0
                    
                    B THEEND

Equal               SUBS R1, R1, R0
                    ADDS R2, R2, #1
                    MOV R0, R2
                    PUSH {R1}
                    PUSH {R0}
                    MRS   R0,APSR
                    MOVS  R1,#0x20
                    LSLS  R1,R1,#24
                    BICS  R0,R0,R1
                    MSR   APSR,R0
                    POP {R1}
                    POP {R0}
                    B THEEND

ENDW
                    MOV R0, R2
                    PUSH {R1}
                    PUSH {R0}
                    MRS   R0,APSR
                    MOVS  R1,#0x20
                    LSLS  R1,R1,#24
                    BICS  R0,R0,R1
                    MSR   APSR,R0
                    POP {R1}
                    POP {R0}
                    B THEEND

THEEND              POP {R2}
                    BX LR

                    

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
                
;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
            EXPORT  DAC0_table_0
            EXPORT  PWM_duty_table_0 
;>>>>> begin constants here <<<<<
DAC0_table_0
DAC0_table
          DCW ((DAC0_STEPS - 1) / (SERVO_POSITIONS * 2))
          DCW (((DAC0_STEPS - 1) * 3) / (SERVO_POSITIONS * 2))
          DCW (((DAC0_STEPS - 1) * 5) / (SERVO_POSITIONS * 2))
          DCW (((DAC0_STEPS - 1) * 7) / (SERVO_POSITIONS * 2))
          DCW (((DAC0_STEPS - 1) * 9) / (SERVO_POSITIONS * 2))

PWM_duty_table
PWM_duty_table_0
;Servo positions
    DCW PWM_2ms
    DCW ((3*(PWM_2ms-PWM_1ms)/4)+PWM_1ms)
    DCW (((PWM_2ms-PWM_1ms)/2)+PWM_1ms)
    DCW (((PWM_2ms-PWM_1ms)/4)+PWM_1ms)
    DCW  PWM_1ms
    
;>>>>>   end constants here <<<<<
;**********************************************************************
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
Bufferstring SPACE	MAX_BUFFER
	
;>>>>>   end variables here <<<<<
            END