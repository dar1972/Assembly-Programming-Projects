		TTL Exercise 8
;****************************************************************
;The program adds two numbers and displays the output
;Name:  Dhruv Rajpurohit
;Date:  26 March 2020
;Class:  CMPE-250
;Section: 01L1, Thursday 2:00 - 3:50
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
;---------------------------------------------------------------
;Characters
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
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
;---------------------------------------------------------------
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
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
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
UART0_C2_T_R  EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
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
;---------------------------------------------------------------
 
MAX_BUFFER     EQU     128
MAX_STRING     EQU     17   
NUM_WORDS	   EQU		4			;Number of Words
;---------------------------------------------------------------
;-------------------------------------------------------------
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
            BL      Startup				;This subroutine calls SetClock48MHZ subroutine which configures MCG to be used.      
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
            BL      INIT_UART0_POLLING	; iniatilize UART0
first
Num1
            MOVS    R0,#32				
            BL      PutChar
            LDR     R0,=entry1	; "Enter first 128-bit hex number:   0x"
            BL      PutStringSB			; displays the string in memory
            MOVS    R1,#NUM_WORDS
            LDR     R0,=Number1
            BL      GetHexIntMulti		; branch to GetHexIntMulti
check1
            BCS     NumberInvalid1		
            
Num2
            LDR     R0,=entry2	;"Enter 128-bit hex number to add:   0x"
            BL      PutStringSB
            MOVS    R1,#NUM_WORDS
            LDR     R0,=Number2
            BL      GetHexIntMulti		;branches to GetHexIntMulti
check2
			BCS     NumberInvalid2		
            
;Addition of the Two nNumbers
            MOVS    R3,#NUM_WORDS		
            LDR     R1,=Number1			; load Address of number1 in R1
            LDR     R2,=Number2			; load Address of number in R2
            LDR     R0,=Addition		; load Address of buffer of the sum
            BL      AddIntMultiU		; branch to AddIntMultiU
            BCS     Overflow1			;if Carry is set then overflow
            LDR     R0,=Sum				;  "Sum:  0x"
            BL      PutStringSB
            LDR     R0,=Addition		
            MOVS    R1,#NUM_WORDS
            BL      PutHexIntMulti		
            BL      NewLine
            B       first				; branch backs to first
Overflow1
            LDR     R0,=Sum
            BL      PutStringSB
            LDR     R0,=Overflow		; "OVERFLOW"
            BL      PutStringSB
            BL      NewLine
            B       first				; branch baks to first
			
NumberInvalid1
            BL      Invalid				;  "Invalid number--try again: 		      0x"
			BL		GetHexIntMulti		; branches to GetHexIntMulti
			B		check1				; branch to check1
			
NumberInvalid2
            BL      Invalid
			BL		GetHexIntMulti
			B		check2				
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
Invalid     PROC    {R0-R14}

            PUSH    {R0-R1,LR}
            LDR     R0,=Invalid1
            BL      PutStringSB
            POP     {R0-R1,PC}
            ENDP

            
AddIntMultiU 	PROC	{R0-R14}


			PUSH 	{R0-R7}			
			MOVS	R5,#0			
			LDR		R6,[R2,#0]		
			LDR		R4,[R1,#0]		

			PUSH	{R0-R1}
			MRS		R0,APSR
			MOVS	R1,#20
			LSLS	R1,R1,#24
			BICS	R0,R0,R1
			MSR		APSR,R0
			POP		{R0-R1}

AdditionC		
			ADCS	R6,R6,R4		

			MRS		R7,APSR			
			STR		R6,[R0,#0]		
			ADDS	R2,R2,#4		
			ADDS	R1,R1,#4		
			ADDS	R0,R0,#4		
			ADDS	R5,R5,#1		
			CMP		R3,R5			
			BEQ		done		
			LDR		R6,[R2,#0]		
			LDR		R4,[R1,#0]		
			MSR		APSR,R7			
			B		AdditionC		
done
			MSR		APSR,R7			


			POP		{R0-R7}			
			BX		LR
			ENDP
		 		
GetHexIntMulti	PROC	{R0-R14}
 
			
			PUSH	{R0-R7,LR}
            MOVS    R7,R0               
			MOVS 	R5,#0               
			MOVS	R6,R1               
			MOVS	R2,#8               
			MULS	R1,R2,R1            
			ADDS	R1,R1,#1		    
			LDR		R0,=Stringbuffer    
			BL		GetStringSB			
            ADDS    R1,R0,R1            
            SUBS    R1,R1,#1            
			
next	
            LDRB	R3,[R0,#0]		    
            CMP		R0,R1			    
			BEQ		conversiondone 	    

            CMP     R3,#'0'
            BLO     invalid
            CMP     R3,#'9'				
            BHI     alphacheck		
            B       convert		

alphacheck
            CMP     R3,#'A'
            BLO     invalid
            CMP     R3,#'z'
            BHI     invalid    
			B		Alpha

Alpha
			CMP		R3,#'a'
			BHS		lowercase1		
			CMP		R3,#'Z'
			BLS		uppercase1		

uppercase1
			CMP		R3,#'F'			
			BHI		invalid			
			SUBS	R3,R3,#55		
			STRB	R3,[R0,#0]		
			B		nextbuffer		
lowercase1
			CMP		R3,#'f'			
			BHI		invalid		
			SUBS	R3,R3,#87		
			STRB	R3,[R0,#0]		
			B		nextbuffer		

convert
			SUBS	R3,R3,#48       
			STRB	R3,[R0,#0]      
            B       nextbuffer		
nextbuffer	
			ADDS	R0,R0,#1        
            ADDS    R5,R5,#1        
			B		next			

conversiondone

            MOVS    R2,R1			
            SUBS    R2,R2,#1		
            LDR     R5,=Stringbuffer
packing
            CMP     R5,R2			
			BHI     packingdone		
			LDRB    R3,[R2,#0]		

            SUBS    R2,R2,#1		
            LDRB    R4,[R2,#0]		
            LSLS    R4,R4,#4		
            ORRS    R3,R3,R4		
            STRB    R3,[R7,#0]		
            ADDS    R7,R7,#1		
			SUBS	R2,R2,#1		
            B       packing

packingdone
valid    

            MRS		R0,APSR			
			MOVS	R1,#0x20		
			LSLS	R1,R1,#24		
			BICS	R0,R0,R1		
			MSR		APSR,R0			
            B       GHIMdone

invalid  
			MRS		R0,APSR			
			MOVS 	R1,#0x20		
			LSLS	R1,R1,#24		
			ORRS	R0,R0,R1		
			MSR		APSR,R0			
GHIMdone                
            POP		{R0-R7,PC}
            ENDP
                
  
PutHexIntMulti	PROC {R0-R14}

	
			PUSH	{R0-R5,LR} 
			MOVS	R2,#0		
			MOVS	R4,R0		

			MOVS	R5,R1		
			LSLS	R1,#1	
			ADDS	R4,R4,R1
			ADDS	R4,R4,R5	
loadword
			LDR 	R3,[R4,#0]	
			MOVS	R0,R3		
			BL		PutNumHex	
			ADDS	R2,R2,#1	
			CMP		R2,R5		
			BEQ		alldone
			SUBS	R4,R4,#4	
			B		loadword	
alldone
			POP		{R0-R5,PC} 
			ENDP
                
NewLine      PROC  {R0-R15}
           PUSH    {R0,LR}
     	   MOVS    R0,#CR
           BL      PutChar
           MOVS    R0,#LF
           BL      PutChar
           POP     {R0,PC}
           ENDP
                
            
INIT_UART0_POLLING      PROC    {R0-R15}	


            PUSH {R1-R3}					
			LDR R1,=SIM_SOPT2				
			LDR R2,=SIM_SOPT2_UART0SRC_MASK
			LDR R3,[R1,#0]
			BICS R3,R3,R2
			LDR R2,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS R3,R3,R2
			STR R3,[R1,#0]
			LDR R1,=SIM_SOPT5
			LDR R2,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR R3,[R1,#0]
			BICS R3,R3,R2
			STR R3,[R1,#0]

			LDR R1,=SIM_SCGC4
			LDR R2,= SIM_SCGC4_UART0_MASK
			LDR R3,[R1,#0]
			ORRS R3,R3,R2
			STR R3,[R1,#0]

			LDR R1,=SIM_SCGC5
			LDR R2,= SIM_SCGC5_PORTA_MASK
			LDR R3,[R1,#0]
			ORRS R3,R3,R2
			STR R3,[R1,#0]

			LDR R1,=PORTA_PCR1
			LDR R2,=PORT_PCR_SET_PTA1_UART0_RX
			STR R2,[R1,#0]

			LDR R1,=PORTA_PCR2
			LDR R2,=PORT_PCR_SET_PTA2_UART0_TX
			STR R2,[R1,#0]

			LDR R1,=UART0_BASE
			MOVS R2,#UART0_C2_T_R
			LDRB R3,[R1,#UART0_C2_OFFSET]
			BICS R3,R3,R2
			STRB R3,[R1,#UART0_C2_OFFSET]

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

			MOVS R2,#UART0_C2_T_R
			STRB R2,[R1,#UART0_C2_OFFSET] 
            		POP  {R1-R3}
           		BX     LR
            		ENDP


GetChar     PROC {R1-R15}

            PUSH {R2-R4}			

            LDR R2,=UART0_BASE
            MOVS R3,#UART0_S1_RDRF_MASK
PollRx      LDRB R4,[R2,#UART0_S1_OFFSET]
            ANDS  R4,R4,R3
           	BEQ PollRx				

            LDRB R0,[R2,#UART0_D_OFFSET] 
            POP   {R2-R4}
            BX     LR				
            ENDP
 
PutChar     PROC {R1-R15}

            PUSH  {R2-R4}						

			LDR R2,=UART0_BASE
			MOVS R3,#UART0_S1_TDRE_MASK
PollTx		LDRB R4,[R2,#UART0_S1_OFFSET]
			ANDS R4,R4,R3
			BEQ PollTx				
	
			STRB R0,[R2,#UART0_D_OFFSET]		
            POP {R2-R4}							
            BX  LR								
            ENDP
                
GetStringSB		PROC {R0-R13},{}

			PUSH    {R0-R3,LR}		
			MOVS	R3,#0			
			MOVS	R4,R0			
			SUBS	R1,R1,#1		
			BL		GetChar			
While
			CMP 	R0,#CR			
			BEQ 	nextline		
			ADDS	R3,R3,#1		
			CMP		R3,R1			
			BHI		EndWhile		
			BL		PutChar			
			STRB	R0,[R4,#0]		
			ADDS	R4,R4,#1		
			BL		GetChar			
			B		While			
EndWhile							
			BL		GetChar			
			CMP		R0,#CR			
			BEQ		nextline		
			B		EndWhile		
nextline
			MOVS	R0,#NULL		
			BL		PutChar			
			STRB	R0,[R4,#0]		
			MOVS	R0,#CR			
			BL		PutChar	
			MOVS	R0,#LF
			BL		PutChar			
			POP		{R0-R3,PC}		
			BX		LR				
			ENDP

PutStringSB PROC {R0-R13},{}

			PUSH	{R1-R4,LR}		
			MOVS 	R4,#0			
            MOVS    R1, #79
Whileloop		LDRB 	R2,[R0,#0]	
			CMP	 	R2,#NULL		
			BEQ	 	EndWhileLoop	
			ADDS 	R4,R4,#1		
			CMP	 	R4,R1			
			BHS	 	EndWhileLoop	
			PUSH 	{R0}			
			MOVS 	R0,R2			
			BL	 	PutChar			
			POP		{R0}			
			ADDS 	R0,R0,#1		
			B		Whileloop		
EndWhileLoop
			POP		{R1-R4,PC}		
			BX		LR				
			ENDP


PutNumHex   PROC   {R0-R15}

            PUSH   {R1-R4,LR}	
            MOVS    R3,#0			
            MOVS    R2,#0x0000000F	
            MOVS    R1,R0			
            MOVS    R4,R0			
shifted   
           	CMP     R3,#8			
            BEQ     printhex		
            ANDS    R4,R4,R2		
            MOVS    R0,R4			
            ADDS    R3,R3,#1		
            CMP     R0,#9			
            BGT     alpha		
            ADDS    R0,R0,#48	
            B       pushstack	
alpha    ADDS    R0,R0,#55		
            B       pushstack	
pushstack
            PUSH{R0}			
            LSRS    R1,R1,#4	
            MOVS    R4,R1		
            B       shifted		
            
printhex       
           	CMP     R3,#0			
            BEQ     printfin		
            POP     {R0}			
           	BL      PutChar			
            SUBS    R3,R3,#1		
           	B       printhex		
printfin
            POP     {R1-R4,PC}		
            BX      LR
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
            DCD    Dummy_Handler      ;28:UART0 (status; error)
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
entry1     DCB     "Enter first 128-bit hex number:   0x",NULL
entry2     DCB     "Enter 128-bit hex number to add:   0x",NULL
Sum              DCB     "                            Sum:   0x",NULL
Invalid1         DCB     "Invalid number--try again: 		      0x",NULL
Overflow         DCB     "OVERFLOW",NULL

;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
Number1     SPACE       MAX_BUFFER
Number2     SPACE       MAX_BUFFER
Addition    SPACE		MAX_BUFFER	
Stringbuffer SPACE       MAX_STRING 
Storebuffer SPACE       16    
;>>>>>   end variables here <<<<<
            ALIGN
            END