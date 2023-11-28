; adapted from the generic Cortex-M4 startup file 
;    in the standard Keil software pack
; code area can go at the Reset_Handler part or at the end
; data area goes at the end of the file, just before the "end" directive


;/**************************************************************************//**
; * @file     startup_ARMCM4.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM4 Device Series
; * @version  V1.08
; * @date     23. November 2012
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2011 - 2012 ARM LIMITED
;
;   All rights reserved.
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions are met:
;   - Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;   - Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;   - Neither the name of ARM nor the names of its contributors may be used
;     to endorse or promote products derived from this software without
;     specific prior written permission.
;   *
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
;   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;   ---------------------------------------------------------------------------*/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000C00

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     WDT_IRQHandler            ;  0:  Watchdog Timer
                DCD     RTC_IRQHandler            ;  1:  Real Time Clock
                DCD     TIM0_IRQHandler           ;  2:  Timer0 / Timer1
                DCD     TIM2_IRQHandler           ;  3:  Timer2 / Timer3
                DCD     MCIA_IRQHandler           ;  4:  MCIa
                DCD     MCIB_IRQHandler           ;  5:  MCIb
                DCD     UART0_IRQHandler          ;  6:  UART0 - DUT FPGA
                DCD     UART1_IRQHandler          ;  7:  UART1 - DUT FPGA
                DCD     UART2_IRQHandler          ;  8:  UART2 - DUT FPGA
                DCD     UART4_IRQHandler          ;  9:  UART4 - not connected
                DCD     AACI_IRQHandler           ; 10: AACI / AC97
                DCD     CLCD_IRQHandler           ; 11: CLCD Combined Interrupt
                DCD     ENET_IRQHandler           ; 12: Ethernet
                DCD     USBDC_IRQHandler          ; 13: USB Device
                DCD     USBHC_IRQHandler          ; 14: USB Host Controller
                DCD     CHLCD_IRQHandler          ; 15: Character LCD
                DCD     FLEXRAY_IRQHandler        ; 16: Flexray
                DCD     CAN_IRQHandler            ; 17: CAN
                DCD     LIN_IRQHandler            ; 18: LIN
                DCD     I2C_IRQHandler            ; 19: I2C ADC/DAC
                DCD     0                         ; 20: Reserved
                DCD     0                         ; 21: Reserved
                DCD     0                         ; 22: Reserved
                DCD     0                         ; 23: Reserved
                DCD     0                         ; 24: Reserved
                DCD     0                         ; 25: Reserved
                DCD     0                         ; 26: Reserved
                DCD     0                         ; 27: Reserved
                DCD     CPU_CLCD_IRQHandler       ; 28: Reserved - CPU FPGA CLCD
                DCD     0                         ; 29: Reserved - CPU FPGA
                DCD     UART3_IRQHandler          ; 30: UART3    - CPU FPGA
                DCD     SPI_IRQHandler            ; 31: SPI Touchscreen - CPU FPGA
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

;Reset_Handler   PROC
                ;EXPORT  Reset_Handler             [WEAK]
                ;IMPORT  SystemInit
                ;IMPORT  __main
                ;LDR     R0, =SystemInit
                ;BLX     R0
                ;LDR     R0, =__main
                ;BX      R0
                ;ENDP
				

; &&&&&&& code could go here if no read/write data area
; &&&&&&&    but if read/write data area, then place at end


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  TIM0_IRQHandler           [WEAK]
                EXPORT  TIM2_IRQHandler           [WEAK]
                EXPORT  MCIA_IRQHandler           [WEAK]
                EXPORT  MCIB_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  UART3_IRQHandler          [WEAK]
                EXPORT  UART4_IRQHandler          [WEAK]
                EXPORT  AACI_IRQHandler           [WEAK]
                EXPORT  CLCD_IRQHandler           [WEAK]
                EXPORT  ENET_IRQHandler           [WEAK]
                EXPORT  USBDC_IRQHandler          [WEAK]
                EXPORT  USBHC_IRQHandler          [WEAK]
                EXPORT  CHLCD_IRQHandler          [WEAK]
                EXPORT  FLEXRAY_IRQHandler        [WEAK]
                EXPORT  CAN_IRQHandler            [WEAK]
                EXPORT  LIN_IRQHandler            [WEAK]
                EXPORT  I2C_IRQHandler            [WEAK]
                EXPORT  CPU_CLCD_IRQHandler       [WEAK]
                EXPORT  SPI_IRQHandler            [WEAK]

WDT_IRQHandler
RTC_IRQHandler
TIM0_IRQHandler
TIM2_IRQHandler
MCIA_IRQHandler
MCIB_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
UART4_IRQHandler
AACI_IRQHandler
CLCD_IRQHandler
ENET_IRQHandler
USBDC_IRQHandler
USBHC_IRQHandler
CHLCD_IRQHandler
FLEXRAY_IRQHandler
CAN_IRQHandler
LIN_IRQHandler
I2C_IRQHandler
CPU_CLCD_IRQHandler
SPI_IRQHandler
                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF




;     global reset_handler
        EXPORT  Reset_Handler

; &&&&&&& insert code "AREA" specification here
	AREA FINALPROJECT, CODE, READONLY
	 ENTRY

	 
Reset_Handler

; &&&&&&& insert body of code here
; &&&&&&& insert data AREA, if any, after code AREA

MAIN 

SEQUENCE RN 2
	MOV SEQUENCE, #2
CONFIGURE RN 12
	LDR CONFIGURE, =CONFIG
STOREDATA RN 11
	LDR STOREDATA, =DATASTORE
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
			    ; SET GPIO A TO OUTPUTS
		
	LDR R3, =GPIO_A_START 	;	LET R3 -> GPIO A MODER (IN KEIL)
	LDR R3, [R3]			; 	LET R3 -> GPIO A MODER 
	LDR R4, [R3]		 	; 	R4 IS CURRENT VAL OF GPIO A MODER
	LDR R5, =0X1FF			;   R5 = 0...111111111 (9 ONES)
	ORR R4, R4, R5 			; 	PA0 - PA8 -> 1 (OUTPUT)
	STR R3, [R4]			;   STORE BACK TO GPIO A MODER	
	
	STR R4, [CONFIGURE], #4	; 	STORE OUTPUT PIN SETTINGS TO CONFIG
	
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		   ; SET GPIO B TO USER SWITCH INPUT
		
	LDR R3, =GPIO_B_START	; LET R3 -> GPIO B START
	LDR R3, [R3]			; 	LET R3 -> GPIO B MODER
	LDR R4, [R3]			; R3 IS CURRENT VAL OF GPIO B MODER
	BIC R4, R4, #0X1		; CLEAR PC 0 -> 0 (INPUT)
	STR R4, [R3]			; STORE PIN CONFIG BACK TO GPIO B MODER
	STR R4, [CONFIGURE], #4	; 	STORE OUTPUT PIN SETTINGS TO CONFIG
	
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		   ; CONFIGURE INPUT PIN HARDWARE
		
	LDR R3, =GPIO_B_START	; LET R3 -> GPIO B START
	LDR R3, [R3]			; 	LET R3 -> GPIO B MODER
	ADD R3, R3, #0X0C		; R3 += PUPDR OFFSET 
	LDR R4, [R3]			; R4 IS CURRENT VAL OF R3
	BIC R4, R4, #0X0		; CLEAR PB0
	ORR R4, R4, #0X1		; SET PB0 -> 1 (PULL DOWN)
	STR R4, [R3]			; STORE HARDWARE CONFIG TO GPIOB PUPDR
	STR R4, [CONFIGURE], #4	; 	STORE OUTPUT PIN SETTINGS TO CONFIG
	ADD STOREDATA, STOREDATA, #12
	
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	B INFINITE	
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;				1	2	3	
;				4	5	6
;				7	8	9
RFWD ; (1,2,4) -> (3,5,7) -> (6,8,9)
	PUSH{R3,R6,R7,LR}
	LDR R6, =GPIO_A_START		; R6 -> GPIO_A_START
	LDR R6, [R6]				; LET R6 -> GPIO A START
	ADD R6, R6, #0X14			; R6 -> GPIO_A_DATAOUT
	MOV R7, #2_000001011		; R7 = (1,2,4)
	STR R7, [R6]				; STORE R7 TO OUTPUT DATA
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT 
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_001010100		; R7 = (3,5,7)
	STR R7, [R6]				; STORE (3,5,7) IN DATA OUT
	STR R7, [STOREDATA], #4			; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT_1 
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT_1					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_110100000		; R7 = (6,8,9)
	STR R7, [R6]				; STORE (6,8,9) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	BL LBCK
	POP{R3,R6,R7,PC}

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;				1	2	3	
;				4	5	6
;				7	8	9
LBCK ; (6,8,9) -> (3,5,7) -> (1,2,4)
	PUSH{R3,R6,R7,LR}
	LDR R6, =GPIO_A_START		; R6 -> GPIO_A_START
	LDR R6, [R6]				; LET R6 -> GPIO A START
	ADD R6, R6, #0X14			; R6 -> GPIO_A_DATAOUT
	MOV R7, #2_110100000		; R7 = (6,8,9)
	STR R7, [R6]				; STORE R7 TO OUTPUT DATA
	LDR R3, =DATASTORE			; R3 -> DATASTORE
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT_2
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT_2					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_001010100		; R7 = (3,5,7)
	STR R7, [R6]				; STORE (3,5,7) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT_3
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT_3					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_000001011    	; R7 = (1,2,4)
	STR R7, [R6]				; STORE (1,2,4) IN DATA OUT
	STR R7, [STOREDATA], #4		; R3 -> DATASTORE
	
	POP{R3,R6,R7,PC}

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;				1	2	3	
;				4	5	6
;				7	8	9	
RIGHT ; (1,4,7) -> (2,5,8) -> (3,6,9)
	PUSH{R3,R6,R7,LR}
	LDR R6, =GPIO_A_START		; R6 -> GPIO_A_START
	LDR R6, [R6]				; LET R6 -> GPIO A START
	ADD R6, R6, #0X14			; R6 -> GPIO_A_DATAOUT
	MOV R7, #2_001001010		; R7 = (1,4,7)
	STR R7, [R6]				; STORE R7 TO OUTPUT DATA
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
; WAIT_4
	; SUBS R7, R7, #1				; DECREMENT FOR TIMER
	; BNE WAIT_4					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_010010010		; R7 = (2,5,8)
	STR R7, [R6]				; STORE (2,5,8) IN DATA OUT
	STR R7, [STOREDATA], #4			; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
; WAIT_5
	; SUBS R7, R7, #1				; DECREMENT FOR TIMER
	; BNE WAIT_5					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_100100100		; R7 = (3,6,9)
	STR R7, [R6]				; STORE (3,6,9) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	BL LEFT
	POP{R3,R6,R7,PC}

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;				1	2	3	
;				4	5	6
;				7	8	9
LEFT ; (3,6,9) -> (2,5,8) -> (1,4,7)
	PUSH{R3,R6,R7,LR}
	LDR R6, =GPIO_A_START		; R6 -> GPIO_A_START
	LDR R6, [R6]				; LET R6 -> GPIO A START
	ADD R6, R6, #0X14			; R6 -> GPIO_A_DATAOUT
	MOV R7, #2_100100100		; R7 = (3,6,9)
	STR R7, [R6]				; STORE R7 TO OUTPUT DATA
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
; WAIT_6
	; SUBS R7, R7, #1				; DECREMENT FOR TIMER
	; BNE WAIT_6					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_010010010		; R7 = (2,5,8)
	STR R7, [R6]				; STORE (2,5,8) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
; WAIT_7
	; SUBS R7, R7, #1				; DECREMENT FOR TIMER
	; BNE WAIT_7					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_001001010		; R7 = (1,4,7)
	STR R7, [R6]				; STORE (1,4,7) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	POP{R3,R6,R7,PC}

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;				1	2	3	
;				4	5	6
;				7	8	9
UP   ; (7,8,9) -> (4,5,6) -> (1,2,3)
	PUSH{R3,R6,R7,LR}
	LDR R6, =GPIO_A_START		; R6 -> GPIO_A_START
	LDR R6, [R6]				; LET R6 -> GPIO A START
	ADD R6, R6, #0X14			; R6 -> GPIO_A_DATAOUT
	MOV R7, #2_111000000		; R7 = (7,8,9)
	STR R7, [R6]				; STORE R7 TO OUTPUT DATA
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT_8
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT_8					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_000111000		; R7 = (4,5,6)
	STR R7, [R6]				; STORE (4,5,6) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT_9
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT_9					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_000000111		; R7 = (1,2,3)
	STR R7, [R6]				; STORE (1,2,3) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	BL DOWN
	POP{R3,R6,R7,PC}

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;				1	2	3	
;				4	5	6
;				7	8	9
DOWN   ; (1,2,3) -> (4,5,6) -> (7,8,9)
	PUSH{R3,R6,R7,LR}
	LDR R6, =GPIO_A_START		; R6 -> GPIO_A_START
	LDR R6, [R6]				; LET R6 -> GPIO A START
	ADD R6, R6, #0X14			; R6 -> GPIO_A_DATAOUT
	MOV R7, #2_000000111		; R7 = (1,2,3)
	STR R7, [R6]				; STORE R7 TO OUTPUT DATA
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT_10
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT_10					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_000111000		; R7 = (4,5,6)
	STR R7, [R6]				; STORE (4,5,6) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	LDR R7, =0XABACADAB			; CREATE LARGE NUMBER
WAIT_11
	SUBS R7, R7, #1				; DECREMENT FOR TIMER
	BNE WAIT_11					; WHILE NOT 0 -> WAIT
	
	MOV R7, #2_111000000		; R7 = (7,8,9)
	STR R7, [R6]				; STORE (7,8,9) IN DATA OUT
	STR R7, [STOREDATA], #4		; STORE R7 IN DATASTORE
	
	POP{R3,R6,R7,PC}
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
			; CHANGE THE PATTERN BEING DISPLAYED
CHANGE
	SUBS SEQUENCE, SEQUENCE, #1 	; SEQUENCE--
	IT EQ							; IF = 0, RESET TO 3
	MOVEQ SEQUENCE, #3				; SET SEQUENCE BACK TO 3
	LDR R3, =DATASTORE				; R3 -> DATASTORE
	STR SEQUENCE, [R3], #4			; STORE SEQUENCE IN DATASTORE
	BL PATTERN						; BRANCH AND LINK TO PATTERN
	B INFINITE						; UNCONDITIONAL TO INFINITE

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
				; SELECT PATTERN 
PATTERN
	BEQ RFWD						; IF EQUAL, SEQ = 3
	CMP SEQUENCE, #2				; CHECK SECOND CASE
	BEQ RIGHT						; IF EQUAL, SEQUENCE = 2
	B UP							; ELSE, SEQUENCE = 1


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
INFINITE 
	LDR R3, =GPIO_B_START		; R3 -> GPIO B
	ADD R3, R3, #0XC10			; R3 += OFFSET TO INPUT DATA REG
	LDR R4, [R3]				; R4 = CURRENT VAL OF INPUT DATA REG
	AND R5, R4, #0X1			; TEST PB0 IN
	CMP R5, #1					; IF 1, BUTTON IS BEING PRESSED
	BEQ CHANGE					; CHANGE THE PATTERN
	BL PATTERN					; B & L TO PATTERN
	B INFINITE					; UNCONDITIONAL BACK TO INFINITE

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
STOP B STOP

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


SRAMSTART 			DCD 0X20000000 ; DATA STORAGE START
GPIO_A_START 		DCD 0X40000000 ; START OF GPIO A / MODER ADDRESS
GPIO_B_START 		DCD 0X40000400 ; START OF GPIO B / MODER ADDRESS
GPIO_C_START		DCD 0X40000800 ; START OF GPIO C / MODER ADDRESS
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	AREA STORAGE, DATA, READWRITE

CONFIG    DCD 0
DATASTORE DCD 0


     END



; 														   NOTES: 

; 								SRAM START: 										0X20000000
; 								PERIPHERALS: 										0X40000000


; 								GPIO A: 		  							  0x40000000-0X4000003FF
; 								GPIO A MODER:								    GPIO_A_START + 0X0
; 								GPIO A OUTPUT DATA: 							GPIO_A_START + 0X14

; 								GPIO B: 		  							  0x40000400-0X4000007FF
; 								GPIO B MODER: 								    GPIO_B_START + 0X0
; 								GPIO B PUPDR: 								    GPIO_B_START + 0X0C
; 								GPIO B INPUT DATA REG							GPIO_B_START + 0XC10

; 								GPIO C: 									  0x4800 0800 - 0x4800 0BFF
; 								GPIO C MODER: 								    GPIO_C_START + 0X0
; 								GPIO C PUPDR: 								    GPIO_C_START + 0X0C




