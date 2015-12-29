;**********************************************************************
;   This file is a basic code template for assembly code generation   *
;   on the PICmicro PIC16F876. This file contains the basic code      *
;   building blocks to build upon.                                    *  
;                                                                     *
;   If interrupts are not used all code presented between the ORG     *
;   0x004 directive and the label main can be removed. In addition    *
;   the variable assignments for 'w_temp' and 'status_temp' can       *
;   be removed.                                                       *                         
;                                                                     *
;   Refer to the MPASM User's Guide for additional  information on     *
;   features of the assembler (Document DS33014).                     *
;                                                                     *
;   Refer to the respective PICmicro data sheet for additional        *
;   information on the instruction set.                               *
;                                                                     *
;   Template file assembled with MPLAB V4.00 and MPASM V2.20.00.      *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Filename:	    xxx.asm                                           *
;    Date:                                                            *
;    File Version:                                                    *
;                                                                     *
;    Author:                                                          *
;    Company:                                                         *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files required:                                                  *
;                                                                     *
;                                                                     *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes:  things to remember                                       *
;   															      *
;   *---WHEN DOING EEPROM ROUTINE DONT FORGET TO CHANGE BANK TO 0     *
;   *---                                                              *
;                                                                     *
;**********************************************************************


	list      p=16f876            ; list directive to define processor
	#include <p16f876.inc>        ; processor specific variable definitions
	
	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_OFF & _HS_OSC & _WRT_ENABLE_ON & _LVP_ON & _DEBUG_ON & _CPD_OFF 

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.






;***** VARIABLE DEFINITIONS
;**  I / O PIN ASSIGNMENTS

					; (1= IN, 0= OUT)		PORT  BIT IN/OUT STATE	NORMALLY
					;---------------------------------------------------------------
VIDSTB		EQU	0		;6450 VIDEO STROBE	   	PORTA, 0, OUTPUT  0	LOW
BUSY		EQU	1		;VIDEO BUSY INDICATOR	   	PORTA, 1, INPUT   1	LOW
RAMCS		EQU	2		;EEPROM CHIP SELECT	   	PORTA, 2, OUTPUT  0	LOW
KEY_UP          EQU	3		;KEYPAD, MENU UP	   	PORTA, 3, INPUT   1	HIGH
KEY_DOWN	EQU	4		;KEYPAD, MENU DOWN	   	PORTA, 4, INPUT   1	HIGH
KEY_ENTER	EQU	5		;KEYPAD, ENTER		   	PORTA, 5, INPUT   1	HIGH

VERT_VGA	EQU	0		;VGA VERTICLE SYNCH	   	PORTB, 0, INPUT   1	TRISTATE
DS_LATCH	EQU	1		;74595 DESERIALIZER LATCH	PORTB, 1, OUTPUT  0	LOW
FS_LCD		EQU	2		;LCD FRAME SELECT		PORTB, 2, OUTPUT  0	LOW (?)
CD_LCD		EQU	3		;LCD C/D			PORTB, 3, OUTPUT  0	LOW (?)
EN_LCD		EQU	4		;LCD ENABLE			PORTB, 4, OUTPUT  0	LOW
WR_LCD		EQU	5		;LCD WRITE			PORTB, 5, OUTPUT  0	LOW (?)
PROG6		EQU	6		;PROGRAMMER PORT		PORTB, 6, INPUT   1	TRISTATE
PROG7   	EQU	7		;PROGRAMMER PORT		PORTB, 7, INPUT	  1	TRISTATE

FIELD		EQU	0		;LM1881 FIELD INDICATOR		PORTC, 0, INPUT   1	TRISTATE
SYNC_VGA	EQU	1		;VGA VIDEO LINE SYNC TO OSCOPE	PORTC, 1, OUTPUT  0	LOW
HORZ_VGA	EQU	2		;VGA HORIZONTAL SYNCH		PORTC, 2, INPUT   1	TRISTATE
CLOCK		EQU	3		;SERIAL CLOCK 			PORTC, 3, OUTPUT  0	HIGH
DI		EQU	4		;SERIAL DATA INPUT		PORTC, 4, INPUT	  1	TRISTATE
DO		EQU	5		;SERIAL DATA OUTPUT		PORTC, 5, OUTPUT  0	HIGH
TXD		EQU	6		;SERIAL TXMT TO RS-232		PORTC, 6, OUTPUT  0	HIGH (?)
RXD		EQU     7		;SERIAL RXCV FROM RS-232	PORTC, 7, INPUT   1	HIGH (?)




;*******************************************************************************************************
;*******************************************************************************************************
;**  DEFINE VARIABLE REGISTERS
;**  NOTE:  I make it a habit to number each register.  There are 96 possible
;**  registers in a PIC16F876.


;** Interrupt registers:
w_temp        	EQU     0x70        	; 1) 	variable used for context saving 
status_temp   	EQU     0x71        	; 2) 	variable used for context saving

;** DELAY registers:
DTOTAL		EQU	0x20		; 3) 	DELAY
DCOUNT1		EQU	0x21		; 4) 	NESTED LOOP LEVEL 1
DCOUNT2		EQU	0x22		; 5) 	NESTED LOOP LEVEL 2

;** 6460 Video Overlay registers:
VID_DATA	EQU	0x23		; 6)  	8 BITS OF VIDEO DATA
VID_COUNT	EQU	0x24		; 7)  	VIDEO COUNT MARKER
VID_COUNT2	EQU	0x25		; 8)   	VIDEO COUNT MARKER
VID_COUNT3	EQU	0X50		; X)	XTRA CONSTANT *******-----******------@@@@@@@@@@
VID_LINE	EQU	0x26		;10)	USED TO INDICATE LINE 0-11 (0x00 TO 0x0B)
VID_COL		EQU	0x27		;11)	USED TO INDICATE COLUMN 0-23 (0x00 TO 0x17)


;** Field Counter registers:
FFIELDS		EQU	0x28		;12) 	USED TO HOLD THE DISPLAY DIGITS FROM THE LOOKUP TABLE
FCOUNTL		EQU	0x29		;13) 	USED TO HOLD THE FIELD ONES DIGIT (0-5)
FCOUNTH		EQU	0x2A		;13) 	USED TO HOLD THE FIELD TENS DIGIT (0-9)
FSECONDSL	EQU	0x2B		;14) 	USED TO HOLD THE SECOND ONES DIGIT (0-9)
FSECONDSH	EQU	0x2C		;15) 	USED TO HOLD THE SECOND TENS DIGIT (0-5)
FMINUTESL	EQU	0x2D		;16) 	USED TO HOLD THE MINUTE ONES DIGIT (0-9)
FMINUTESH	EQU	0x2E		;15) 	USED TO HOLD THE MINUTE TENS DIGIT (0-5)
FHOURSL		EQU	0x2F		;16) 	USED TO HOLD THE HOUR ONES DIGIT (0-9)
FHOURSH		EQU	0x30		;17) 	USED TO HOLD THE HOUR TENS DIGIT (0-9)
OEDETECT	EQU 0X31		;X)		THIS WILL STORE ODD OR EVEN FIELD BIT IN ITS 0 BIT
							;		XXXX XXXD		D=0 OR 1
							; 		I AM ONLY LOOKING AT BIT 0 OF THIS REG
								
;** GENERAL DATA AND ADDRESS registers:
;** NOTE:  These registers are used for the PIC Data EEPROM, for the Serial EEPROM (93LC56)
;** and for the serial port routine.
DATA_BYTE	EQU	0x32		;18)	DATA FOR EEPROM
DATA_ADDR	EQU	0x33		;19)	ADDRESS FOR EEPROM.  (0 TO FF ADDRESS LOCATIONS)

;** EEPROM registers:
DATA_ADDR_END	EQU	0x34		;20)	END ADDRESS FOR EEPROM (0 TO FF)
EE_CLOCKS	EQU	0x35		;21)	USED BY THE SERIAL EEPROM TO DETERMINE CLOCK COUNT


P_DIR		EQU	0x36		;22)	USED TO INDICATE DIRECTION OF MEMORY 2 OUTPUT WRITE

;** MISC registers:
DELAY_1		EQU 	0X37		;23) 	DELAY 1 REG		\
DELAY_2		EQU	0X38		;23) 	DEALY 2 REG		 |
DELAY_0		EQU	0X39		;24)	DELAY 0 REG		 |------ALL OF THESE REGS ARE USED IN DELAY ROUTINE
DELAY_COUNT0	EQU 	0X3A		;25)	DELAY COUNTERS		 |------THATS 3 STAGE DELAY WITH PRECISION OF 200nS		
DELAY_COUNT1	EQU	0X3B		;26)	DELAY COUNTERS		 |-------CHECK OUR DELAY ROUTINE FOR MORE INFO
DELAY_COUNT2	EQU	0X3C		;27)	DELAY COUNTERS		/	 
GENERAL_COUNT	EQU	0x3D		;28)	GENERAL COUNT REGISTER
INTRO_COUNT	EQU	0x3E		;29)	INTRO ROUTINE COUNTER
GENERAL_COUNT2	EQU	0x3F		;30)    VERY TEMPORARY

STATE_DETECT	EQU	0x40		;31)	THIS IS STAGE DETECT REG WHICH WILL HOLD THE 
								;		CURRENT STAGE NUMBER
								;		STAGE-1=1  2=2.....
								;	 	I WILL USE THIS REG FOR KEY_PRESS ROUTINES TO DETERMINE WHICH STAGE TO GO
								;		WHEN KEY IS PRESSED
KEY_DETECT_C	EQU	0X41		;32)    AFTER 1mS DELAY CHECK  IF BOTH ARE SAME THATS CONFORMED KEY PRESS
								;		THIS REG HOLDS THE PORTA KEY PRESS BITS
								;		0000 0000
								;		00ED U000
								;		0X08==UP KEY    0X10==DONW KEY    OX20==ENTER KEY ,ELSE NO KEY==SAME STAGE
KEY_DETECT	EQU 0X42		; 		FIRST TIME CHECK				

;** UART registers:
RC_DATA		EQU	0X43		; THIS REG HOLDS BYTE FROM RECREG REG
RC_FLAG		EQU	0X44		; THIS IS A FLAG REG WHEN REG IS FULL SET IT,WHEN ITS EMPTY CLEAR IT
					; I MA USING ONLY 0 BIT TO TEST IT
CURSOR_COL	EQU 0X45		; THIS IS A WORDPROSESSOR NAVIGATION CURSOR---24 COLUMN
					; THIS WILL KEEP TRACK OF CURSOR IN MEMORY,HYPERTERMINAL AND NTSC
CURSOR_LINE	EQU 0X46		; LINE CURSUR---6 LINE 
					; SO HERE WE ARE IMPLEMENTING 24*6 MATRIX 		
HYPE_TEMP	EQU	0X47		; TEMP REG	
GENERAL_TEMP	EQU	0X48			
LOOKUP_REG	EQU	0X49		; REGISTER TO HOLD NEC LOOKUP VARIABLE	
SEC_INC_DETECT	EQU 0X4A		; THIS REG DETECTS THE SECOND INCREMENT AFTER 1-60 FRAMES IS DONE
;*******************************************************************************************************
;*******************************************************************************************************
;**  DEFINE CONSTANTS

KEY_MENU	EQU	2		;KEYPRESS 'MENU DISPLAYED' DATA BIT
KEY_VALID	EQU	3		;KEYPRESS 'REGISTER VALID' DATA BIT
KEYDELAY	EQU	0x1F		;BUTTON REACTION DELAY

;** SERIAL EEPROM (93LC56) CONSTANTS (OPCODES)
;** NOTE:  All of these constants start with SE_
;**
SE_ERASE	EQU	0xE0		; OPCODE TO ERASE A BYTE
SE_ERAL		EQU	0x90		; OPCODE TO ERASE ALL BYTES
SE_EWDS		EQU	0x80		; OPCODE TO DISABLE ERASE/WRITE
SE_EWEN		EQU	0x98		; OPCODE TO ENABLE ERASE/WRITE
SE_READ		EQU	0xC0		; OPCODE TO READ A BYTE
SE_WRITE	EQU	0xA0		; OPCODE TO WRITE A BYTE
SE_WRAL		EQU	0x88		; OPCODE TO WRITE ALL BYTES

;** WRITE FROM MEMORY TO OUTPUT (MEMORY_TO_OUTPUT_WR Routine) Opcodes.
;** NOTE:  All of these constants start with P_
;**
P_COMPLETE	EQU	0x00		; COMPLETE ROUTINE
P_PIC_EE	EQU	0x01		; INDICATE PIC EEPROM IS SOURCE
P_SERIAL_EE	EQU	0x02		; INDICATE SERIAL EEPROM IS SOURCE
P_NTSC		EQU	0x10		; DIRECT PRINT TO NTSC OUTPUT
P_SERIAL	EQU	0x20		; DIRECT PRINT TO SERIAL OUTPUT
P_LCD		EQU	0x40		; DIRECT PRINT TO LCD OUTPUT

;** VIDEO DISPLAY CONSTANTS
;** NOTE:  All of these constants start with V_
;**
V_FORMAT	EQU	0xFF		;6450 TEST MODE RELEASE, SELECT BANK 1
V_VERTPOS	EQU	0x43		;6450 DISPLAY VERTICAL POSITION
V_HORZPOS	EQU	0xC8		;6450 DISPLAY HORIZONTAL POSITION
V_CHARSIZ	EQU	0x8B		;6450 CHARACTER SIZE
V_BANK0		EQU	0xFC		;6450 BANK 0 SELECT
V_BANK1		EQU	0xFE		;6450 BANK 1 SELECT
V_MODE		EQU	0xF1		;6450 VIDEO MODE - CURRENTLY EXTERNAL / NTSC
V_COLOR		EQU	0xC2		;6450 VIDEO COLOR - CURRENTLY SET FOR NO BACKGROUND
					;  WITH A BLACK BACKGROUND COLOR (IF USED)
V_DISPLAY	EQU	0xE9		;6450 DISPLAY ON/OFF, BLINK ON/OFF
V_CHARLINE	EQU	0x94		;6450 LINE THAT HRS MNS SNDS FLDS IS DISPLAYED AT
V_NUMLINE	EQU	0x95		;6450 LINE THAT THE ACTUAL NUMBERS IS DISPLAYED AT


V_LINE		EQU	0x90		;6450 STARTING LINE
V_COLUMN	EQU	0xA0		;6450 STARTING COLUMN
					; Note - V_LINE and V_COLUMN are USED ONLY TO INITIALISE THE NEC CHIP
					; WHICH SETS LINE AND COL AT(0,0) LOCATION
					; AFTER THAT WE ARE GOING TO USE VID_LINE & VID_COL REG TO LOCATE THE CURSUR POSITION

V_A		EQU	0x11		;A
V_B		EQU	0x12		;B
V_C		EQU	0x13		;C
V_D		EQU	0x14		; GET THE IDEA?
V_E		EQU	0x15		;
V_F		EQU	0x16		;
V_G		EQU	0x17		;
V_H		EQU	0x18		;
V_I		EQU	0x19		;
V_J		EQU	0x1A		;
V_K		EQU	0x1B		;
V_L		EQU	0x1C		;
V_M		EQU	0x1D		;
V_N		EQU	0x1E		;
V_O		EQU	0x00		;  (NOTE THAT 0 IS THE SAME AS O)
V_P		EQU	0x20		;
V_Q		EQU	0x21		;
V_R		EQU	0x22		;
V_S		EQU	0x23		;
V_T		EQU	0x24		;
V_U		EQU	0x25		;
V_V		EQU	0x26		;
V_W		EQU	0x27		;
V_X		EQU	0x28		;
V_Y		EQU	0x29		;
V_Z		EQU	0x2A		;
V_BLANK		EQU	0x10		;BLANK
V_COLON		EQU	0x0A		;COLON
V_HAPPY		EQU	0x77		;MUSICAL QUARTER NOTE SYMBOL
V_ARROW		EQU	0x3B		;RIGHT ARROW   --->
V_DECIMAL	EQU	0x0E		;DECIMAL .
V_0		EQU	0x00		;  (NOTE THAT 0 IS THE SAME AS O)
V_1		EQU	0x01		;
V_2		EQU	0x02		;
V_3		EQU	0x03		;
V_4		EQU	0x04		;
V_5		EQU	0x05		;
V_6		EQU	0x06		;
V_7		EQU	0x07		;
V_8		EQU	0x08		;
V_9		EQU	0x09		;  THAT'S IT!

BS		EQU 2			;BACKSPACE BIT =2 OF RC_FLAG

;*******************************************************************************************************
;*******************************************************************************************************
;*******************************************************************************************************







;**********************************************************************
		ORG     0x000             ; processor reset vector
		clrf  PCLATH            ; ensure page bits are cleared
  		goto    SETUP             ; go to beginning of program

		ORG     0x004             ; interrupt vector location
		movwf   w_temp            ; save off current W register contents
		movf	STATUS,w          ; move status register into W register
		movwf	status_temp       ; save off contents of STATUS register


; isr code can go here or be located as a call subroutine elsewhere


		movf    status_temp,w     ; retrieve copy of STATUS register
		movwf	STATUS            ; restore pre-isr STATUS register contents
		swapf   w_temp,f
		swapf   w_temp,w          ; restore pre-isr W register contents
		retfie                    ; return from interrupt








;*******************************************************************************************************
;** SETUP
;** 
;** This block of code must be the first block to run.  It must run only once.

SETUP

		CLRWDT
		CLRF 	TMR0
		MOVLW 	0X00
		MOVWF	INTCON		; CLEAR INCON REG FOR NO INTERRUPTS

		
		
		BSF	STATUS,RP0		;SWITCH TO BANK 1
		MOVLW	0x06			;CONFIGURE PORTA
		MOVWF	ADCON1			; AS DIGITAL INPUTS


;** Assign ports as inputs or as outputs
;**
;**  NOTE: This setup makes HORZ_VGA and VERT_VGA outputs.
;**        TRISB = 1100,0001 & TRISC = 1001,0101 when using HORZ_VGA & VERT_VGA as inputs.

		MOVLW	0xFA		;1100,0010 TO W:  1=IN  0=OUT
		MOVWF	TRISA		; PORT A ASSIGNMENTS RB(n):  7 6 5 4 3 2 1 0
					;                DIRECTION:  1 1 1 1 1 0 1 0    
		MOVLW	0xC0		;1100,0001 TO W:  1=IN  0=OUT
		MOVWF	TRISB		; PORT B ASSIGNMENTS RB(n):  7 6 5 4 3 2 1 0
					;                DIRECTION:  1 1 0 0 0 0 0 0
		MOVLW	0xD1		;1001,0101 TO W:  1=IN  0=OUT
		MOVWF	TRISC		; PORT C ASSIGNMENTS RB(n):  7 6 5 4 3 2 1 0
					;                DIRECTION:  1 1 0 1 0 0 0 1
					
					
;** Set up the RS-232 Serial Port
;**


;** INITIALIZE USART
					; 8-bit transmission, transmit enabled, Asynchronous mode, low
					; baud rate,
					;  Clear TRMT shift register status bit.
		MOVLW	0x22		; TXSTA = b'0010,0010'
		MOVWF	TXSTA		; 
		BCF	STATUS,RP0	; Bank 0 
					; Serial port enabled, 8-bit receive, enable continuous receive.
		MOVLW	0x90		; RCSTA = b'1001,0000'
		MOVWF	RCSTA		; 

; USART Baudrate adjust 
		BSF	STATUS,RP0	; Bank1 
		MOVLW	D'15'		; Set Baud rate to 19,200 (based on a 20 MHz oscillator)
		MOVWF	SPBRG 
		BCF	TXSTA, BRGH	; Just in case - make sure set to low baud rate.  (I may remove
					; this line later) 

		BCF	STATUS,RP0	;SWITCH BACK TO BANK 0
		BCF	STATUS,RP1
		NOP			
		MOVF	TXREG,W		;	\
		MOVF	TXREG,W		;	|CLEARS ALL THE STACKS IN THESE REGS
		MOVF	TXREG,W		;	|NECESSARY STEPS TO PREVENT ERRORS
		MOVF	RCREG,W		;	|
		MOVF	RCREG,W		;	|
		MOVF	RCREG,W		;	/	
		
		CLRW
		MOVWF	TXREG		; SEND DUMY CHARACTER THIS WILL SET THE TXIF FLASH SET AFTER TRANSMISSION
		
		
;** Set up the ports for use:
;**

		CLRF	PORTA		;SET ALL LINES LOW
		NOP
		CLRF	PORTB		;SET ALL LINES LOW
		NOP
		CLRF	PORTC		;SET ALL LINES LOW
		NOP



		CLRF	PIR1		; Delete all Interrupt flags  
		CLRF	PIR2		; NOTE: I'm not actually using interrupts, but I do check these 
							; bits. 
					
					
		CLRF	FFIELDS		;	\
		CLRF	FCOUNTL		;	|
		CLRF	FCOUNTH		;	|
		CLRF	FSECONDSL	;	|	
		CLRF	FSECONDSH	;	|----CLEAR ALL THE FIELD REGISTERS
		CLRF	FMINUTESL	;	|
		CLRF	FMINUTESH	;	|
		CLRF	FHOURSL		;	|
		CLRF	FHOURSH		;	/
		CLRF	RC_FLAG
		
		
		
		;**************************************************************************************************************

	
		
;*******************************************************************************************************
;*******************************************************************************************************
;	WAKEUP DISPLAY,DISPLAY INTRO TEXT
;	STARTUP ROUTINE
;*******************************************************************************************************
STARTUP		
		CLRW						
		MOVWF 	DELAY_1			;		\WAIT FOR 1 SEC		
		CLRW				;		 |
		MOVWF 	DELAY_2			;		 |
		MOVLW 	0X13			;		 |STARTUP DELAY			
		MOVWF 	DELAY_0			;		 |FOR EVERYTHING TO GET SETTLED DOWN
		CALL 	DELAY			;		/ 
	
	
		CLRW
		MOVWF	TXREG		; SEND DUMY CHARACTER THIS WILL SET THE TXIF FLASH SET AFTER TRANSMISSION
		CALL 	VID_INIT		; ACTIVATE THE NEC VIDEO  ;********STACK-3***********
		CALL	INTRO			; DISPLAY INTRO TEXT
	
		CLRW					
		MOVWF	STATE_DETECT	; SET STATE_DETECT REGISTER TO 00. THE DEFALUT STATE--FIELD COUNTER
;*******************************************************************************************************
;*******************************************************************************************************
;*******************************************************************************************************
;*******************************************************************************************************
;							MAIN PROGRAM START
;
;MAIN PROGRAM IS BASICALLY A LOOKUP ROUTINE WHICH DETETS THE STATE BASED ON THE VALUE OF
;STATE_DETECT REGISTER
; THE DEFAULT STATE IS STATE 00. SO AT THE BEGINING OF MY PROGRAM I WILL SET MY STATE_DETECT REGISTER
;TO 00
;*******************************************************************************************************
;*******************************************************************************************************
;*******************************************************************************************************
;*******************************************************************************************************
	;	MOVLW	0X01				
;		MOVWF	STATE_DETECT	; SET STATE_DETECT REGISTER TO 00. THE DEFALUT STATE--FIELD COUNTER
	;	CALL	ENTER_TEXT_HTRM		; THIS IS TEST I WILL REMOVE IT AFTER DONE
;*******************************************************************************************************
;*******************************************************************************************************
MAIN
								
		MOVF	STATE_DETECT,W
		NOP
		NOP
		NOP
		
		ADDWF	PCL,F				; BRNACH TO DIFF STAGE BASED ON STATE_DETECT REG
		
		
		GOTO	STAGE_00			;STATE 00
		GOTO	STAGE_01			;STATE 01
		GOTO	STAGE_02			;STATE 02 IT DOES NOT DELETE TEXT BUT IT WILL ASK YOU "DO YOU WANT TO DELETE TEXT"
									; AND WHEN YOU ENTER IT IT GOES TO STATE 12 AND DELETES THE TEXT
		GOTO	STAGE_03			;STATE 03
		GOTO	STAGE_04			;STATE 04
		NOP							;MY COUNTER WILL NEVER COME TO THIS POINT	
		NOP							; BECAUSE ITS CONTROLLED BY STATE_DETECT REG
		NOP							; AND IN MY SUBROUTINE THERE ARE NOT ANY VALUES FOR 	
		NOP							; STATE_DETECT TO COME TO THIS POINT
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		GOTO	STAGE_10		 	; STATE 10 RESET THE COUNTER
		GOTO	STAGE_11			; STATE 11 GOES TO THE SAME STATE 
		GOTO	STAGE_12			; STATE 12 DELETES TEXT
		GOTO	STAGE_13			; STATE 13 DISPLAY VGA_TRIGGER--THIS STATE ALSO HAS SUB-STATES 
									; I WILL IMPLEMENT THOSE LETTER
		GOTO	STAGE_14			;STATE 14  GOES TO SAME STATE
;-----------------------------------------------------------------------------------------------------	

STAGE_00
		CALL FIELD_COUNTER			; FIELD COUNTER
		GOTO MAIN
STAGE_01
		CALL ENTER_TEXT				; ENTER TEST THROUGH HYPER TERMINAL
		GOTO MAIN
STAGE_02
		CALL CLEAR_TEXT?			; ASKS IF YOU WANT TO DELETE TEXT FROM EE PROM EXTERNAL
		GOTO MAIN
STAGE_03						
		CALL VGA_LINE_COUNTER			; VGA TRIGER COUNTER;; THIS STAGE IS NOT IMPLEMENTED			
		GOTO MAIN
STAGE_04
		CALL INFRARED_TXRX		  	;INFRARED;; THIS STAGE IS NOT IMPLEMENTED
		GOTO MAIN
STAGE_10
		CALL FIELD_COUNTER			; FIELD COUNTER RESET
		GOTO MAIN
STAGE_11							; SAME STAGE
		CALL ENTER_TEXT
		GOTO MAIN
STAGE_12							; DELETES TEXT FROM EXTERNAL EE PROM
		CALL CLEAR_TEXT	
		GOTO MAIN
STAGE_13							; VGA TRIGER COUNTER;; THIS STAGE IS NOT IMPLEMENTED
		CALL VGA_TRIG	
		GOTO MAIN
STAGE_14							; 
		CALL INFRARED_TXRX			; INFRARED ;; THIS STAGE IS NOT IMPLEMENTED
		GOTO MAIN	

					
;**************************************************************************************************************
;**************************************************************************************************************
;							END OF MAIN ROUTINE
;**************************************************************************************************************


;*******************************************************************************************************
;*******************************************************************************************************
;** CHARACTER TRANSLATE TABLE
;** 
;** THIS TABLE TRANSLATES ASCII TO NEC 6450 VIDEO STANDARD
	;	ORG 0X1000		; SET THE LOCATION AT THE BEGINNING OF THE PAGE 2

NEC_LOOKUP1

		
		MOVF	LOOKUP_REG,W
		BTFSC	LOOKUP_REG,7
		RETLW	0X7F
		
		
		
		NOP
		NOP
	
		
					;   ASCII		6450	uC
					; CHAR (HEX)	--->	CHAR or Action
		ADDWF	PCL,F		;-----------------------------------

;** INTERCEPTED CHARACTERS:  
;** Any character returned from this table with a value of 0x7F hex is not printed 
;** to the screen or echoed to the keyboard.
;**

		RETLW	0x7F		; NUL 	00		Throw away, do not print or echo.	
		RETLW	0x7F		; SOH	01		Throw away, do not print or echo.
		RETLW	0x7F		; STX	02		Throw away, do not print or echo.
		RETLW	0x7F		; ETX 	03		Throw away, do not print or echo.
		RETLW	0x7F		; EOT	04		Throw away, do not print or echo.
		RETLW	0x7F		; ENQ	05		Throw away, do not print or echo.
		RETLW	0x7F		; ACK	06		Throw away, do not print or echo.
		RETLW	0x7F		; BEL	07		Throw away, do not print or echo.
		RETLW	0x7F		;  BS	08		Throw away, do not print or echo.
		RETLW	0x7F		;  HT	09		Throw away, do not print or echo.
		RETLW	0x7F		;  LF	0A		Throw away, do not print or echo.
		RETLW	0x7F		;  VT	0B		Throw away, do not print or echo.
		RETLW	0x7F		;  FF	0C		Throw away, do not print or echo.
		RETLW	0x80		;  CR	0D		CR - implements a carriage return 
					;                         onscreen & echo.
		RETLW	0x7F		;  SO	0E		Throw away, do not print or echo.
		RETLW	0x7F		;  SI	0F		Throw away, do not print or echo.
		RETLW	0x7F		; DLE	10		Throw away, do not print or echo.
		RETLW	0x7F		; DC1	11		Throw away, do not print or echo.
		RETLW	0x7F		; DC2	12		Throw away, do not print or echo.
		RETLW	0x7F		; DC3	13		Throw away, do not print or echo.
		RETLW	0x7F		; DC4	14		Throw away, do not print or echo.
		RETLW	0x7F		; NAK	15		Throw away, do not print or echo.
		RETLW	0x7F		; SYN	16		Throw away, do not print or echo.
		RETLW	0x7F		; ETB	17		Throw away, do not print or echo.
		RETLW	0x7F		; CAN	18		Throw away, do not print or echo.
		RETLW	0x7F		;  EM	19		Throw away, do not print or echo.
		RETLW	0x7F		; SUB	1A		Throw away, do not print or echo.
		RETLW	0x7F		; ESC	1B		Throw away, do not print or echo.
		RETLW	0x7F		;  FS	1C		Throw away, do not print or echo.
		RETLW	0x7F		;  GS	1D		Throw away, do not print or echo.
		RETLW	0x1E		;  RS	1E		RECORD SEPERATOR - This is used to 
					;                         seperate data in EEPROM.
		RETLW	0x7F		;  US	1F		Throw away, do not print or echo.


		RETLW	0x10		;  SP	20		SP (onscreen and echo)
		RETLW	0x7F		;   !	21		Throw away, do not print or echo.	
		RETLW	0x7F		;   "	22		Throw away, do not print or echo.	
		RETLW	0x7F		;   #	23		Throw away, do not print or echo.
		RETLW	0x7F		;   $	24		Throw away, do not print or echo.
		RETLW	0x7F		;   %	25		Throw away, do not print or echo.
		RETLW	0x7F		;   &	26		Throw away, do not print or echo.
		RETLW	0x73		;   '	27		*
		RETLW	0x0B		;   (	28		<
		RETLW	0x0C		;   )	29		>
		RETLW	0x73		;   *	2A		*
		RETLW	0x7F		;   +	2B		Throw away, do not print or echo.
		RETLW	0x0F		;   ,	2C		,
		RETLW	0x0D		;   -	2D		-
		RETLW	0x0E		;   .	2E		.
		RETLW	0x6D		;   /	2F		/
		RETLW	0x00		;   0	30		0
		RETLW	0x01		;   1	31		1
		RETLW	0x02		;   2	32		2
		RETLW	0x03		;   3	33		3
		RETLW	0x04		;   4	34		4
		RETLW	0x05		;   5	35		5
		RETLW	0x06		;   6	36		6
		RETLW	0x07		;   7	37		7
		RETLW	0x08		;   8	38		8
		RETLW	0x09		;   9	39		9	
		RETLW	0x0A		;   :	3A		:
		RETLW	0x6B		;   ;	3B		;
		RETLW	0x0B		;   <	3C		<
		RETLW	0x3B		;   =	3D		RIGHT ARROW
		RETLW	0x0C		;   >	3E		>
		RETLW	0x50		;   ?	3F		?
		RETLW	0x7F		;   @	40		Throw away, do not print or echo.
		RETLW	0x11		;   A	41		A
		RETLW	0x12		;   B	42		B
		RETLW	0x13		;   C	43		C
		RETLW	0x14		;   D	44		D
		RETLW	0x15		;   E	45		E
		RETLW	0x16		;   F	46		F
		RETLW	0x17		;   G	47		G
		RETLW	0x18		;   H	48		H
		RETLW	0x19		;   I	49		I
		RETLW	0x1A		;   J	4A		J
		RETLW	0x1B		;   K	4B		K
		RETLW	0x1C		;   L	4C		L
		RETLW	0x1D		;   M	4D		M
		RETLW	0x1E		;   N	4E		N
		RETLW	0x00		;   O	4F		O
		RETLW	0x20		;   P	50		P
		RETLW	0x21		;   Q	51		Q
		RETLW	0x22		;   R	52		R
		RETLW	0x23		;   S	53		S
		RETLW	0x24		;   T	54		T
		RETLW	0x25		;   U	55		U
		RETLW	0x26		;   V	56		V
		RETLW	0x27		;   W	57		W
		RETLW	0x28		;   X	58		X
		RETLW	0x29		;   Y	59		Y
		RETLW	0x2A		;   Z	5A		Z
		RETLW	0x0B		;   [	5B		<
		RETLW	0x7F		;   \	5C		Throw away, do not print or echo.
		RETLW	0x0C		;   ]	5D		>
		RETLW	0x3D		;   ^	5E		UP ARROW
		RETLW	0x7F		;   _	5F		Throw away, do not print or echo.		
		RETLW	0x7F		;   `	60		Throw away, do not print or echo.
		RETLW	0x51		;   a	61		a
		RETLW	0x52		;   b	62		b
		RETLW	0x53		;   c	63		c
		RETLW	0x54		;   d	64		d
		RETLW	0x55		;   e	65		e
		RETLW	0x56		;   f	66		f
		RETLW	0x57		;   g	67		g
		RETLW	0x58		;   h	68		h
		RETLW	0x59		;   i	69		i
		RETLW	0x5A		;   j	6A		j
		RETLW	0x5B		;   k	6B		k
		RETLW	0x5C		;   l	6C		l
		RETLW	0x5D		;   m	6D		m
		RETLW	0x5E		;   n	6E		n
		RETLW	0x5F		;   o	6F		o
		RETLW	0x60		;   p	70		p
		RETLW	0x61		;   q	71		q
		RETLW	0x62		;   r	72		r
		RETLW	0x63		;   s	73		s
		RETLW	0x64		;   t	74		t
		RETLW	0x65		;   u	75		u
		RETLW	0x66		;   v	76		v
		RETLW	0x67		;   w	77		w
		RETLW	0x68		;   x	78		x
		RETLW	0x69		;   y	79		y
		RETLW	0x6A		;   z	7A		z
		RETLW	0x0B		;   {	7B		<
		RETLW	0x7F		;   |	7C		Throw away, do not print or echo.
		RETLW	0x0C		;   }	7D		>
		RETLW	0x7F		;   ~	7E		Throw away, do not print or echo.
		RETLW	0x7F		;   	7F		Throw away, do not print or echo.

		RETLW	0xDD		;TOO FAR! DISPLAY --
;
		
		
		
		












;**************************************************************************************************************
;							6450 NEC Video Subroutines	
;**************************************************************************************************************

CIRCLE
	NOP
	GOTO CIRCLE
;**************************************************************************************************************
;DELAY ROUNTINE
; This routine will use 3 delay reg  delay_0,delay_1,delay_2. 
; Before calling this routine provide delay_0,delay_1,delay_2 values for appropriate delay.
;	check the "EEprom_data_me.xls" file to calculate the delay.
;MAX DELAY 13.4744074 SEC WITH ALL REG 0
;MIN DELAY 3.4 micro SEC WITH ALL REG 1
;********STACK-0***********
DELAY
		MOVF 	DELAY_0,W
		MOVWF 	DELAY_COUNT0
		MOVF 	DELAY_1,W
		MOVWF 	DELAY_COUNT1
		MOVF 	DELAY_2,W
		MOVWF	 DELAY_COUNT2
			
DELAY_LOOP0
		MOVF 	DELAY_1,W
		MOVWF	DELAY_COUNT1		
DELAY_LOOP1		
		MOVF 	DELAY_2,W
		MOVWF 	DELAY_COUNT2
DELAY_LOOP2
		NOP
		DECFSZ 	DELAY_COUNT2,F		;loop2= (25+1+2)*(delay_2) - 1 ====4 * delay_2 -1
		GOTO 	DELAY_LOOP2
		DECFSZ 	DELAY_COUNT1,F	;
		GOTO 	DELAY_LOOP1		;loop1= (2+loop2+1+2)*(delay_1) - 1 = _____(6+loop2)* delay_1 -1
		DECFSZ 	DELAY_COUNT0,F		
		GOTO 	DELAY_LOOP0		;loop0= (2+loop1+1+2)*(delay_0) - 1 = _____(5+loop1)* delay_0 + 5
		RETURN

;**************************************************************************************************************
; this routine initialise the NTSC video and cleans out the garbage value 
; it sets the color,size,background,vertical-horizontal address,line-colom address.
; run this routine at the begining of the program
; TIME TO RUN = 14.244200 mS
;********STACK-2***********
VID_INIT
		MOVLW V_FORMAT		;TESTMODE RELEASE OPCODE,SELSECT BANK1
		CALL VID_WRITE	
		
		MOVLW V_VERTPOS		;SELECTS VERTICAL POSITION FROM VERTICAL TRAILING EDGE
		CALL VID_WRITE
		
		MOVLW V_HORZPOS		;SELECTS HORIZONTAL POSITION FROM HORIZONTAL TRAILING EDGE
		CALL VID_WRITE
		
		MOVLW V_CHARSIZ		; MOV V_CHARSIZ INTO COUNTER COZ WE NEED TO DECREMENT THE V_CHARSIZ FOR CHAR SIZE IN 12 LINES
		MOVWF VID_COUNT3
		
		MOVLW 0X0C		;DEFINE CHAR SIZE FOR ALL 12 LINES IN NEC 6450
		MOVWF VID_COUNT2
VID_CHARSIZE		
		MOVF VID_COUNT3,W	; SELECT THE SIZE OF CHARACTER TO BE DISPLYED
		CALL VID_WRITE		; YOU CAN CHANGE THE SIZE JUST BY CHANGING THE CONSTANT "V_CHARSIZ"
	
		DECF VID_COUNT3,F 	; DECREMENTS AS-8B,8A,89... THAT WILL REDUCE THE LINE NUMBER IN NEC

		DECFSZ VID_COUNT2,F	; 
		GOTO VID_CHARSIZE	; 
		
		MOVLW	V_BANK0		;  SELECTS BANK0 FOR OTHER NEC OPCODES
		CALL	VID_WRITE	;   
                                
		MOVLW	V_MODE		;  
		CALL	VID_WRITE	;   
                                     
		MOVLW	V_DISPLAY	;  DISPAY ON OPCODE WAKES UP THE DISPLAY
		CALL	VID_WRITE	; 
		
		MOVLW	V_COLOR		;  DISPAY ON OPCODE WAKES UP THE DISPLAY
		CALL	VID_WRITE	; 
                                
		MOVLW	V_LINE		;  	 LINE ASSIGNMENT----line 1
		CALL	VID_WRITE	;   
                                        
		MOVLW	V_COLUMN	;  COLUMN ASSIGNMENT-----column 1
		CALL	VID_WRITE	;   
        	GOTO 	VID_ERAL	; EARASE ALL THE GARBAGE DATA AND CLEAN UP THE DISPLAY             
		RETURN
		

		
;**************************************************************************************************************
;This routine prints blank characters on to the screen and cleans up the garbage characters
;there are total 12 * 24 = 288 characters needs to be erased
;WE CAN USE 2 COUNTER ERASE ONE WITH 255 OTHER WITH 40
; EVEN IF ITS MORE THAN 288 CHARACTERS, ONCE IT FINISHES 288 CHARACTERS IT AGAIN GOES TO (0,0) 
;LOCATION AND STARTS ERASING IT 
;********STACK-2***********

VID_ERAL
		
		
		
		MOVLW 	0XFF			; COUNTER FOR 12 LINES OUTER COUNTER
		MOVWF 	GENERAL_COUNT	
ERAL_1		
		MOVLW 	0X10			; THIS IS THE BLANK CHARACTER
		CALL 	VID_WRITE			; ERASE ONE CHAR AT A TIME
		DECFSZ 	GENERAL_COUNT,F 
		GOTO 	ERAL_1			; ERASE FIRST 255 CHARACTERS
		
		MOVLW 	0X28			; COUNTER FOR 12 LINES OUTER COUNTER
		MOVWF 	GENERAL_COUNT	
ERAL_2	
		MOVLW 	0X10			; REST 40 CHARACTERS TO ERASE
		CALL 	VID_WRITE		; ERASE ONE CHAR AT A TIME
		DECFSZ 	GENERAL_COUNT,F 
		GOTO 	ERAL_2			; WHEN U COME HERE THE LINES AND COLUMNS MAY HAVE SHIFTED TO DIFFERENT LOCATION
						; OTHER THAN (0,0)
						; TO PLACE IT ON (0,0) WE NEED TO SUPPLY THE FOLOWING OPCODES TO NEC 6450
		
		MOVLW	V_LINE		;   LINE ASSIGNMENT
		MOVWF   VID_LINE	; THIS INDIACATES THE CURRENT POSITION OF THE CURSUR
		CALL	VID_WRITE	;   
                                        
		MOVLW	V_COLUMN	;  COLUMN ASSIGNMENT
		MOVWF   VID_COL		; THIS INDIACATES THE CURRENT POSITION OF THE CURSUR
		CALL	VID_WRITE	;  
	
		RETURN

;**************************************************************************************************************
;6450 requires to hold strobe high to read the data-more like slot machine
;need high >=1 us
;TOTAL TIME = 2+2+14 = 18 C
VID_STROBE
		BSF PORTA,VIDSTB		;	Strobe the 6450
		NOP						;	0.2 uS @ 20 MHZ frequency	
		NOP						;	hold strobe for at least 1 uS
		NOP						;
		NOP						;	
		NOP						;	i gave it 1.2 uS
		NOP						;
		BCF PORTA,VIDSTB		;	clear strobe
		NOP						;	0.2 uS @ 20 MHZ frequency	
		NOP						;	hold strobe for at least 1 uS
		NOP						;
		NOP						;	
		NOP						;	i gave it 1.2 uS
		NOP						;
		GOTO VID_BUSY			

;**************************************************************************************************************		
;after strobe 6450 requires 100 nS to take the data in the chip
;till that time it holds busy high
;you can reenter data only if its low
;so you wait for busy to go low
VID_BUSY
		BTFSC 	PORTA,BUSY		;	check if busy is cleared
		GOTO 	VID_BUSY
		RETURN		

;**************************************************************************************************************		
;portc,clock is used by 3 chips 595,93c56 and 6450
;the clockout routine is used for these 3 chips
;for 6450 min clockout required is 1.6uS and for others is 1 us
;so we are going hold the clock for atleast 1.6uS
;when u call and return from clockout its going to take 2 + 2 = 4 instruction cycles
;   TOTAL TIME = 2+2+12= 16 C		
		
		
CLOCKOUT
		BSF PORTC,CLOCK			;	toggle the clock high
		NOP
		NOP				;	hold for 1 uS 		
		NOP				; 		
		NOP
		NOP
		BCF PORTC,CLOCK			;	toggle the clock low
		NOP
		NOP				;	HOLD FOR 1 uS	
		NOP
		NOP
		NOP
		RETURN


;**************************************************************************************************************
;this routine writes the data to 6450 chip through serial interface on PORTC,D0
;for that we will use RLF command through C carry flag and tranfer to portc,d0
;6450 chip uses big indian first method means MSB goes first 
; ;********STACK-1***********



VID_WRITE
		BCF 	STATUS,RP0		
		BCF 	STATUS,RP1		; SELECT BANK 0 JUST IN CASE YOU ARE IN DIFF BANK
		MOVWF 	VID_DATA		;move from w reg to transport reg
		MOVLW 	0X08			; move 8 to vid_count reg	
		MOVWF 	VID_COUNT
			
VIDWRITE_LOOP	
		BCF	STATUS,C		;CLEAR THE CARRY BIT	
		RLF 	VID_DATA,F		; ROTATE LEFT THROUGH CARRY FLAG	
		BTFSC 	STATUS,C		; SENDS BIT ON PORTC,DO 
		BSF 	PORTC,DO		;	"
		BTFSS 	STATUS,C		;	"
		BCF 	PORTC,DO		;	"
		
		CALL 	CLOCKOUT		; CALL CLOCKOUT ROUTINE TO READ THE BIT
		DECFSZ 	VID_COUNT,F		; DO THIS FOR 8 TIMES
		GOTO 	VIDWRITE_LOOP 		; AND YOU WILL GET ONE BYTE TRANSFERED TO PORTC,D0 AND WITH CLOCKOUT TO NEC TEMP REG
		CALL 	VID_STROBE		; READ THE DATA IN
		
		RETURN
		
;**************************************************************************************************************		
;**************************************************************************************************************		
;**************************************************************************************************************		
;**************************************************************************************************************
	
;**************************************************************************************************************
;NEW_LINE
;This routine is only used for "CR"when printing E2prom to NEC 
; all it does is set VID_COL =0 COL LOCATION AND DOES VID_LINE++					
;**************************************************************************************************************
;EEprom write routine----------EEprom to NEC 6450
; this routine writes from E2 prom pic memory to nec chip
; all u need to supply is the start address in w reg before calling this routine
; ;********STACK-2***********

EE_2_NEC_LOOP
		INCF 	DATA_ADDR,F		; INCREMENT DATA_ADDR
		MOVF 	DATA_ADDR,W		; PREPARE TO READ NEXT ADDRESS IN E2PROM							
EE_2_NEC
		MOVWF 	DATA_ADDR		; MOV TO DATA_ADDRESS REG
		BSF 	STATUS, RP1 		;
		BCF 	STATUS, RP0 		;Bank 2
		MOVWF 	EEADR 			;to read from
		BSF 	STATUS, RP0 		;Bank 3
		BCF 	EECON1, EEPGD 		;Point to Data
		BSF 	EECON1, RD 		;Start read operation
		BCF 	STATUS, RP0 		;Bank 2
		MOVF 	EEDATA, W 		;W = EEDATA
		BCF  	STATUS,RP1
		BCF 	STATUS,RP0		;BACK TO BANK 0 ----DONT FORGET EVER
		MOVWF 	DATA_BYTE		; MOVE TO DATA_BYTE REG	

	
		XORLW 	0X1E			; CHECK IF THE DATA IS "RS" END ADDRESS.SUB WILL PRODUCE Z=1 IF THE NUMBER IS "RS"  
		BTFSC 	STATUS,Z		; IF ITS CLEAR THEN ITS NOT "RS" END OF ADDRESS
		RETURN				; IF ITS SET THEN ITS "RS" END OF ADDRESS--EXIT THE ROUTINE
		
		MOVF 	DATA_BYTE,W
		XORLW 	0X0D			; CHECK IF THE DATA IS "CR" NEW LINE.  SUB WILL PRODUCE Z=1 IF THE NUMBER IS "CR"
		BTFSC 	STATUS,Z		; IF ITS CLEAR THEN ITS NOT "CR" NEW LINE
		GOTO 	NEW_LINE		; IF ITS SET THEN ITS "CR" NEWLINE--RUN NEW_LINE ROUTINE_THIS WILL MOVE CURSUR TO THE BEGINNING OF THE 							;NEW LINE

	
				
		MOVF 	DATA_BYTE,W  		; MOV DATA TO W REG
		MOVWF	LOOKUP_REG
	;	BSF	PCLATH, 4		;SELECT PAGE 2 (1000-17FF)
		CALL 	NEC_LOOKUP1		; CONVERT ASCII TO NEC UNDERSTANDABLE HEX FORMAT THROUGH LOOKUP TABLE
	;	BCF 	PCLATH, 4		; SELECT PAGE 0 (0000-07FF)
	;	BCF	PCLATH, 3		; 
		CALL 	VID_WRITE		; WRITE CHARACTER TO NEC

		

			
COL_INC MOVF VID_COL,W			;	\	TRACE THE CURSUR POSITION
		XORLW 	0XB7		;	|	THIS CODE IS GOING TO TELL YOU WHERE THE CURSUR IS ON THE SCREEN
		BTFSC 	STATUS,Z	;	|	WHENEVER YOU WRITE SOMETHING ON THE NEC INTERNALLY  INCREMENTS THE CURSUR 
		GOTO 	NEW_LINE	;	|  	BUT THERE IS NO WAY FROM NEC CHIP WE CAN FIND ITS POSITION 
		INCF 	VID_COL,F	;	|	SO WE HAVE TO MANUPULATE THE CURSUR POSITION AS WE RIGHT ANY DATA ON NEC CHIP
		GOTO 	LINCOL_DONE	;	|	VID_COL AND VID_LINE HOLDS THE CURSUR POSITION
					;	|	IF WE ARE AT END OF COL 24 GOTO LIN_INC FOR INCREMENTING LINE
					;	|	IF NOT THEN IT WILL INCREMENT THE COLUMN AND QUITS THE VID_LOCATE ROUTINE
NEW_LINE	MOVLW 	V_COLUMN	; 	|
		MOVWF 	VID_COL		; 	|	WHEN I COME TO THIS POINT THAT MEANS END OF COLUMN HAS OCCURED SO I SET MY COL=0
		MOVF 	VID_LINE,W	;	|   AND INCREMENT THE LINE
		XORLW 	0X9C		;	|	
		BTFSC 	STATUS,Z	;	|	IF WE ARE AT END OF LINE 12 GOTO SETLIN0 AND SET LINE =0
		GOTO 	SETLIN0		;	|	OTHERWISE WE INCREMENT THE LINE AND QUIT THE VID_LOCATE ROUTINE
		INCF 	VID_LINE,F	;	|
		GOTO 	LINCOL_DONE	;	|
SETLIN0 MOVLW V_LINE			;	|	
		MOVWF VID_LINE		;	|
LINCOL_DONE				;  	/	
		MOVF 	VID_COL,W	; LOCATE THE NEC CHIP CURSUR POSITION
		CALL 	VID_WRITE	; THIS IS DONE JUST FOR THE NEW LINE FUNCTION
		MOVF 	VID_LINE,W	; 
		CALL 	VID_WRITE	; WHEN YOU CALL NEW_LINE IT WILL MOV COL=0 AND INC LINE AND WRITES THE CURCUR POSITION IN NEC
		GOTO 	EE_2_NEC_LOOP	;	GOTO EE_2NEC_LOOP KEEP FETCHING DATA FROM E2 PROM UNTILL YOU GET "RS" END OF WORD INDICATOR 
					;	DONE

;**************************************************************************************************************





; FIRST RUN MESSEGE DISPLAY
;		Messege						E2 Prom Address			Time to Hold
		
;INTRO----RUNS ONLY AT THE START
;		Pelco Video tester ver X.xx			00				5 secs
;		by Mboyd							
;		Email Mboyd@pelco.com for manual							
							
;FIELD_COUNTER---FOLLOWED BY INTRO---ALSO USED SEPERATELY
;		Field Counter					46 + 4D				1 secs
;		HRS MINS SNDS FLDS O/E				56	
;		999: 59 : 59 : 59 : O E							
	
; this routine will display the above messeges.
; ;********STACK-3***********
INTRO
		CALL  	VID_ERAL		; THIS IS GOING TO ERASE ALL THE GARBAGE CHARACTERS AND PUTS CURSUR AT (0,0) LOCATION
        	CLRW			   	; MOV 00 MEMORY LOCATION WHEERE INTRO MESSEGE IS SITTING
        	CALL  	EE_2_NEC		; DISPLAY MESSEGE ON THE SCREEN
        					; ALL YOU HAVE TO DO IS GIVE START ADDRESS. IT WILL STOP ONCE IT COMES AT "RS" END OF WORD
							; NOW WAIT FOR 5 SEC	
		CLRW
		MOVWF 	DELAY_1		;		\WAIT FOR 5 SEC		
		CLRW				;		 |
		MOVWF 	DELAY_2		;		 |WAITS FOR 5 SECS
		MOVLW 	0X5F			;		 |FOR DELAY ROUTINE YOU HAVE TO PROVIDE VALUE TO THESE REGISTERS			
		MOVWF 	DELAY_0		;		 |CHECK OUT THE EXCEL SHEET NAMED IN DELAY ROUTINE FOR MORE INFO
		CALL 	DELAY			;		/ ON HOW I GOT THE REG VALUES FOR 5 SECS
		RETURN
		
		
		
;**************************************************************************************************************	
;**************************************************************************************************************	
;**************************************************************************************************************			


;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;						THIS IS THE FIELD COUNTER ROUTINE
;							BEGIN OF STAGE 1	
;**************************************************************************************************************		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************	
;********STACK-4***********

FIELD_COUNTER

		CLRF	FFIELDS		;	\
		
		
		MOVLW	0XFF
		MOVWF	FCOUNTL		;	|
		
		
		CLRF	FCOUNTH		;	|
		
		CLRF	FSECONDSL	;	|	
		CLRF	FSECONDSH	;	|----CLEAR ALL THE FIELD REGISTERS
		CLRF	FMINUTESL	;	|
		CLRF	FMINUTESH	;	|
		CLRF	FHOURSL		;	|
		CLRF	FHOURSH		;	/
		



		MOVLW	0X00
		MOVWF	STATE_DETECT	; SET THE STATE 00

        CALL  	VID_ERAL; THIS IS GOING TO ERASE ALL THE GARBAGE CHARACTERS AND PUTS CURSUR AT (0,0) LOCATION
		MOVLW 	0X46			;		\		
		CALL  	EE_2_NEC; 		 		|THIS ROUTINE WILL DISPLAY FIELD COUNTER MESSAGE ON THE SCREEN
		MOVLW 	0X4D			;		 |
		CALL  	EE_2_NEC; 		 |
		
		MOVLW	0X99			
		MOVWF	VID_LINE		; SET LINE = 10TH FOR HRS:59........
		
		MOVLW 	0X56			;		 |
		CALL  	EE_2_NEC; 		/



							; THIS ROUTINE WILL REFRESH THE FOLLOWING VALUES BASED ON FIELD CHANGES
							;		999: 59 : 59 : 59 : O E		
	
		CLRF	CURSOR_COL	; CLEAR THE CURSOR LOCATION TO (00)	
		CLRF 	CURSOR_LINE	
		CLRF	DATA_ADDR
		CLRF	DATA_BYTE
			  					
   		
		CALL	SER_2_NEC
	
	
;**************************************************************************************************************
;**************************************************************************************************************
; THIS ROUTINE WILL START COUNTING FIELDS,SECS,MIN.........
; BEFORE WE START COUNTING WE ARE GOING TO WAIT FOR EVEN PULSE TO OCCUR
; ONCE WE DETECT EVEN FIELD FROM PORTC,FIELD  WE ARE GOING ON INCREMENT FIELDS..AFTER 59 FIELDS INC SECS AND SO ON...
; NEC STORES THE INFORMATION SENT BY PIC AND THEN DISPLAY THE RESULT ON NTSC WHEN NEXT FIELD OCCURS
; SO LETS SAY YOU DETECTED EVEN PULSE ON PORTC,FIELD AND YOU WRITE YOUR DATA TO NEC, BUT NEC WILL DISPLAY THAT DATA
; IN THE NEXT FIELD, MEANS IN ODD FIELD.. SO YOU HAVE TO THINK AHEAD AND WHENEVER YOU DETECT EVEN YOU CONSIDER THAT ODD
; AND WRITE IT TO NEC..SAME WITH ODD -->EVEN    EVEN---->ODD
;--ODD=1 EVEN=0
;MAIN_COUNTER
;********STACK-5***********		
;**************************************************************************************************************

	
		CALL 	O_DETECT	; THIS WILL DETECT THE EVEN FIELD AND THEN THE WHOLE COUNTER START
MAIN_COUNTER
		CALL	NEW_FIELD	; THIS DETECTS NEW FIELD AND ALSO ODD OR EVEN
		CALL	WRITE_OE	; THIS WILL WRITE "O" OR "E" BASED ON THE RESULT OF NEW_FILED
		CALL	WRITE_FIELDS
		CALL	WRITE_SECONDS
		CALL	WRITE_MINUTES
		CALL	WRITE_HOURS
		CALL	KEY_PRESS	; CHECKS IF KEY IS PRESSED
		
		
		

		

		
		MOVF	STATE_DETECT,W
		XORLW	0X00		; CHECK IF MY STATE IS CHANGED OR NOT
		BTFSS	STATUS,Z
		RETURN				; STATE IS CHANGED SO RETURN TO THE MAIN PROGRAM
							; MAIN PROGRAM HAS LOOKUP STATE TABLE BASED ON THE VALUE OF STATE_DETECT REGISTER
		GOTO	MAIN_COUNTER; STATE IS NOT CHANGED SO CONTINUE WITH THE CURRENT STATE


;**************************************************************************************************************
;				SUBROUTINES OF MAIN_COUNTER
;**************************************************************************************************************
O_DETECT					; THIS ROUTINE WILL DETECT THE ODD FIELD
		BTFSC	PORTC,FIELD	; AND BASED ON THAT EVERYTHING ELSE WILL START COUNTING
		GOTO 	O_DETECT	;
WAIT	BTFSS	PORTC,FIELD	; AT THIS POINT MY FILED IS EVEN
		GOTO	WAIT		; SO I AM GOING TO WAIT FOR IT TO CHANGE ODD
		BSF 	OEDETECT,0	; ITS ODD
		RETURN				; ODD IS DETECTED NOW
;****************************************

NEW_FIELD					; THIS ROUTINE WILL DETECT NEW FIELD, ALSO ODD OR EVEN 
		BTFSS	OEDETECT,0	
		GOTO 	W_ODD		; AT THIS POINT EVEN FIELD IS RUNNING SO NEXT WILL BE ODD
							; SO WAIT TILL ODD OCCURS
W_EVEN	BTFSC 	PORTC,FIELD	; ITS ODD SO NEXT WILL BE EVEN SO WAIT FOR EVEN 			
		GOTO	W_EVEN					
		BCF 	OEDETECT,0	; ITS EVEN
		GOTO	NEW_FIELD_DONE	
		
W_ODD	BTFSS	PORTC,FIELD	
		GOTO 	W_ODD
		BSF 	OEDETECT,0	; ITS ODD				
NEW_FIELD_DONE
		RETURN				; AT THIS POINT NEW FILED HAS OCCURED AND OEDETECT,0 WILL TELL YOU IF ITS ODD  OR  EVEN
;*****************************************
WRITE_OE
		MOVLW	0X9B		; SET LINE TO THE LAST
		CALL 	VID_WRITE

					; THIS ROUTINE IS GOING TO WRITE "O" OR "E" ON THE NTCS ON ITS CORRESPONDING COLUMN		
		BTFSS	OEDETECT,0	; "0"=00 "E"=15		OPCODE FOR NEC  
		GOTO 	WRITE_E		; ITS ODD SO WRITE EVEN					
WRITE_O MOVLW	0XB3		; ODD="O"=COLUMN 20 == OPCODE FOR NEC==B3	
		CALL 	VID_WRITE	;
									
	
		MOVLW	0X9B		; SET LINE TO THE LAST
		CALL 	VID_WRITE
		
		MOVLW	0X00		
		CALL	VID_WRITE 	; PRINT "O"
ERASE_E	MOVLW	0XB5		; WHEN YOU WRITE "0" YOU NEED TO ERASE "E"
		CALL 	VID_WRITE 	; I AM ON "E
		MOVLW	0X10		; ERASE "E"	
		CALL	VID_WRITE
		GOTO 	WRITE_OE_DONE
		
WRITE_E MOVLW	0XB5		; EVEN="E"=COLUMN 22 == OPCODE FOR NEC==B6
		CALL 	VID_WRITE
		MOVLW	0X15		; PRINT "E"		
		CALL	VID_WRITE	
ERASE_O	MOVLW	0XB3		; WHEN YOU WRITE "E" YOU NEED TO ERASE "O"
		CALL 	VID_WRITE	; I AM ON "0"
		MOVLW	0X10		; ERASE "0"	
		CALL	VID_WRITE
		
WRITE_OE_DONE
		RETURN		
;****************************************;*************		
;****************************************;*************
;						COUNTER ROUTINE
; THIS ROUTINE DOES IT ALL COUNT FIELDS,SECS,MINS,HRS
;****************************************;*************		
;****************************************			
							; THIS ROUTINE WILL WRITE NUMBER OF FILEDS	 FROM 0 TO 59 AFTER 59 WILL RESET TO 00
							; WHEN THE COUNTER FIRST STARTS IT'S FIELD 00 BUT IF I INCREMENT IT IT WILL PRINT 01
WRITE_FIELDS	
		BTFSC	SEC_INC_DETECT,0
		INCF	FSECONDSL,F	; AND INCREMENT THE SECOND LOW DIGIT
		BCF		SEC_INC_DETECT,0	; CLEAR THE SEC_DETECT FLAG REGISTER
		
		INCF	FCOUNTL,F		; INCREMENT THE LOW DIGIT	
		MOVF	FCOUNTL,W				
		XORLW	0X0A		; 
		BTFSS	STATUS,Z	; CHECK IF LOW IS "10"
		GOTO 	WRITE_F		;RETURN TO MAIN COUNTER PROG
		
		CLRW 		
		MOVWF	FCOUNTL		; LOW DIGIT IS 10 NOW SO CLEAR LOW DIGIT
		INCF	FCOUNTH,F	; AND INCREMENT HIGH DIGIT 		
		MOVF	FCOUNTH,W	
		XORLW	0X06		; CHECK IF HIGH DIGIT IS 6
		
		BTFSS	STATUS,Z
		GOTO	WRITE_F
		call	WRITE_F		; PRINTS 60 ON SCREEN 
		
		CLRW	
		MOVWF 	FCOUNTH		; HIGH DIGIT IS 6 NOW SO CLEAR HIGH DIGIT
		BSF		SEC_INC_DETECT,0	; SET THE BIT INDICATING FRAME 60 HAS OCCURED AND AT NEXT FRAME 01 INCREMENT THE SECOND
		
		
		return					;PRINT 00
							; HERE WE ARE GOING FURTHER DOWN BECAUSE OF FIELD COUNT OVERFLOW, WE ARE INCREMENTING SECS
							; AND POSSIBLY MINUTS, HOURS IF CORRESPONDING OVERFLOW HAS OCCURED 

WRITE_F						; THIS WILL WRITE FCOUNTL AND FCOUNTH REG TO THE SCREEN ON CORRESPONDING COLUMN   
		MOVLW	0X9B		; SET LINE TO THE LAST
		CALL 	VID_WRITE
			
 		MOVLW	0XAF  			
 		CALL 	VID_WRITE	; NOW MY CURSUR IS ON THE FIELDCOUNT LOW BIT
 		MOVF	FCOUNTL,W		
 		CALL	VID_WRITE	; FIELDCOUNT LOW DIGIT IS PRINTED
 		
 
 		MOVLW	0XAE	  			
 		CALL 	VID_WRITE	; NOW MY CURSUR IS ON THE FIELDCOUNT HIGH BIT
 		MOVF	FCOUNTH,W		
 		CALL	VID_WRITE	; FIELDCOUNT HIGH DIGIT IS PRINTED
 		RETURN
 ;****************************************	SAME CONCEPT AS WRITE_FIELDS		
WRITE_SECONDS		
		MOVF	FSECONDSL,W				
		XORLW	0X0A			; 
		BTFSS	STATUS,Z		; CHECK IF LOW IS "10"
		GOTO 	WRITE_S			;RETURN TO MAIN COUNTER PROG
		
		CLRW 		
		MOVWF	FSECONDSL		; LOW DIGIT IS 10 NOW SO CLEAR LOW DIGIT
		INCF	FSECONDSH,F		; AND INCREMENT HIGH DIGIT 		
		MOVF	FSECONDSH,W	
		XORLW	0X06			; CHECK IF HIGH DIGIT IS 6
		
		BTFSS	STATUS,Z
		GOTO	WRITE_S
	
		
		CLRW	
		MOVWF 	FSECONDSH		; HIGH DIGIT IS 6 NOW SO CLEAR HIGH DIGIT
		INCF	FMINUTESL,F	; AND INCREMENT THE SECOND LOW DIGIT
		GOTO	WRITE_S		;PRINT 00
							; HERE WE ARE GOING FURTHER DOWN BECAUSE OF FIELD COUNT OVERFLOW, WE ARE INCREMENTING SECS
							; AND POSSIBLY MINUTS, HOURS IF CORRESPONDING OVERFLOW HAS OCCURED 

WRITE_S						
		MOVLW	0X9B		; SET LINE TO THE LAST
		CALL 	VID_WRITE						
							; THIS WILL WRITE FCOUNTL AND FCOUNTH REG TO THE SCREEN ON CORRESPONDING COLUMN   
		MOVLW	0XAA
 		CALL 	VID_WRITE	; NOW MY CURSUR IS ON THE FIELDCOUNT LOW BIT
 		MOVF	FSECONDSL,W		
 		CALL	VID_WRITE	; FIELDCOUNT LOW DIGIT IS PRINTED
 		
 	
 		MOVLW	0XA9	  			
 		CALL 	VID_WRITE	; NOW MY CURSUR IS ON THE FIELDCOUNT HIGH BIT
 		MOVF	FSECONDSH,W		
 		CALL	VID_WRITE	; FIELDCOUNT HIGH DIGIT IS PRINTED
 		RETURN
 		
 		
;****************************************	SAME CONCEPT AS WRITE_FIELDS			
 		
WRITE_MINUTES		
		MOVF	FMINUTESL,W				
		XORLW	0X0A			; 
		BTFSS	STATUS,Z		; CHECK IF LOW IS "10"
		GOTO 	WRITE_M			;RETURN TO MAIN COUNTER PROG
		
		CLRW 		
		MOVWF	FMINUTESL		; LOW DIGIT IS 10 NOW SO CLEAR LOW DIGIT
		INCF	FMINUTESH,F		; AND INCREMENT HIGH DIGIT 		
		MOVF	FMINUTESH,W	
		XORLW	0X06			; CHECK IF HIGH DIGIT IS 6
		
		BTFSS	STATUS,Z
		GOTO	WRITE_M
	
		
		CLRW	
		MOVWF 	FMINUTESH		; HIGH DIGIT IS 6 NOW SO CLEAR HIGH DIGIT
		INCF	FHOURSL,F	; AND INCREMENT THE SECOND LOW DIGIT
		GOTO	WRITE_M		;PRINT 00
							; HERE WE ARE GOING FURTHER DOWN BECAUSE OF FIELD COUNT OVERFLOW, WE ARE INCREMENTING SECS
							; AND POSSIBLY MINUTS, HOURS IF CORRESPONDING OVERFLOW HAS OCCURED 

WRITE_M						; THIS WILL WRITE FCOUNTL AND FCOUNTH REG TO THE SCREEN ON CORRESPONDING COLUMN   
		MOVLW	0X9B		; SET LINE TO THE LAST
		CALL 	VID_WRITE						
		
		MOVLW	0XA5			
 		CALL 	VID_WRITE	; NOW MY CURSUR IS ON THE FIELDCOUNT LOW BIT
 		MOVF	FMINUTESL,W		
 		CALL	VID_WRITE	; FIELDCOUNT LOW DIGIT IS PRINTED
 		
 		MOVLW	0XA4	  			
 		CALL 	VID_WRITE	; NOW MY CURSUR IS ON THE FIELDCOUNT HIGH BIT
 		MOVF	FMINUTESH,W		
 		CALL	VID_WRITE	; FIELDCOUNT HIGH DIGIT IS PRINTED
 		RETURN
 		
							
;****************************************	SAME CONCEPT AS WRITE_FIELDS			
 		
WRITE_HOURS		
		MOVF	FHOURSL,W				
		XORLW	0X0A			; 
		BTFSS	STATUS,Z		; CHECK IF LOW IS "10"
		GOTO 	WRITE_H			;RETURN TO MAIN COUNTER PROG
		
		CLRW 		
		MOVWF	FHOURSL		; LOW DIGIT IS 10 NOW SO CLEAR LOW DIGIT
		INCF	FHOURSH,F		; AND INCREMENT HIGH DIGIT 		
		MOVF	FHOURSH,W	
		XORLW	0X0A			; CHECK IF HIGH DIGIT IS 6
		
		BTFSS	STATUS,Z
		GOTO	WRITE_H
	
		
		CLRW	
		MOVWF 	FHOURSH		; HIGH DIGIT IS 6 NOW SO CLEAR HIGH DIGIT
		GOTO	WRITE_H		;PRINT 00
							; HERE WE ARE GOING FURTHER DOWN BECAUSE OF FIELD COUNT OVERFLOW, WE ARE INCREMENTING SECS
							; AND POSSIBLY MINUTS, HOURS IF CORRESPONDING OVERFLOW HAS OCCURED 

WRITE_H						; THIS WILL WRITE FCOUNTL AND FCOUNTH REG TO THE SCREEN ON CORRESPONDING COLUMN   
		MOVLW	0X9B		; SET LINE TO THE LAST
		CALL 	VID_WRITE						
		
		MOVLW	0XA1			
 		CALL 	VID_WRITE
							; NOW MY CURSUR IS ON THE FIELDCOUNT LOW BIT
 		MOVF	FHOURSL,W		
 		CALL	VID_WRITE	; FIELDCOUNT LOW DIGIT IS PRINTED
 		
		MOVLW	0XA0	  			
 		CALL 	VID_WRITE	; NOW MY CURSUR IS ON THE FIELDCOUNT HIGH BIT
 		MOVF	FHOURSH,W		
 		CALL	VID_WRITE	; FIELDCOUNT HIGH DIGIT IS PRINTED
 		RETURN
			

;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************				
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************				
;**************************************************************************************************************
;**************************************************************************************************************
;							END OF FIELD COUNTER ROUTINE							
;							STAGE 1 COMPLETE
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************



;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;
;
;
;								KEY PRESS ROUTINE
;KEY_PRESS					
;		CALL KEY_PRESS			; THIS ROUTINE CHECKS IF ANY KEY IS PRESSED OR NOT
								; KEY PRESS IS CONTINUOUS LOOP 
								; IF NONE OF THE KEYS ARE PRESSED THEN IT JUST GOES TO CURRENT STAGE
								; CURRENT STAGE IS DETECTED BY STATE_DETECT REG	
								; ALL STAGES WILL SET STATE_DETECT REG WITH ITS STAGE VALUE
								; LETS SAY I AM IN FIELD_COUNTER STAGE--I CALL IT AS DEFAULT STAGE WITH STAGE
								; NUMBER 0X00. SO IN THIS STAGE I WILL SET STATE_DETECT AS 0X00

; SO WHENEVER SOME BODY PRESSES ANY KEYS, AT THE END OF ANY STAGE IT WILL CALL KEY_PRESS ROUTINE
; AND DETECT UP,DOWN,ENTER KEYS AND ACT ACCORDINGLY.

;NOTE-FOR STAGE 03 CREATE SMALL ROUTINE TO DETECT THE 13 SUB UP-DOWN-ENTER DETECTION SEPERATELY
; SATGE 13 WILL BE TREATED SPECIALLY IN ITS SUB ROUTINE--JUST KEEP IN MIND
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
KEY_PRESS
		MOVLW 	0X00			
		MOVWF 	DELAY_2
		MOVLW 	0X05
		MOVWF 	DELAY_1		;		\WAIT FOR 1 m SEC		
		MOVLW 	0X01		;		 |FOR DELAY ROUTINE YOU HAVE TO PROVIDE VALUE TO THESE REGISTERS			
		MOVWF 	DELAY_0		;		 |CHECK OUT THE EXCEL SHEET NAMED IN DELAY ROUTINE FOR MORE INFO
		
		MOVF	PORTA,W		; MOVE CONTENT OF PORTA TO W
		MOVWF	KEY_DETECT	; MOVE IT TO KEY DETECT TO CHECK WHICH KEY IS PRESSED
		COMF	KEY_DETECT,W; COMPLEMENT IT BECASUE PORTA 3-4-5 IS ACTIVE LOW 
		ANDLW	0X38		; WE ARE ONLY INTERESTED IN BIT 3-4-5 FOR UP-DONW-ENTER
		MOVWF	KEY_DETECT	; MOVE IT TO KEY DETECT TO CHECK WHICH KEY IS PRESSED
		CALL	DELAY		; WAIT FOR DEBOUNCING TO VANISH FOR 1 m SECOND		
		
		MOVF	PORTA,W		; MOVE CONTENT OF PORTA TO W
		MOVWF	KEY_DETECT_C; MOVE IT TO KEY DETECT CONFORM TO CHECK WHICH KEY IS PRESSED
		COMF	KEY_DETECT_C,W; COMPLEMENT IT BECASUE PORTA 3-4-5 IS ACTIVE LOW 
		ANDLW	0X38		; WE ARE ONLY INTERESTED IN BIT 3-4-5 FOR UP-DONW-ENTER
		MOVWF	KEY_DETECT_C; MOVE IT TO KEY DETECT CONFORM TO CHECK WHICH KEY IS PRESSED
		
		XORWF	KEY_DETECT,W	; MAKE SURE KEY PRESSED BEFORE DELAY WAS CORRECT BY CHECKING BOTH KY_PRESSED & KEY_PRESSED_C
							; REG. IF BOTH ARE EQAUL THEN WE HAVE RIGHT KEY IF NOT WRONG KEY BECASUE OF DEBOUNSING
		BTFSS	STATUS,Z	; Z=1 MATCH					
		RETURN				; NO KEY IS PRESSED
		
		MOVF	KEY_DETECT,W; CHECK FOR NO KEY PRESSED--KEY DETECT VALUE WILL BE 00
		XORLW	0X00
		BTFSC	STATUS,Z
		RETURN
		
		MOVLW 	0X07		;		\WAIT FOR 420 m SEC	
		MOVWF 	DELAY_2		;		|
		MOVLW 	0X00		;		|
		MOVWF 	DELAY_1		;		|
		MOVLW 	0X00		;		|FOR DELAY ROUTINE YOU HAVE TO PROVIDE VALUE TO THESE REGISTERS			
		MOVWF 	DELAY_0		;		|CHECK OUT THE EXCEL SHEET NAMED IN DELAY ROUTINE FOR MORE INFO
							;		|
		CALL	DELAY		; 		/
		
		
		
UP_DETECT
		MOVF	KEY_DETECT,W; 
		XORLW	0X08		; UP KEY OPCODE
		BTFSS	STATUS,Z
		GOTO	DOWN_DETECT	; UP IS NOT PRESSED, SO CHECK IF DOWN IS PRESSED
		GOTO	UP_STAGE	; UP KEY IS PRESSED SO MODIFY THE STATE_DETECT REG
	

DOWN_DETECT
		MOVF	KEY_DETECT,W;
		XORLW	0X10		; DOWN KEY OPCODE
		BTFSS	STATUS,Z
		GOTO	ENTER_DETECT	; DOWN IS NOT PRESSED, SO CHECK IF DOWN IS PRESSED
		GOTO	DOWN_STAGE	; DOWN KEY IS PRESSED SO MODIFY THE STATE_DETECT REG
	
		
ENTER_DETECT
		MOVF	KEY_DETECT,W; 
		XORLW	0X20		; ENTER KEY OPCODE
		BTFSS	STATUS,Z
		RETURN				; NONE KEY IS PRESSED..JUST A BAD CALL...RETURN AND CONTINUE WITH THE CURRENT STAGE
		GOTO	ENTER_STAGE	; ENTER KEY IS PRESSED SO MODIFY THE STATE_DETECT REG
		
		
;;**************************************************************************************************************		
;						KEY PRESS SUB ROUTINES
;						ENTER_STAGE,UP_STAGE,DOWN_STAGE,STAGE_CALL		
;**************************************************************************************************************		
UP_STAGE		
		MOVF	STATE_DETECT,W
		XORLW	0X04		; CHECK IF THE STATE_DETECT IS ON 0X04
		BTFSS	STATUS,Z	; 
		GOTO	INC_STAGE	; IF ITS NOT 4 THEN INCREMENT THE STATE_DETECT REG
		CLRW
		MOVWF	STATE_DETECT; ITS 4 SO RESET IT TO STAGE 0X00
		RETURN		
		
INC_STAGE	INCF	STATE_DETECT,F; INC STATE_DETECT		
		RETURN
DOWN_STAGE		
		MOVF	STATE_DETECT,W
		XORLW	0X00		; CHECK IF THE STATE_DETECT IS ON 0X00
		BTFSS	STATUS,Z	; 
		GOTO	DEC_STAGE	; IF ITS NOT 00 THEN DECREMENT THE STATE_DETECT REG
		MOVLW	0X04		
		MOVWF	STATE_DETECT; ITS 00 SO RESET IT TO STAGE 0X04
		RETURN		
		
DEC_STAGE	DECF	STATE_DETECT,F; INC STATE_DETECT		
		RETURN		
		
		
		
ENTER_STAGE	

		MOVF	STATE_DETECT,W
		ADDLW	0X10		; ADD 0X10  IN THE STATE_DETECT REG FOR SUB_STAGES
							; AT THE END OF SUB_STAGES I WILL SUBLW 0X10 FRMO STATE_DETECT TO RESET IT INTO MAIN STAGES
		MOVWF	STATE_DETECT
		RETURN		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;											STATE   01
;ENTER_TEXT							
;**************************************************************************************************************		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************	
;********STACK-X***********		

ENTER_TEXT	
	
		MOVLW	0X01
		MOVWF	STATE_DETECT	; SET THE STATE 01
			
		
		CALL  	VID_ERAL	; THIS IS GOING TO ERASE ALL THE GARBAGE CHARACTERS AND PUTS CURSUR AT (0,0) LOCATION
        MOVLW	0XBB	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN
        					; ALL YOU HAVE TO DO IS GIVE START ADDRESS. IT WILL STOP ONCE IT COMES AT "RS" END OF WORD
        MOVLW	0XA5	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN
        					; ALL YOU HAVE TO DO IS GIVE START ADDRESS. IT WILL STOP ONCE IT COMES AT "RS" END OF WORD		     
        MOVLW	0XD4	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN
        					; ALL YOU HAVE TO DO IS GIVE START ADDRESS. IT WILL STOP ONCE IT COMES AT "RS" END OF WORD	
       
		
		
		
ENTER_TEXT_HTRM     
		CLRF	CURSOR_COL	; CLEAR THE CURSOR LOCATION TO (00)	
		CLRF 	CURSOR_LINE	
		CLRF	DATA_ADDR
		CLRF	DATA_BYTE
		
		CALL	NEW_LINE_2	; CR RETURN AND NEW LINE FEED	  					
        CALL	OK			; PRINTS OK ON TO THE SCREEN	
     	CALL	SER_2_HYPE_NEC; PRINTS EE MEMORY TO NEC AND HYPER TERMINAL
    	CALL	CURSOR_ZERO
     	
        			
ENTER_TEXT_LOOP				; THIS IS THE MAIN LOOP,--ERROR_CHECK,DATA RECEIVE,KEY_PRESS,NEC_LOOKUP,ECO

		CALL	ERROR_CHECK	; REMOVE ALL THE ERROR FLAGS
		CALL	UART_RECEIVE; RECEIVE CHARACTER FROM HYPER TERMINAL
		CALL	KEY_PRESS	; CHECKS IF KEY IS PRESSED-- THESE ARE FIELD COUNTER KEYS  UP --DOWN ---ENTER
		
		BTFSS	RC_FLAG,0	; FIRST GATEWAY**---->>>>	
		GOTO	KEY_PRESS_CHECK;THIS CONDITION IS MY GATEWAY TO MOVE FURTHER
		;IF RC_FLAG IS SET THEN AND THEN MY PC WILL MOVE FORWARD OTHERWISE WILL GO IN FOREVER LOOP
		; FROM THIS POINT I KNOW THAT A KEY IS PRESSED ADN I NEED TO PROCESS MY KEY STROKES AND RELATED DATA
		;CALL	KEY_DETECT	; DETECTS WHICH KEY IS PRESSED--THIS IS FOR KEYBOARD..IT DETECTS 
							; ARROW KEYS,ENTER,ESC,BACKSPACE,DELETE
		
		
		
		
		CALL	HYPE_2_NEC	; THIS PRINTS TEXT FROM HYPER TERMINAL TO NEC
							; WHATEVER YOU WRITE IT WILL DISPLAY ON NEC
		CALL	KEYS_DETECT ; THIS IS FOR KEYS ON THE KEYBOARD
							; KEY_PRESS AND KEY_DETECT ARE DIFF..DONT GET CONFUSED
							; KEY_PRESS IS FOR FIRLD COUNTER KEYS UP DOWN ENTER
							;KEY_DETECT IS KEYBOARD KEY DETECTION FROM HYPER TERMINAL	
							; SECOND GATEWAY**---->>>>	
		BTFSS	RC_FLAG,1	; IF RC_FLAG,1 IS SET THEN WE KNOW THAT SOME DATA HAS PRINTED TO THE SCREEN 
		GOTO	KEY_PRESS_CHECK	; AND WE NEED TO SAVE THAT DATA IN TO SEE PROM	
		
		
		CALL	HYPE_2_SER	; THIS ROUTINE WILL SAVE DATA TO EE PROM FROM HYPER TERMINAL
			
							

						
KEY_PRESS_CHECK							
		MOVF	STATE_DETECT,W
		XORLW	0X01		; CHECK IF MY STATE IS CHANGED OR NOT
		BTFSC	STATUS,Z
		GOTO	ENTER_TEXT_LOOP	; STATE IS NOT CHANGED SO CONTINUE WITH THE CURRENT STATE
		RETURN				; STATE IS CHANGED SO RETURN TO THE MAIN PROGRAM
							; MAIN PROGRAM HAS LOOKUP STATE TABLE BASED ON THE VALUE OF STATE_DETECT REGISTER		

	;*******************************************************************************************************

;**************************************************************************************************************
;**************************************************************************************************************
;						SUB ROUTINES FOR ENTER_TEXT   STATE  01
;**************************************************************************************************************
;**************************************************************************************************************		
;*******************************************************************************************************
;*******************************************************************************************************
;** SERIAL PORT ROUTINES
;**
;** These routines write or read one byte of data to/from the serial communication port.  The byte of 
;** data is always stored in the DATA_BYTE register.  

;*******************************************************************************************************		
		

OK							; THIS WILL PRINT OK ON TO HTRM(HYPER TERMINAL)SCREEN
    	MOVLW	0X4F		; TO VARIFY THAT PIC TO PC RS232 IS ESTABLISHED
		CALL	UART_SEND	; MORE LIKE HANDSHAKING
		MOVLW	0X4B
		CALL	UART_SEND
		MOVLW	0X0A
		CALL	UART_SEND	
		MOVLW	0X0D
		CALL	UART_SEND		
		RETURN
;*******************;*******************;*******************


;*********************************
;*********************************



	

	;*************************************


ERROR_CHECK					; THIS ROUTINE WILL CHECK ERRORS  OERROR AND FERROR
							; OVERSLOW ERROR AND BAUDRATE ERROR
		BTFSC	RCSTA,OERR
		GOTO	OVERERROR	;if overflow error...
		BTFSC	RCSTA,FERR
		GOTO	FRAMEERROR	;if framing error...
		RETURN				; UART READY	

OVERERROR					; THIS ERROR IS CAUSEED BY OVERFLOW OF RCREG FIFO 
							; FLUSHING THE FIFO WILL REMOVE THIS ERROR 
		BCF		RCSTA,CREN	;pulse cren off...
		MOVF	RCREG,W		;flush fifo
		MOVF	RCREG,W		; all three elements.
		MOVF	RCREG,W
		BSF		RCSTA,CREN	;turn cren back on.
							;this pulsing of cren
							;will clear the oerr flag.
		RETURN

FRAMEERROR					;framing errors are usually
		MOVF	RCREG,W		;due to wrong baud rate
		RETURN

		

;*******************
UART_SEND
						
		BTFSS	PIR1,TXIF	; CHECK TO SEE IF THERE IS DATA IN THE TRANSMIT BUFFER
		GOTO	UART_SEND
		MOVWF	TXREG		; send byte
		RETURN
		
		
	
;*******************		
UART_RECEIVE		
		BTFSS	PIR1,RCIF			; CHECK THE RCREG STATUS BIT		
		GOTO	EMPTY_RETURN		; REG IS EMPTY
		MOVF	RCREG,W				; REG IS FULL SO READ THE DATA
		MOVWF	RC_DATA				; THIS WILL CLEAR THE RCIF FLAG INDICATIONG RCREG IS EMPLTY
		BSF		RC_FLAG,0			; SET THE FLAG
		RETURN						; MOVE CONTENT OF W TO RC_DATA REG
									; THE REG TO HOLD THE DATA RECEIVED FROM HYPER TERMINAL	
									
  EMPTY_RETURN								
		BCF		RC_FLAG,0			; CLEAR THE FLAG			\
		RETURN							
;*******************		
ECHO								; ECHOS BACK TO THE TERMINAL
		MOVF	RC_DATA,W
		CALL	UART_SEND		
		RETURN		
;*******************		
KEYS_DETECT		
		MOVF	RC_DATA,W	
		XORLW	0X0D
		BTFSC	STATUS,Z
		GOTO	ENTER				; ENTER IS DETECTED		
		
		;XORLW	0X7F				;CHECKING FOR DELETE KEY
		;BTFSC	STATUS,Z
		;GOTO	DELETE				; DELETE IS DETECTED
		
		MOVF	RC_DATA,W
		XORLW	0X08
		BTFSC	STATUS,Z
		GOTO	BACKSPACE			; BACKSPACE IS DETECTED
		
		MOVF	RC_DATA,W
		XORLW	0X1B	
		BTFSS	STATUS,Z
		RETURN						; AT THIS POINT CHARACTER IS NOT DEL,BACKSPACE,ESC OR ARROW 
		CALL	UART_RECEIVE		; SO JUST QUIT THE LOOP 
		MOVF	RC_DATA,W
		XORLW	0X5B
		BTFSS	STATUS,Z
		GOTO	ESC					; ESC IS DETECTED SO GOTO ESC ROUTINE AND PERFORM RELATED TASKS
		
		CALL	UART_RECEIVE
		MOVF	RC_DATA,W
		XORLW	0X41
		BTFSC	STATUS,Z
		GOTO	UP_DETECTED			; UP ARROW DETECTED
	
		MOVF	RC_DATA,W
		XORLW	0X42
		BTFSC	STATUS,Z
		GOTO	DOWN_DETECTED		; DOWN DETECTED
		
		MOVF	RC_DATA,W
		XORLW	0X43
		BTFSC	STATUS,Z
		GOTO	RIGHT_DETECTED		; RIGHT DETECTED

		MOVF	RC_DATA,W
		XORLW	0X44
		BTFSC	STATUS,Z
		GOTO	LEFT_DETECTED		; LEFT DETECTED
		RETURN
;*********************************
;*********************************
;*********************************
;*********************************
;		WORD PROCESSOR ROUTINES
;*********************************
;*********************************
;*********************************
;*********************************
;*********************************
;*********************************
;*********************************

UP_DETECTED
		MOVF	CURSOR_LINE,W
		XORLW	0X00				; CHECK IF CURSUR IS AT LINE 0
		BTFSC	STATUS,Z			; IF CURSUR IS AT LINE 0 WE ARE NOT ABLE TO GO UP IN HYPER TERMINAL
		RETURN						; SO WE ARE GOING TO COPY EXACLY WHAT HYPER TERMINAL DOES
									; DO NOTHING NOT EVEN ECHO IT BACK
		MOVLW	0X1B
		CALL 	UART_SEND
		MOVLW	0X5B
		CALL 	UART_SEND
		MOVLW	0X41
		CALL 	UART_SEND
		
		DECF	CURSOR_LINE,F			; DECREMENT CURSOR LOCATION

		RETURN
DOWN_DETECTED
		MOVF	CURSOR_LINE,W
		XORLW	0X05				; CHECK IF CURSUR IS AT LINE 5
		BTFSC	STATUS,Z			; IF CURSUR IS AT LINE 5 WE ARE NOT ABLE TO GO DOWN IN HYPER TERMINAL
		RETURN						; SO WE ARE GOING TO COPY EXACLY WHAT HYPER TERMINAL DOES
									; DO NOTHING NOT EVEN ECHO IT BACK


		MOVLW	0X1B
		CALL 	UART_SEND
		MOVLW	0X5B
		CALL 	UART_SEND
		MOVLW	0X42
		CALL 	UART_SEND

		INCF	CURSOR_LINE,F			; INCREMENT CURSOR LOCATION

		RETURN	
RIGHT_DETECTED
		MOVF	CURSOR_COL,W
		XORLW	0X17				; CHECK IF CURSUR IS AT COLUMN 0X17--DECIMAL 24
		BTFSC	STATUS,Z			; IF CURSUR IS AT COLUMN 0X17 WE ARE NOT ABLE TO GO FURTHER RIGHT IN HYPER TERMINAL
		RETURN						; SO WE ARE GOING TO COPY EXACLY WHAT HYPER TERMINAL DOES
									; DO NOTHING NOT EVEN ECHO IT BACK

		
		MOVLW	0X1B
		CALL 	UART_SEND
		MOVLW	0X5B
		CALL 	UART_SEND
		MOVLW	0X43
		CALL 	UART_SEND

		INCF	CURSOR_COL,F
		RETURN
LEFT_DETECTED
		MOVF	CURSOR_COL,W
		XORLW	0X00				; CHECK IF CURSUR IS AT COLUMN 0X00--DECIMAL 00
		BTFSC	STATUS,Z			; IF CURSUR IS AT COLUMN 0X00 WE ARE NOT ABLE TO GO FURTHER LEFT IN HYPER TERMINAL
		RETURN						; SO WE ARE GOING TO COPY EXACLY WHAT HYPER TERMINAL DOES
									; DO NOTHING NOT EVEN ECHO IT BACK

		MOVLW	0X1B
		CALL 	UART_SEND
		MOVLW	0X5B
		CALL 	UART_SEND
		MOVLW	0X44
		CALL 	UART_SEND


		DECF	CURSOR_COL,F
		RETURN	
		
ESC

		MOVLW	0X02				; GO TO STATE 2 ASKING CLEAR TEXT?
		MOVWF	STATE_DETECT		; ESC IS FOR CLEAR TEST FROM MEMORY,NTSC AND FROM HYPERTERMINAL
		RETURN						; RIGHT NOW WE ARE ONLY IMPLEMENTING IN NTSC AND MEMORY
DELETE
		MOVLW	0X44
		CALL 	UART_SEND
		RETURN	
BACKSPACE
		MOVF	CURSOR_COL,W
		XORLW	0X00				; CHECK IF CURSUR IS AT COLUMN 0X00--DECIMAL 00
		BTFSC	STATUS,Z			; IF CURSUR IS AT COLUMN 0X00 WE ARE NOT ABLE TO GO FURTHER LEFT IN HYPER TERMINAL
		RETURN						; SO WE ARE GOING TO COPY EXACLY WHAT HYPER TERMINAL DOES
									; DO NOTHING NOT EVEN ECHO IT BACK
		MOVLW	0X08				; PUT SPACE ON AND ERASE IT	
		CALL	UART_SEND
		MOVLW	0X20				; PUT SPACE ON AND ERASE IT	
		CALL	UART_SEND
		MOVLW	0X08				; PUT SPACE ON AND ERASE IT	
		CALL	UART_SEND


		DECF	CURSOR_COL,F
		CALL	NEC_BACKSPACE		; MIMICS THE BACKSPACE ON NEC
		RETURN	
		
ENTER
		MOVF	CURSOR_LINE,W
		XORLW	0X05				; CHECK IF CURSUR IS AT LINE 5
		BTFSC	STATUS,Z			; IF CURSUR IS AT LINE 5 WE ARE NOT ABLE TO GO DOWN IN HYPER TERMINAL
		RETURN						; SO WE ARE GOING TO COPY EXACLY WHAT HYPER TERMINAL DOES
									; DO NOTHING NOT EVEN ECHO IT BACK

		MOVLW	0X0D				; ECHO THE ENTER
		CALL	UART_SEND
		MOVLW	0X0A				; THIS IS FOR NEW LINE FEED
		CALL	UART_SEND			; SEE WHEN U ENTER IT STARTS WITH NEW LINE AT 0 COL LOCATION
									; THATS WHAT WE ARE DOING OVER HERE
										
		CLRF	CURSOR_COL			; OK NOW WE ARE GOING TO MIMIC EXACTLY WHAT HYPER TERMINAL DOES
		INCF	CURSOR_LINE,F		; CURSUR COLUMN IS 00 THATS WHAT CARRIAGE RETURN OR ENTER DOES
											
		RETURN
;********************************
CURSOR_ZERO
		CALL	UP_DETECTED
		CALL	ENTER
		CALL	UP_DETECTED
		CALL	UP_DETECTED
		CALL	UP_DETECTED
		CALL	UP_DETECTED
		CALL	UP_DETECTED
		RETURN		
;********************************
HYPE_2_NEC
		BCF		RC_FLAG,1		; THIS FLAG BIT I AM USING TO SEE IF DATA PRINTED ON THE SCREEN--DATA NOT PRINTED YET
		MOVF 	RC_DATA,W  		; MOV DATA TO W REG
		MOVWF	LOOKUP_REG
	;	BSF		PCLATH, 4		;SELECT PAGE 2 (1000-17FF)
		CALL 	NEC_LOOKUP1		; CONVERT ASCII TO NEC UNDERSTANDABLE HEX FORMAT THROUGH LOOKUP TABLE
	;	BCF 	PCLATH, 4		; SELECT PAGE 0 (0000-07FF)
	;	BCF		PCLATH, 3		; 
		MOVWF	HYPE_TEMP		; MOVE DATA TO TEMP REG
		
	    XORLW	0X7F
		BTFSC	STATUS,Z
		RETURN
		MOVF	HYPE_TEMP,W		; THIS IS TO DETECT CR 0X0D
		XORLW	0X80
		BTFSC	STATUS,Z
		RETURN
		
		MOVF	CURSOR_COL,W
		XORLW	0X17
		BTFSS	STATUS,Z		; IF ITS END OF 24 TH COLUMN CARRIEAGE RETURN --ENTER IT START WITH NEW LINE COL 0
		GOTO	ECHO_HYPE
	
		MOVF	CURSOR_LINE,W
		XORLW	0X05
		BTFSC	STATUS,Z	
		RETURN					; END OF LAST COLUMN
		CALL	ENTER
		
	
		
ECHO_HYPE
		CALL	NEC_LOCATE		; LOCATE THE POSITION OF CURSOR FOR NEC
		MOVF 	HYPE_TEMP,W
		CALL 	VID_WRITE		; WRITE CHARACTER TO NEC	
		CALL	ECHO
		BSF		RC_FLAG,1		; THIS FLAG BIT I AM USING TO SEE IF DATA PRINTED ON THE SCREEN--DATA PRINTED
INC_CURSOR_COLUMN				; WE ARE NOT GOING TO INCREMENT CURSOR COL HERE
		INCF	CURSOR_COL,F	; WE WILL DO IT IN HYPE_2_SER LOOP	
		RETURN
		
;*********************************		
NEC_BACKSPACE
		CALL	NEC_LOCATE
		
		MOVLW	0X10				; PRINT SPACE AT LOCATION TO DELETE CHAR
		CALL 	VID_WRITE
		BSF		RC_FLAG,1
		MOVLW	0X20
		MOVWF	RC_DATA				; MOVE 20 TO RC_DATA I AM GOING TO USE RC_DATA TO WRITE ON SEE PROM
		BSF		RC_FLAG,BS			;SPECIAL CASE FOR BACKSPACE CURSOR STAYS THE SAME POINT AFTER DELETING PREVIOUS CHAR
		RETURN
		
;*********************************		
		
NEC_LOCATE
		MOVF	CURSOR_COL,W		; THIS WILL SET THE CURSOR POSITION ON THE NEC
		ADDLW	0XA0				; ADD TO CREATE NEC COL ADDRESS OPCODE
		CALL	VID_WRITE			; CURSOR POSITION WE GOT FROM ARROW KEYS,ENTER AND BACK SPACE KEYS
		MOVF	CURSOR_LINE,W		; IT WILL REFLACT OVER HERE
		ADDLW	0X94				; ADD TO CRETE NEC LINE ADDRESS OPCODE
		CALL	VID_WRITE
		RETURN
		
		
;*********************************
;*********************************
SER_ADDRESS_LOCATE					; THIS ROUTINE WILL LOCATE THE DATA ADDRESS INT HE MEMORY 
									; BY USING CURSOR_COL AND CURSOR_LINE REGS
									; THIS ROUTINE WILL PUT THE SER ADRESS IN TO DATA_ADDRESS REG
		MOVF	CURSOR_COL,W
		MOVWF	DATA_ADDR
		
		MOVF	CURSOR_LINE,W
		ADDLW	0X01
		MOVWF	HYPE_TEMP
SER_ADD_LOOP		
		DECFSZ	HYPE_TEMP,F
		GOTO	ADD_0X18
		RETURN
ADD_0X18		
		MOVLW	0X18			
		ADDWF	DATA_ADDR,F
		GOTO 	SER_ADD_LOOP
;*********************************
;*********************************
;*********************************
;*********************************		
HYPE_2_SER							; THIS ROUTINE WILL COPIES THE HYPER TERMINAL 
									; WHATEVER YOU WRITE -ERASE WILL BE COPIED IN TO EE PROM
		
		
		
		
		BTFSS 	RC_FLAG,BS
		GOTO	NO_BACKSPACE
		
		CALL	SER_ADDRESS_LOCATE;LOCATES ADDRESS OF SER BASED ON CURSOR_COL AND LINE REGS	
		CALL	SER_EE_EWEN			; ENABLE	ERASE/WRITE ON SER	
		MOVF	RC_DATA,W
		
		MOVWF	DATA_BYTE			; MOVE DATA TO DATA_BYTE REG
									; DATA ADDRESS IS ALREADY LOADED INTO DATA_ADDRESS REG
		CALL	SER_EE_WR			; THATS IT
		CALL	SER_EE_EWDS	
		BCF		RC_FLAG,BS			; CLEARTHE BACKSPACE FLAG
		RETURN						; I AM NOT USING ERASE DIRECTLY
		
		
								
				
NO_BACKSPACE	
		DECF	CURSOR_COL,F	
		CALL	SER_ADDRESS_LOCATE;LOCATES ADDRESS OF SER BASED ON CURSOR_COL AND LINE REGS	
		CALL	SER_EE_EWEN			; ENABLE	ERASE/WRITE ON SER	
		MOVF	RC_DATA,W
		
		MOVWF	DATA_BYTE			; MOVE DATA TO DATA_BYTE REG
									; DATA ADDRESS IS ALREADY LOADED INTO DATA_ADDRESS REG
		CALL	SER_EE_WR			; THATS IT
		CALL	SER_EE_EWDS	
		INCF	CURSOR_COL,F	
		
		BCF		RC_FLAG,BS			; CLEARTHE BACKSPACE FLAG
		RETURN						; I AM NOT USING ERASE DIRECTLY
									; BUT USING FF AS DATA WHEN BACKSPACE IS PRESSED AND WRITING 
									; IT AT THE ADDRESS
									; I AM NOT WORRIED ABOUT ANY KEY PRESS
									; BECAUSE EVERYTHING IS TAKE CARE IN HYPE_2_NEC AND KEY_DETECT ROUTINE
									; WHICH GIVES ME MY LINE-COL ADDRESSES AND DATA INTO RC_DATA REGS
;*********************************
;*********************************
SER_2_HYPE_NEC	
		MOVF	CURSOR_COL,W
		XORLW	0X17
		BTFSS	STATUS,Z			; IF ITS END OF 24 TH COLUMN CARRIEAGE RETURN --ENTER IT START WITH NEW LINE COL 0
		GOTO	PRINT_HYPE
	
		MOVF	CURSOR_LINE,W
		XORLW	0X05
		BTFSC	STATUS,Z	
		RETURN						; END OF LAST COLUMN
		CALL	ENTER
	
		
PRINT_HYPE	
		CALL	SER_EE_EWEN			; THIS ROUTINE WILL EXCECUTE ONLY AT THE BEGINING OF THE STAGE 1
		CALL	SER_ADDRESS_LOCATE
		CALL	SER_EE_RD
		MOVF	DATA_BYTE,W
		MOVWF	HYPE_TEMP		; MOVE DATA TO TEMP REG
		CALL	UART_SEND
		CALL	SER_EE_EWDS	
		MOVF 	HYPE_TEMP,W  	
	

	
	
		MOVWF	LOOKUP_REG
	;	BSF		PCLATH, 4		;SELECT PAGE 2 (1000-17FF)
		CALL 	NEC_LOOKUP1		; CONVERT ASCII TO NEC UNDERSTANDABLE HEX FORMAT THROUGH LOOKUP TABLE
	;	BCF 	PCLATH, 4		; SELECT PAGE 0 (0000-07FF)
	;	BCF		PCLATH, 3		; 
		MOVWF	HYPE_TEMP		; MOVE DATA TO TEMP REG
		

		
		CALL	NEC_LOCATE		
		MOVF	HYPE_TEMP,W
		CALL	VID_WRITE
	


		INCF	CURSOR_COL,F
		GOTO	SER_2_HYPE_NEC
;*********************************;*********************************
;*********************************;*********************************
	

SER_2_NEC	
		MOVF	CURSOR_COL,W
		XORLW	0X17
		BTFSS	STATUS,Z			; IF ITS END OF 24 TH COLUMN CARRIEAGE RETURN --ENTER IT START WITH NEW LINE COL 0
		GOTO	PRINT_HYPE1
	
		MOVF	CURSOR_LINE,W
		XORLW	0X05
		BTFSC	STATUS,Z	
		RETURN						; END OF LAST COLUMN
		CALL	ENTER
	
		
PRINT_HYPE1	
		CALL	SER_EE_EWEN			; THIS ROUTINE WILL EXCECUTE ONLY AT THE BEGINING OF THE STAGE 1
		CALL	SER_ADDRESS_LOCATE
		CALL	SER_EE_RD
		MOVF	DATA_BYTE,W
		MOVWF	HYPE_TEMP		; MOVE DATA TO TEMP REG
	;	CALL	UART_SEND
		CALL	SER_EE_EWDS	
		MOVF 	HYPE_TEMP,W  	
	

	
	
		MOVWF	LOOKUP_REG
	;	BSF		PCLATH, 4		;SELECT PAGE 2 (1000-17FF)
		CALL 	NEC_LOOKUP1		; CONVERT ASCII TO NEC UNDERSTANDABLE HEX FORMAT THROUGH LOOKUP TABLE
	;	BCF 	PCLATH, 4		; SELECT PAGE 0 (0000-07FF)
	;	BCF		PCLATH, 3		; 
		MOVWF	HYPE_TEMP		; MOVE DATA TO TEMP REG
		

		
		CALL	NEC_LOCATE		
		MOVF	HYPE_TEMP,W
		CALL	VID_WRITE
	


		INCF	CURSOR_COL,F
		GOTO	SER_2_NEC
	


	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	













;*******************************************************************************************************
;*******************************************************************************************************
;** SERIAL EEPROM FUNCTIONS
;** These routines read, write, and erase the on board Serial EEPROM.
;** 
;** NOTE:  The serial EEPROM is wired in the 256 addresses x 8 bits format.
;** This routine uses the two registers, DATA_BYTE and DATA_ADDR to access
;** the EEPROM.  The data register can be an input or an output depending
;** on whether the data is being read or written.  The address register is
;** always an input.
;** 
;** NOTE:  THE SERIAL EEPROM MUST BE ERASE/WRITE ENABLED OR DISABLED BEFORE
;** AND AFTER USE.
;** 

;*******************************************************************************************************
;**  ERASE / WRITE ENABLE
;**   All serial EEPROM programming modes must be preceded by an ERASE/WRITE
;**   Enable (EWEN) instruction.

SER_EE_EWEN
		
		BSF 	PORTA,RAMCS	; Select the EEPROM. 
		MOVLW	0x0C		; 12 CLOCK CYCLES FOR EWEN
		MOVWF	EE_CLOCKS	;

		MOVLW	SE_EWEN		; PUT THE EEPROM ENABLE OPCODE 
		MOVWF	GENERAL_TEMP	;  INTO THE GENERAL TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		BCF		PORTA,RAMCS	; DESELECT THE EEPROM, AND YOU ARE DONE!

		RETURN

;*******************************************************************************************************
;** ERASE / WRITE DISABLE
;**  All serial EEPROM programming modes remain enabled unless the ERASE/WRITE
;**  Disable (EWDS) instruction is executed (or when VCC is removed.)

SER_EE_EWDS
		BSF 	PORTA,RAMCS	; Select the EEPROM. 
		MOVLW	0x0C		; 12 CLOCK CYCLES FOR EWEN
		MOVWF	EE_CLOCKS	;

		MOVLW	SE_EWDS		; PUT THE EEPROM DISABLE OPCODE 
		MOVWF	GENERAL_TEMP	;  INTO THE GENERAL TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		BCF	PORTA,RAMCS	; DESELECT THE EEPROM, AND YOU ARE DONE!
	
		RETURN

;*******************************************************************************************************
;** WRITE
;**  SERIAL EEPROM WRITE (ONE BYTE TO ADDRESS SPECIFIED)

SER_EE_WR				; NOTE: Registers DATA_ADDR and DATA_BYTE must be loaded 
 					; before starting this routine.

		BSF 	PORTA,RAMCS	; Select the EEPROM. 
		MOVLW	0x04		; 4 CLOCK CYCLES FOR WRITE
		MOVWF	EE_CLOCKS	;

		MOVLW	SE_WRITE	; PUT THE EEPROM WRITE OPCODE 
		MOVWF	GENERAL_TEMP	;  INTO THE GENERAL TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		MOVLW	0x08		; 8 CLOCK CYCLES FOR 8 ADDRESS BITS
		MOVWF	EE_CLOCKS	;

		MOVF	DATA_ADDR,W	; TAKE THE ADDRESS BYTE TO BE WRITTEN
		MOVWF	GENERAL_TEMP	;  PLACE IT INTO THE TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		MOVLW	0x08		; 8 CLOCK CYCLES FOR 8 DATA BITS
		MOVWF	EE_CLOCKS	;

		MOVF	DATA_BYTE,W	; TAKE THE DATA BYTE TO BE WRITTEN
		MOVWF	GENERAL_TEMP	;  PLACE IT INTO THE TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		BCF	PORTA,RAMCS	; DESELECT THE EEPROM, AND YOU ARE DONE!

		RETURN

;*******************************************************************************************************
;** READ
;**  SERIAL EEPROM READ (ONE BYTE FROM ADDRESS SPECIFIED)

SER_EE_RD				; NOTE: Register DATA_ADDR must be loaded before starting this
					; routine.  
					; The READ routine will load the selected byte into DATA_BYTE and
					; also into the W register.

		BSF 	PORTA,RAMCS	; Select the EEPROM. 
		MOVLW	0x04		; 12 CLOCK CYCLES FOR READ
		MOVWF	EE_CLOCKS	;

		MOVLW	SE_READ		; PUT THE EEPROM READ OPCODE 
		MOVWF	GENERAL_TEMP	;  INTO THE GENERAL TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		MOVLW	0x08		; 8 CLOCK CYCLES FOR 8 ADDRESS BITS
		MOVWF	EE_CLOCKS	;

		MOVF	DATA_ADDR,W	; TAKE THE ADDRESS BYTE TO BE WRITTEN
		MOVWF	GENERAL_TEMP	;  PLACE IT INTO THE TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		MOVLW	0x08		; 8 CLOCK CYCLES TO READ 8 DATA BITS
		MOVWF	EE_CLOCKS	;

SER_EE_RD_LOOP
					; DATA IS VALID ON RISING EDGE OF THE CLOCK
		BSF	PORTC,CLOCK	;TOGGLE CLOCK HIGH 
		NOP			;DELAY             (.2 uSEC) \   Serial EEPROM must have 
		NOP			;DELAY             (.2 uSEC)  |-  a clock cycle of 1uSec.
		NOP			;DELAY             (.2 uSEC) /    (.5us low, .5us high)

		BTFSS	PORTC,DI	; TEST THE DATA INPUT TO SEE IF IT IS A ONE OR ZERO
		BCF	STATUS,C	;
		BTFSC	PORTC,DI	;
		BSF	STATUS,C	;
		
		RLF	GENERAL_TEMP,F		; AND ROTATE IT INTO THE TEMPORARY REGISTER

		BCF	PORTC,CLOCK	;TOGGLE CLOCK LOW  
		NOP			;DELAY             (.2 uSEC) \   Serial EEPROM must have 
		NOP			;DELAY             (.2 uSEC)  |-  a clock cycle of 1uSec.
		NOP			;DELAY             (.2 uSEC) /    (.5us low, .5us high)

		DECFSZ	EE_CLOCKS,F	; 
		GOTO	SER_EE_RD_LOOP	; GO GET THE NEXT BIT

		MOVF	GENERAL_TEMP,W	; MOVE THE DATA FROM THE TEMPORARY REGISTER AND INTO
		MOVWF	DATA_BYTE	; THE DATA REGISTER, AND INTO THE W REGISTER.

		BCF	PORTA,RAMCS	; DESELECT THE EEPROM, AND YOU ARE DONE!
;		CALL	SER_EE_SEND_BUSY
		RETURN


;*******************************************************************************************************
;** ERASE ALL
;**   SERIAL EEPROM ERASE ALL (SET ALL MEMORY LOCATIONS TO 0xFF)

SER_EE_ERAL				; NOTE: Nothing needed, just call the routine and the EEPROM 
					; is erased.

		BSF 	PORTA,RAMCS	; Select the EEPROM. 
		MOVLW	0x0C		; 12 CLOCK CYCLES FOR ERAL
		MOVWF	EE_CLOCKS	;

		MOVLW	SE_ERAL		; PUT THE EEPROM ERASE ALL OPCODE 
		MOVWF	GENERAL_TEMP	;  INTO THE GENERAL TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		BCF	PORTA,RAMCS	; DESELECT THE EEPROM, AND YOU ARE DONE!
		CALL	SER_EE_SEND_BUSY
		RETURN


;*******************************************************************************************************
;** WRITE ALL
;**  SERIAL EEPROM WRITE ALL (SET ALL MEMORY LOCATIONS TO THE VALUE HELD IN DATA_BYTE)

SER_EE_WRAL				; NOTE: Load the register DATA_BYTE with the value to be written.

		BSF 	PORTA,RAMCS	; Select the EEPROM. 
		MOVLW	0x0C		; 12 CLOCK CYCLES FOR WRAL
		MOVWF	EE_CLOCKS	;

		MOVLW	SE_WRAL		; PUT THE EEPROM WRITE ALL OPCODE 
		MOVWF	GENERAL_TEMP	;  INTO THE GENERAL TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		MOVLW	0x08		; 8 CLOCK CYCLES FOR 8 DATA BITS
		MOVWF	EE_CLOCKS	;

		MOVF	DATA_BYTE,W	; TAKE THE DATA BIT TO BE WRITTEN
		MOVWF	GENERAL_TEMP	;  PLACE IT INTO THE TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		BCF	PORTA,RAMCS	; DESELECT THE EEPROM, AND YOU ARE DONE!
		CALL	SER_EE_SEND_BUSY
		RETURN

;*******************************************************************************************************
;** ERASE
;**  SERIAL EEPROM ERASE (SET THE SPECIFIED MEMORY LOCATION TO THE VALUE 0xFF)

SER_EE_ERASE				; NOTE: This is an easy command for the serial EEPROM to 
					; execute, easier than using the WRITE command as a sort 
					; of erase method.

		BSF 	PORTA,RAMCS	; Select the EEPROM. 
		MOVLW	0x04		; 4 CLOCK CYCLES FOR ERASE (SENDS 4 MOST SIG. BITS)
		MOVWF	EE_CLOCKS	;

		MOVLW	SE_ERASE	; PUT THE EEPROM ERASE BYTE OPCODE 
		MOVWF	GENERAL_TEMP	;  INTO THE GENERAL TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		MOVLW	0x08		; 8 CLOCK CYCLES FOR 8 ADDRESS BITS
		MOVWF	EE_CLOCKS	;

		MOVF	DATA_ADDR,W	; TAKE THE ADDRESS BYTE TO BE WRITTEN
		MOVWF	GENERAL_TEMP	;  PLACE IT INTO THE TEMPORARY REGISTER
		CALL	SER_EE_SEND	;  AND SEND IT TO THE EEPROM

		BCF	PORTA,RAMCS	; DESELECT THE EEPROM, AND YOU ARE DONE!
	;	CALL	SER_EE_SEND_BUSY
		RETURN

;*******************************************************************************************************
;** SERIAL EEPROM IS BUSY
;**  USE THIS ROUTINE TO CHECK IF THE SERIAL EEPROM IS STILL BUSY
;**  CAUTION!  CAN CAUSE LOCKUP!

SER_EE_SEND_BUSY			; THIS ROUTINE CHECKS TO SEE IF THE SERIAL EEPROM IS 
					; CURRENTLY BUSY.  If the serial EEPROM is busy, this 
					; routine will wait for it.  
					; CAUTION, THIS COULD CAUSE A LOCK UP

		BSF	PORTA,RAMCS		; SELECT SERIAL EEPROM

SER_EE_SEND_BUSY_LOOP
		BTFSS	PORTC,DI		; CHECK TO SEE IF SERIAL EEPROM IS READY.
		GOTO	SER_EE_SEND_BUSY_LOOP	;  IF NOT, WAIT. (EEPROM is ready when DI is high)
		BCF	PORTA,RAMCS		; DESELECT SERIAL EEPROM
		RETURN

;*******************************************************************************************************
;** SERIAL SEND
;**  THIS ROUTINE IS USED BY ALL OF THE SERIAL EEPROM ROUTINES.

SER_EE_SEND				; NOTE: This routine is not meant to be called directly.
					; It should be called by a read/write/erase routine.




		RLF 	GENERAL_TEMP,F	; Rotate bit7 of temp into carry
		BTFSS 	STATUS,C	; Move carry bit to input of EEPROM

		BCF 	PORTC,DO	; SET THE DATA OUT BIT TO MATCH THE SELECTED BIT
		BTFSC	STATUS,C	;
		BSF	PORTC,DO	; 
		
		CALL	CLOCKOUT		; CLOCK THE BIT INTO THE EEPROM

		DECFSZ	EE_CLOCKS,F	; COUNT THE NUMBER OF BITS TO SEND.
					; NOTE: Zero is not counted!  

		GOTO	SER_EE_SEND	
		RETURN
		
		
		
		

;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;											STATE   02
;DELETE_TEXT?						
;**************************************************************************************************************		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************	
;********STACK-X***********	
CLEAR_TEXT?		
		MOVLW	0X02
		MOVWF	STATE_DETECT	; SET THE STATE 02



		CALL  	VID_ERAL	; THIS IS GOING TO ERASE ALL THE GARBAGE CHARACTERS AND PUTS CURSUR AT (0,0) LOCATION
        MOVLW	0XB0	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN
        
        MOVLW	0X50		; ? MARK ASKING DO YOU WANT TO DELETE TEXT?
        CALL	VID_WRITE
		
		CALL	NEC_CLEAR_TEXT?_MESSEGE
		CALL	HYPE_CLEAR_TEXT?_MESSEGE
        
       
        
        					
CLEAR_TEXT?_LOOP       					
		CALL	UART_RECEIVE; RECEIVE CHARACTER FROM HYPER TERMINAL
		CALL	KEY_PRESS	; CHECKS IF KEY IS PRESSED-- THESE ARE FIELD COUNTER KEYS  UP --DOWN ---ENTER
		
		
		BTFSS	RC_FLAG,0	; FIRST GATEWAY**---->>>>	
		GOTO	KEY_PRESS_CHECK_STATE2	
        CALL	Y_N_DETECT	; DETECTS ONLY Y OR N KEY IF U DONT PRESS ONE OF THOSE KEYS YOU ARE NOT GOING ANYWHERE

KEY_PRESS_CHECK_STATE2		
		MOVF	STATE_DETECT,W
		XORLW	0X02		; CHECK IF MY STATE IS CHANGED OR NOT
		BTFSS	STATUS,Z
		RETURN				; STATE IS CHANGED SO RETURN TO THE MAIN PROGRAM
							; MAIN PROGRAM HAS LOOKUP STATE TABLE BASED ON THE VALUE OF STATE_DETECT REGISTER
		GOTO	 CLEAR_TEXT?_LOOP    ; STATE IS NOT CHANGED SO CONTINUE WITH THE CURRENT STATE
		
;**************************************************		
		
	
NEC_CLEAR_TEXT?_MESSEGE		
		MOVLW	0x9A		; column 0
		CALL	VID_WRITE
		MOVLW	0XA0		; SECOND LAST LINE
		CALL	VID_WRITE
		
		
		MOVLW	0X20
		CALL	VID_WRITE	;P
		MOVLW	0X62
		CALL	VID_WRITE	;r
		MOVLW	0X55
		CALL	VID_WRITE	;e
		MOVLW	0X63
		CALL	VID_WRITE	;s
		MOVLW	0X63
		CALL	VID_WRITE	;s
		
		MOVLW	0X10
		CALL	VID_WRITE	;
		
		MOVLW	0X15
		CALL	VID_WRITE	;E
		
		MOVLW	0X10
		CALL	VID_WRITE	;
		
		MOVLW	0X24
		CALL	VID_WRITE	;T
		MOVLW	0X00
		CALL	VID_WRITE	;o
		MOVLW	0X10
		CALL	VID_WRITE	;
		
		MOVLW	0X13
		CALL	VID_WRITE	;C
		
		MOVLW	0X5c
		CALL	VID_WRITE	;l
		MOVLW	0X55
		CALL	VID_WRITE	;e
		MOVLW	0X51
		CALL	VID_WRITE	;a
		MOVLW	0X62
		CALL	VID_WRITE	;r
		MOVLW	0X10
		CALL	VID_WRITE	;
		MOVLW	0X24
		CALL	VID_WRITE	;T	
		MOVLW	0X55
		CALL	VID_WRITE	;e
		MOVLW	0X68
		CALL	VID_WRITE	;x
		MOVLW	0X64
		CALL	VID_WRITE	;t	
		
		RETURN
;**************************************************			
NEW_LINE_2	
	MOVLW	0X0A				; THIS IS FOR NEW LINE FEED
	CALL	UART_SEND			; 
	MOVLW	0X0D				; CR
	CALL	UART_SEND
	RETURN	
	
;**************************************************	
HYPE_CLEAR_TEXT?_MESSEGE		
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2			; 10 NEW LINES
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2	
	CALL	NEW_LINE_2	
		
DISLPAY_MESSEGE
	MOVLW	0X43				; C
	CALL	UART_SEND			; 
	MOVLW	0X6C				; l
	CALL	UART_SEND			; 	
	MOVLW	0X65				; e
	CALL	UART_SEND			; 	
	MOVLW	0X61				; a
	CALL	UART_SEND			; 	
	MOVLW	0X72				; r
	CALL	UART_SEND			; 	
		
	MOVLW	0X20				; SP
	CALL	UART_SEND			; 	
		
	MOVLW	0X54				; T
	CALL	UART_SEND			;	
	MOVLW	0X65				; e
	CALL	UART_SEND			;	
	MOVLW	0X78				; x
	CALL	UART_SEND			;	
	MOVLW	0X74				; t
	CALL	UART_SEND			;	
	MOVLW	0X3F				; ?
	CALL	UART_SEND			;	
		
	MOVLW	0X20				; SP
	CALL	UART_SEND			; 	
		
	MOVLW	0X28				; (
	CALL	UART_SEND			; 	
		
	MOVLW	0X59				; Y
	CALL	UART_SEND			; 	
	MOVLW	0X2F				; /
	CALL	UART_SEND			; 	
	MOVLW	0X4E				; N
	CALL	UART_SEND			; 	
	MOVLW	0X29				; )
	CALL	UART_SEND			; 	
		
	RETURN	
		
		
		
		
Y_N_DETECT		
	MOVF	RC_DATA,W
	
	XORLW	0X59				; CHECKS FOR Y	
	BTFSC	STATUS,Z	
	GOTO	YES_PRESSED	
	
	
	MOVF	RC_DATA,W
	XORLW	0X4E				; CHECKS FOR N	
	BTFSC	STATUS,Z	
	GOTO	NO_PRESSED
	
	MOVF	RC_DATA,W
	XORLW	0X79				; CHECKS FOR y---LOWERCASE
	BTFSC	STATUS,Z	
	GOTO	YES_PRESSED
	
	
	MOVF	RC_DATA,W
	XORLW	0X6E				; CHECKS FOR Y	
	BTFSC	STATUS,Z	
	GOTO	NO_PRESSED
	
	
	
	
	RETURN						; NO KEY IS PRESSED	
			
NO_PRESSED		
	MOVLW	0X01				; GOTO STATE 1 ENTER TEXT MODE	
	MOVWF	STATE_DETECT
	RETURN	
		
YES_PRESSED		
	MOVLW	0X12				; GOTO STATE 12 ERASE ALL TEXT MODE
	MOVWF	STATE_DETECT
	RETURN		
	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;											STATE   03
;VGA_LINE_COUNTER							
;**************************************************************************************************************		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************	
;********STACK-X***********	
VGA_LINE_COUNTER
		MOVLW	0X03
		MOVWF	STATE_DETECT	; SET THE STATE 03
		

		CALL  	VID_ERAL	; THIS IS GOING TO ERASE ALL THE GARBAGE CHARACTERS AND PUTS CURSUR AT (0,0) LOCATION
        MOVLW	0X9B	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN
        MOVLW	0X4D	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN        
        
        
        MOVLW	0X96	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	VID_WRITE	; DISPLAY MESSEGE ON THE SCREEN
        MOVLW	0XAA		; NOT APPLICABEL
        CALL	VID_WRITE	; THIS STAGE IS UNDER CONSTRUCTION
        MOVLW	0X1E		;-------\
        CALL	VID_WRITE	;		|
        MOVLW	0X6D		;		|N/A   MESSEGE
        CALL	VID_WRITE	;		|
        MOVLW	0X11		;		|
        CALL	VID_WRITE	;		/
        
        
        			
 VGA_LINE_COUNTER_LOOP       			
        			
		CALL	KEY_PRESS	; CHECKS IF KEY IS PRESSED
		
		MOVF	STATE_DETECT,W
		XORLW	0X03		; CHECK IF MY STATE IS CHANGED OR NOT
		BTFSS	STATUS,Z
		RETURN				; STATE IS CHANGED SO RETURN TO THE MAIN PROGRAM
							; MAIN PROGRAM HAS LOOKUP STATE TABLE BASED ON THE VALUE OF STATE_DETECT REGISTER
		GOTO	VGA_LINE_COUNTER_LOOP; STATE IS NOT CHANGED SO CONTINUE WITH THE CURRENT STATE
					

;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;											STATE   04
;INFRARED_TXRX							
;**************************************************************************************************************		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************	
;********STACK-X***********	
INFRARED_TXRX
		MOVLW	0X04
		MOVWF	STATE_DETECT	; SET THE STATE 04
	

		CALL  	VID_ERAL	; THIS IS GOING TO ERASE ALL THE GARBAGE CHARACTERS AND PUTS CURSUR AT (0,0) LOCATION
        MOVLW	0X8C	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN
       
       
       
       
        MOVLW	0X96	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	VID_WRITE	; DISPLAY MESSEGE ON THE SCREEN
        MOVLW	0XAA		; NOT APPLICABEL
        CALL	VID_WRITE	; THIS STAGE IS UNDER CONSTRUCTION
        MOVLW	0X1E		;-------\
        CALL	VID_WRITE	;		|
        MOVLW	0X6D		;		|N/A   MESSEGE
        CALL	VID_WRITE	;		|
        MOVLW	0X11		;		|
        CALL	VID_WRITE	;		/
        
        
        
        
        
        
INFRARED_TXRX_LOOP        
        CALL	KEY_PRESS	; CHECKS IF KEY IS PRESSED
		MOVF	STATE_DETECT,W
		XORLW	0X04		; CHECK IF MY STATE IS CHANGED OR NOT
		BTFSS	STATUS,Z
		RETURN				; STATE IS CHANGED SO RETURN TO THE MAIN PROGRAM
							; MAIN PROGRAM HAS LOOKUP STATE TABLE BASED ON THE VALUE OF STATE_DETECT REGISTER
		GOTO	INFRARED_TXRX_LOOP; STATE IS NOT CHANGED SO CONTINUE WITH THE CURRENT STATE
		

							
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;											STATE   12
; DELETE_TEXT							
;**************************************************************************************************************		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************	
;********STACK-X***********	
CLEAR_TEXT			
 		MOVLW	0X02
		MOVWF	STATE_DETECT	; AFTER DELETING TEXT I WILL GO TO STATE 02
 									
 
 		CALL  	VID_ERAL	; THIS IS GOING TO ERASE ALL THE GARBAGE CHARACTERS AND PUTS CURSUR AT (0,0) LOCATION
        MOVLW	0XB0	   	; MOV A8 MEMORY LOCATION WHEERE MESSEGE IS SITTING
        CALL  	EE_2_NEC	; DISPLAY MESSEGE ON THE SCREEN
       MOVLW	0X0E
        CALL	VID_WRITE	;...
        MOVLW	0X0E
        CALL	VID_WRITE
        MOVLW	0X0E
        CALL	VID_WRITE
        
        
        CLRW
		MOVWF 	DELAY_1		;		\WAIT FOR 1 SEC		
		CLRW				;		 |
		MOVWF 	DELAY_2		;		 |WAITS FOR 1 SECS
		MOVLW 	0X13			;		 |FOR DELAY ROUTINE YOU HAVE TO PROVIDE VALUE TO THESE REGISTERS			
		MOVWF 	DELAY_0		;		 |CHECK OUT THE EXCEL SHEET NAMED IN DELAY ROUTINE FOR MORE INFO
		CALL 	DELAY			;		/ ON HOW I GOT THE REG VALUES FOR 5 SECS
		
		MOVLW	0X20
		MOVWF	DATA_BYTE
		
		CALL	SER_EE_EWEN
        CALL	SER_EE_WRAL		; EARASE ALL TEXT FROM SER EE PROM	
        CALL	SER_EE_EWDS
        MOVLW	0X01			; GOTO STATE 1
        MOVWF	STATE_DETECT
 		
		RETURN				; 
							; MAIN PROGRAM HAS LOOKUP STATE TABLE BASED ON THE VALUE OF STATE_DETECT REGISTER
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;											STATE   13
;VGA_TRIG							
;**************************************************************************************************************		
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************
;**************************************************************************************************************	
;********STACK-X***********	
VGA_TRIG

		MOVLW	0X03
		MOVWF	STATE_DETECT	; AFTER DELETING TEXT I WILL GO TO STATE 02
		
		
;;THIS STATE IS NOT IMPLEMENTED YET
;;MARK BOYD IS GOING TO IMPLEMENT THIS        
     
		RETURN				; STATE IS CHANGED SO RETURN TO THE MAIN PROGRAM



		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
			
		
		
		


































; THIS ARE TEST ROUTINES 
; ITS NOT INCLUDED IN THE FINALA PROGRAM
; I AM JUST USING THIS PAGE AS SCRATCH PAD


		
		
;			MOVF DATA_ADDR,W					
;		ANDLW 0X0F			; RIGHT NIBLE
;		ADDLW 0X40					
;		CALL	UART_SEND		
;		SWAPF DATA_ADDR,W								
;		ANDLW 0X0F			; LEFT NIBLE
;		ADDLW 0X40					
;		CALL	UART_SEND	




















		END

