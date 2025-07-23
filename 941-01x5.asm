;******************************************************************************
$TITLE (SCC 941-001 version 0.05 AT89S53 MicroController.)
;
;Copyright (c) 1997-2003 Static Controls Corporation
;
;Author:   Rich Schupbach, tune data created by David Schmidt
;          Based on AD-0974-021 version 0.00 code by Greg Robertson
;Company:  for Static Controls Corporation
$DATE      (01-20-03)
;PartNo:   ___-____-___
;
;Version:  0.05<-- Also update VERSION code field below when changing value.
;Assembly: AD-0941-001, Music System
;Location: U2
;
;******************************************************************************
;DESCRIPTION -- SCC AD-0941-001 Music System
;
;  This is the program for the AT89S53 microcontroller which resides on the
;  AD-0941-001 music system circuit board.  The AD-0941-001 is a music system
;  that plays a brief "clip" of each of 255 tunes. The music system hardware 
;  consists of four audio outputs which share a common audio signal. 
;
;  Independent local volume contol is provided in the form of "up" and "down" 
;  SPST momentary push-button switches, one pair per output. Remote volume
;  control and tune selection are supported via SCC Bus commands. The user may
;  select normal tempo or double-time tempo for any given tune.
;
;  The power-up sequence is lamptest, clear, version number display. 
;  Lamptest lasts 1 second. Clear lasts 1 second. Version number remains
;  displayed until the system is commanded otherwise. 
;
;  During lamptest, the LED labeled MICRO, the LED labeled TUNE and the
;  3-digit numeric display lamptest for 1 second. 
;
;  During clear, both LEDs are off and the 3-digit numeric display shows "000".
;
;  After clear, the numeric display shows the firmware version number, "0.0x"
;  and the LED labeled MICRO begins to flash with a period of 1 second and a 
;  50% duty cycle. 
;
;  The AT89S53 internal UART is used to receive commands and data.
;
;  For AD-0941-001 command set, refer to the PARSE subroutine.
;
;
;************************  revision history  **********************************
;ver 0.00 03-12-01 (RLS) Created. (based on AD-0974-021 ver 0.00 code)
;                        This initial release supports the folowing:
;                        - 255 tunes
;                        - Normal tempo and double-time tempo of all tunes
;                        - Common signal shared by 4 audio outputs
;                        - Independent local volume control of 4 audio outputs
;                        - Independent remote volume control of 4 audio outputs
;                        - 3-digit tune number display
;                        - Adjustable baud rate of 9.6k,19.2k,28.8k,57.6k
;
;ver 0.01 05-16-01 (RLS) (1) Bug fix.
;                        Problem:
;                          There was an intermittent problem in which the 
;                          system would sometimes respond to a tune command 
;                          with notes other than those in the LUTs.
;                        Cause:
;                          The DPTR was losing it's place. This appears to
;                          have occurred because the DPTR was being set to 1 in
;                          PLAY_TUNE prior to timer 2 being disabled. If the
;                          timing was just right, the routine called by the
;                          timer 2 overflow (SERV_NOTE_OUT) would change the
;                          DPTR back to 0 before PLAY_TUNE disabled timer 2.
;                        Solution:
;                          MOD the location at which the DPTR is changed 
;                          in the PLAY_TUNE subroutine.
;                          Added a statement to clear timer 2 overflow flag
;                          in the PLAY_TUNE routine.
;                        (2) Changed the name of numerous flags, variables 
;                          and routines for clarity.
;                        (3) Modified operation to turn on "TUNE" LED anytime
;                          a tune (other than zero) is being played. This 
;                          provides a feedback alternative to the numeric
;                          displays and Maxim 7219.
;                          The TUNE LED is solid for normal tempo tunes
;                          and flashing for fast tempo tunes.
;                        (4) Fixed lamptest to always display 0.0.0. on the
;                          numeric display instead of changing to 255 after
;                          the first repetition of tune 255, which is played
;                          during lamptest.
;                        (5) Modified operation of remote volume control.
;                          Version 0.01 continues to support all functions
;                          while processing a remote volume command.
;                          Version 0.00 ignored everything else while
;                          processing the remote volume command.
;                        (6) Modified INIT_UART routine to set PCON to 000h
;                          then added an ACALL to the routine following an
;                          ESC 0 reset command. Previously following an ESC 0
;                          the SMOD bit of the Power Control Register (PCON)
;                          remained set (if set previously). This flag is
;                          used for baud rate doubling (e.g. 9,600 --> 19,200
;                          14,400 --> 28,800, 28,800 --> 57,600). This flag
;                          is now cleared upon an ESC 0 to ensure 9,600.
;                          (This statement was corrected on 06-26-01, from:
;                          "Previously following an ESC 0 the system
;                          defaulted to whatever baud rate was set prior
;                          to the ESC 0. Now it defaults to 9600.")
;
;ver 0.02 06-26-01 (RLS) (1) Bug fix.
;                        Problem:
;                          Version 0.01 ignores received tune commands 
;                          if a "rest" is being played at that time.
;                        Cause:
;                          The value being loaded into Timer 2 for note #49
;                          (i.e. rest) is the cause.
;                        Solution:
;                          Change the value to something acceptable. This
;                          value is not used anyway. The other 48 notes
;                          (i.e. 4 octaves, 12 notes each) represent an output
;                          frequency while selecting the "rest" note #49
;                          results in the output being disabled. While this
;                          is the quickest fix, a new approach may have been
;                          used. Instead of referring to the look-up-table
;                          (LUT) when note 49 is requested, I could have
;                          instead skipped the LUT and disabled the output
;                          directly.
;                        (2) Disabled direct-drive feature because 
;                          garbage on the SCC Bus network at power-up caused
;                          a random tune to play.
;                        (3) Added CLeaR of PLAY_REST flag in PRCT0 and PRCT2.
;                          Added CLeaR of PLAY_REST flag in GN_REST_NO_A.
;                          This corrected a problem in which the first note
;                          of a tune was sometimes skipped during the first
;                          iteration.
;                        (4) Modified lamp test and clear to not affect the
;                          status LED. I prefer not to interrupt "heart beat".
;
;ver 0.03 12-03-01 (RLS) (1) POTENTIAL bug fix.
;                        Problem:
;                          Code for AD-0941-011, version 0.01 sometimes hangs 
;                          while playing a tune. This occurs infrequently.
;                          This problem has not been seen in AD-0941-001
;                          but the troublesome code of AD-0941-011 originated
;                          here.
;                        Cause:
;                          Calling and executing PLAY_TUNE from SERV_NOTE_OUT
;                          takes too long. This was determined through tests
;                          of code for AD-0941-011 version 0.01.
;                        Solution:
;                          Changed SERV_NOTE_OUT to ensure RETI. Moved
;                           check of REPLAY_TUNE flag to main loop.
;                        (2) Performed some cleanup:
;                          Modified/Added PUSH & POPs in following:
;                            - SERV_TIMER0
;                            - SERV_FLASHER
;                            - SERV_NOTE_OUT
;                            - SERV_VOLUME
;                          Eliminated redundant T2MOD assignments throughout.
;                          Changed location of ET2 mod in SNO_3_D, PLAY_TUNE
;                          and PT_5.
;                        (3) Added SPARE_0 & SPARE_1 test outputs.
;
;ver 0.04 01-31-02 (RLS)  Used 954-02x9.asm as a model for these changes:
;                         1) Added Rx and Tx buffers. Previously used only 
;                            SBUF SFR.
;                         2) Added new SCC BUS 'Z' command to control
;                            RECV_DWNSTREAM_ENABLE_n via new control bit
;                            MSTR_RDWN_EN.  When 941-001 transmits to the
;                            headend (i.e. upstream), it disables the
;                            downstream receiver.  Upon completion of
;                            transmitting, it will enable the downstream
;                            receiver ONLY IF MSTR_RDWN_EN has been set by the
;                            host via the new 'Z' command.
;                         3) Power up default state of RECV_DWNSTREAM_ENABLE_n
;                            is now disabled.
;                         4) Added ESC4 return board ID support.
;
;ver 0.05 01-20-03 (RLS)  1) Modified tune # 34 "the Adams' Family" to more
;                             closely resemble older model music systems.
;
;******************************************************************************

;Allowable Target Devices: AT89S53 running from 11.0592 MHz crystal
;******************************************************************************

;******************************************************************************
;MACRO Definitions
;******************************************************************************
;TOGGLE_DPS  MACRO
;               XRL        WMCON, #00000100b       ;Toggle DPS to switch DPTRs.
;               ENDM

SELECT_DPTR0 MACRO
                ANL        WMCON, #11111011b       ;Set bit DPS=0 to sel DPTR0.
                ENDM

SELECT_DPTR1 MACRO
                ORL        WMCON, #00000100b       ;Set bit DPS=1 to sel DPTR1.
                ENDM

;******************************************************************************
;ASM51 Cross-Assembler Controls.
;******************************************************************************
$MOD53
$NODEBUG
$OBJECT
;$NOPAGING
$PAGEWIDTH(132)

;******************************************************************************
;Constant Declarations.
;******************************************************************************

;Define MAX7219 control parameters.
;  MAX7219 control register address definitions.
MAX7219_ADR_NOP             EQU  00h
MAX7219_ADR_DIGIT_0         EQU  01h
MAX7219_ADR_DIGIT_1         EQU  02h
MAX7219_ADR_DIGIT_2         EQU  03h
MAX7219_ADR_DIGIT_3         EQU  04h
MAX7219_ADR_DIGIT_4         EQU  05h
MAX7219_ADR_DIGIT_5         EQU  06h
MAX7219_ADR_DIGIT_6         EQU  07h
MAX7219_ADR_DIGIT_7         EQU  08h
MAX7219_ADR_DECODE_MODE     EQU  09h
MAX7219_ADR_INTENSITY       EQU  0Ah
MAX7219_ADR_SCAN_LIMIT      EQU  0Bh
MAX7219_ADR_SHUTDOWN        EQU  0Ch
MAX7219_ADR_DSPLY_TEST      EQU  0Fh

;  MAX7219 control register data definitions.
MAX7219_DAT_NOP             EQU  00h

MAX7219_DAT_DECODE_ALL_OFF  EQU  00h
MAX7219_DAT_DECODE_ALL_ON   EQU 0FFh

MAX7219_DAT_INTENSITY_MIN   EQU  00h    ;Duty cycle=01/32 (MIN Brightness)
MAX7219_DAT_INTENSITY_03_32 EQU  01h    ;Duty cycle=03/32
MAX7219_DAT_INTENSITY_05_32 EQU  02h    ;Duty cycle=05/32
MAX7219_DAT_INTENSITY_07_32 EQU  03h    ;Duty cycle=07/32
MAX7219_DAT_INTENSITY_09_32 EQU  04h    ;Duty cycle=09/32
MAX7219_DAT_INTENSITY_11_32 EQU  05h    ;Duty cycle=11/32
MAX7219_DAT_INTENSITY_13_32 EQU  06h    ;Duty cycle=13/32
MAX7219_DAT_INTENSITY_15_32 EQU  07h    ;Duty cycle=15/32
MAX7219_DAT_INTENSITY_17_32 EQU  08h    ;Duty cycle=17/32
MAX7219_DAT_INTENSITY_19_32 EQU  09h    ;Duty cycle=19/32
MAX7219_DAT_INTENSITY_21_32 EQU  0Ah    ;Duty cycle=21/32
MAX7219_DAT_INTENSITY_23_32 EQU  0Bh    ;Duty cycle=23/32
MAX7219_DAT_INTENSITY_25_32 EQU  0Ch    ;Duty cycle=25/32
MAX7219_DAT_INTENSITY_27_32 EQU  0Dh    ;Duty cycle=27/32
MAX7219_DAT_INTENSITY_29_32 EQU  0Eh    ;Duty cycle=29/32
MAX7219_DAT_INTENSITY_MAX   EQU  0Fh    ;Duty cycle=31/32 (MAX Brightness)

MAX7219_DAT_SCAN_LIMIT_1    EQU  00h    ;Display Digit 0 only
MAX7219_DAT_SCAN_LIMIT_2    EQU  01h    ;Display Digits 0,1
MAX7219_DAT_SCAN_LIMIT_3    EQU  02h    ;Display Digits 0,1,2
MAX7219_DAT_SCAN_LIMIT_4    EQU  03h    ;Display Digits 0,1,2,3
MAX7219_DAT_SCAN_LIMIT_5    EQU  04h    ;Display Digits 0,1,2,3,4
MAX7219_DAT_SCAN_LIMIT_6    EQU  05h    ;Display Digits 0,1,2,3,4,5
MAX7219_DAT_SCAN_LIMIT_7    EQU  06h    ;Display Digits 0,1,2,3,4,5,6
MAX7219_DAT_SCAN_LIMIT_8    EQU  07h    ;Display Digits 0,1,2,3,4,5,6,7

MAX7219_DAT_SHUTDOWN_ON     EQU  00h
MAX7219_DAT_SHUTDOWN_NORMAL EQU  01h

MAX7219_DAT_DSPLY_TEST_NORM EQU  00h
MAX7219_DAT_DSPLY_TEST_ON   EQU  01h

;Define ASCII control codes.
STX                         EQU  02h    ;Start of Text.
ETX                         EQU  03h    ;End of Text.
EOT                         EQU  04h    ;End of Transmission.
ESC                         EQU  1Bh    ;Escape.

ACK                         EQU  06h    ;ACKnowledge.
NAK                         EQU  15h    ;Negative AcKnowledge.

;Define AD-0941-001 command codes.
CMD_ADR                     EQU  'A'    ;Address assignment cycle command.
CMD_TUN                     EQU  'T'    ;Play tune command.
CMD_URT                     EQU  'U'    ;Set UaRT baud rate command.
CMD_ZTM                     EQU  'Z'    ;Bus terminator (RDWN_EN) cntrl command
CMD_VOL                     EQU  'V'    ;Remote volume command.
CMD_MAX                     EQU  'M'    ;MAX7219 direct write command.

;Define Tune Resolution / LED Flasher Timer (Timer #0) parameters.
TMR0_PERIOD                 EQU  36
TMR0_RLD                    EQU (256-TMR0_PERIOD)

;******************************************************************************
;Variable Declarations (SRAM)
;******************************************************************************
;The AT89S53 has 256 bytes of INTERNAL RAM, assigned as follows:
;  00h-07h  Register bank 0 (R0,R1,..,R7).
;  08h-1Fh  Sometimes used as register banks 1, 2 and 3.
;  20h-2Fh  Bit addressable RAM locations.
;  30h-7Fh  Scratch pad RAM area (direct or indirect addressable)
;  80h-FFh  Scratch pad RAM area (indirect addressable only)

                DSEG   AT  20h          ;Data Memory, read/write.
;Status flags.
STA_FLAGA:      DS         01h          ;Status flags.
STA_FLAGB:      DS         01h          ;Status flags.
STA_FLAGC:      DS         01h          ;Status flags.
STA_FLAGD:      DS         01h          ;Status flags.
STA_FLAGE:      DS         01h          ;Status flags.
 STX_WAS_SEEN    BIT       STA_FLAGA.0  ; STX was previously seen.

 OP_ADR          BIT       STA_FLAGA.1  ; Opcode is for ADDRESS assignment.
 OP_ESC          BIT       STA_FLAGA.2  ; Opcode is for ESCape sequence.
 OP_TUN          BIT       STA_FLAGA.3  ; Opcode is for TUNe control.
 OP_URT          BIT       STA_FLAGA.4  ; Opcode is for UaRT control.
 OP_VOL          BIT       STA_FLAGA.5  ; Opcode is for VOLume control.
 OP_MAX          BIT       STA_FLAGA.6  ; Opcode is for MAX7219 direct write.
 OP_ZTM          BIT       STA_FLAGA.7  ; Opcode is for Bus terminator cntrl

 ;RXRDY           BIT       STA_FLAGB.0  ; UART receiver has byte ready.
 ;TXRDY           BIT       STA_FLAGB.1  ; UART transmitter is available.

 MSTR_RDWN_EN    BIT       STA_FLAGB.0  ; MaSTeR Receive DoWNstream ENable.

 FAST_ON         BIT       STA_FLAGB.2  ; FAST flashing ON state.
 SLOW_ON         BIT       STA_FLAGB.3  ; SLOW flashing ON state.

 TUNE_ENBL       BIT       STA_FLAGB.4  ; TUNE ENaBLe flag.
 PLAY_REST       BIT       STA_FLAGB.5  ; Rest on flag.
 TUNE_OFFSET     BIT       STA_FLAGB.6  ; Tune number offset flag.
                                        ; (true if tune # > 127).
 DBL_TIME        BIT       STA_FLAGC.0  ; DouBLe TIME flag.
 RST_TUNE_ENBL   BIT       STA_FLAGC.1  ; ReSeT TUNE_ENBL after volume
                                        ; adjustment.
 REPLAY_TUNE     BIT       STA_FLAGC.2  ; Replay current tune flag.

 LAMP_TST_FLG    BIT       STA_FLAGC.3  ; LAMP TeST FLaG.

 INC_CHAN_1      BIT       STA_FLAGD.0  ; INCrement CHANnel 1 flag.
 INC_CHAN_2      BIT       STA_FLAGD.1  ; INCrement CHANnel 2 flag.
 INC_CHAN_3      BIT       STA_FLAGD.2  ; INCrement CHANnel 3 flag.
 INC_CHAN_4      BIT       STA_FLAGD.3  ; INCrement CHANnel 4 flag.
 DEC_CHAN_1      BIT       STA_FLAGD.4  ; DECrement CHANnel 1 flag.
 DEC_CHAN_2      BIT       STA_FLAGD.5  ; DECrement CHANnel 2 flag.
 DEC_CHAN_3      BIT       STA_FLAGD.6  ; DECrement CHANnel 3 flag.
 DEC_CHAN_4      BIT       STA_FLAGD.7  ; DECrement CHANnel 4 flag.

 UART_TX_EMT     BIT       STA_FLAGE.0  ; UART transmit buffer is available.

 RX_BFR_FUL      BIT       STA_FLAGE.1  ; Receive BuFfeR FULl.
 RX_BFR_EMT      BIT       STA_FLAGE.2  ; Receive BuFfeR EMpTy.
 RX_BFR_OVF      BIT       STA_FLAGE.3  ; Receive BuFfeR OVerFlow.

 TX_BFR_FUL      BIT       STA_FLAGE.4  ; Transmit BuFfeR FULl.
 TX_BFR_EMT      BIT       STA_FLAGE.5  ; Transmit BuFfeR EMpTy.
 TX_BFR_OVF      BIT       STA_FLAGE.6  ; Transmit BuFfeR OVerFlow.
 TX_ID_STR       BIT       STA_FLAGE.7  ; Transmit PCB ID string request.

TUNE_VAL:       DS         01h          ;8-BIT TUNE data storage.

;                ORG        30h
REC_BYTE:       DS         01h          ;Byte received by the UART from upstrm.
REC_BYTE_1:     DS         01h          ;Received byte storage.
REC_BYTE_2:     DS         01h          ;Received byte storage.
REC_BYTE_3:     DS         01h          ;Received byte storage.
REC_BYTE_4:     DS         01h          ;Received byte storage.
;REC_BYTE_5:    DS         01h          ;Received byte storage.
REC_BYTE_6:     DS         01h          ;Received byte storage.
REC_BYTE_7:     DS         01h          ;Received byte storage.
REC_BYTE_8:     DS         01h          ;Received byte storage.
REC_BYTE_9:     DS         01h          ;Received byte storage.
REC_BYTE_10:    DS         01h          ;Received byte storage.
;REC_BYTE_11:   DS         01h          ;Received byte storage.

LOC_AD1:        DS         01h          ;Local address high nibble (binary).
LOC_AD0:        DS         01h          ;Local address  low nibble (binary).
LOC_DEV_AD1:    DS         01h          ;Local DEVICE SET address hi nibble.
LOC_DEV_AD0:    DS         01h          ;Local DEVICE SET address lo nibble.
ID_INDEX:       DS         1            ;index counter for xmit of PCB ID.

SW_FAST_TIMER:  DS         01h          ;Software FAST timer.
TMRF_PRD        EQU        25           ;Software FAST timer period.

SW_SLOW_TIMER:  DS         01h          ;Software SLOW timer.
TMRS_PRD        EQU       (TMRF_PRD*2)  ;Software SLOW timer period.

DELAY_1_TIMER:  DS         01h          ;Delay timer.

;Define PARSE machine state variable.
PAR_STATE:      DS         01h          ;

TX_BFR_LEN      EQU        4            ;serial Transmit BuFfer LENgth.
TX_BFR:         DS  TX_BFR_LEN          ;serial Transmit BuFfeR.

TX_BFR_RDPTR:   DS         1            ;Transmit BuFfeR ReaD PoinTeR.
TX_BFR_WRPTR:   DS         1            ;Transmit BuFfeR WRite PoinTeR.

RX_BFR_LEN      EQU        15           ;serial Receive BuFfer LENgth.
RX_BFR:         DS  RX_BFR_LEN          ;serial Receive BuFfeR.

RX_BFR_WRPTR:   DS         1            ;Receive BuFfeR WRite PoinTeR.
RX_BFR_RDPTR:   DS         1            ;Receive BuFfeR ReaD PoinTeR.

;Define tune LUT variables.
NOTE_COUNT:     DS         01h          ;The # of notes in the tune.
NOTE_DURATION:  DS         01h          ;The current note duration in
                                        ;  10ms increments.
;Define volume control variables.
VOLUME_TIMER:   DS          01h         ;VOLUME TIMER.
VOLUME_PRD      EQU           6         ;VOLUME timer PeRioD.

REM_VOL_CNTR1:  DS         01h          ;REMote VOLume CouNTeR 1.
REM_VOL_CNTR2:  DS         01h          ;REMote VOLume CouNTeR 2.
REM_VOL_CNTR3:  DS         01h          ;REMote VOLume CouNTeR 3.
REM_VOL_CNTR4:  DS         01h          ;REMote VOLume CouNTeR 4.

CH1_COUNT:      DS         01h          ;CHannel 1 volume inc/dec COUNT.
CH2_COUNT:      DS         01h          ;CHannel 2 volume inc/dec COUNT.
CH3_COUNT:      DS         01h          ;CHannel 3 volume inc/dec COUNT.
CH4_COUNT:      DS         01h          ;CHannel 4 volume inc/dec COUNT.

;Store data and attributes of 3 BCD digits.
DAT_DIG1:       DS         01h          ;Data, digit 1. (Left most.)
DAT_DIG2:       DS         01h          ;Data, digit 2.
DAT_DIG3:       DS         01h          ;Data, digit 3. (Right most.)
ATR_DIG:        DS         01h          ;Digit attributes (apply to all 3 dig)

                ORG        60h          ;stack origin
STACK:          DS         20h          ;stack depth

;******************************************************************************
;I/O Port Pin Declarations.
;******************************************************************************
;Definitions for MAX489 Transceivers.
DATA_FROM_HEADEND       BIT     P3.0    ;input  (RXD to UART).
DATA_TO_HEADEND         BIT     P3.1    ;output (TXD from UART).
RECV_DWNSTREAM_ENABLE_n BIT     P3.6    ;output (active low).
XMIT_DWNSTREAM_ENABLE   BIT     P3.7    ;output.

;Definitions for MAX7219 control.
MAX7219_DATA_LOAD       BIT     P1.2    ;output.
MAX7219_DATA0           BIT     P1.1    ;output.
MAX7219_DATA_CLOCK      BIT     P1.0    ;output.

;Definition for status LED.
STATUS_LED_n            BIT     P3.5    ;output (active low).

;Definition for tune LED.
TUNE_LED_n              BIT     P3.4    ;output (active low).

;Definition for spare outputs, used for testing.
SPARE_0                 BIT     P3.2    ;test output 0.
SPARE_1                 BIT     P3.3    ;test output 1.

;Definition for tune output.
TUNE_MICRO_OUT          BIT     P1.3    ;output.

;Definition for volume control outputs.
INC_VOL1_n             BIT     P0.0    ;output (active low).
DEC_VOL1_n             BIT     P0.1    ;output (active low).
INC_VOL2_n             BIT     P0.2    ;output (active low).
DEC_VOL2_n             BIT     P0.3    ;output (active low).
INC_VOL3_n             BIT     P0.4    ;output (active low).
DEC_VOL3_n             BIT     P0.5    ;output (active low).
INC_VOL4_n             BIT     P0.6    ;output (active low).
DEC_VOL4_n             BIT     P0.7    ;output (active low).

;Definition for volume control inputs.
INC_BTN1_n              BIT     P2.0    ;input (active low).
DEC_BTN1_n              BIT     P2.1    ;input (active low).
INC_BTN2_n              BIT     P2.2    ;input (active low).
DEC_BTN2_n              BIT     P2.3    ;input (active low).
INC_BTN3_n              BIT     P2.4    ;input (active low).
DEC_BTN3_n              BIT     P2.5    ;input (active low).
INC_BTN4_n              BIT     P2.6    ;input (active low).
DEC_BTN4_n              BIT     P2.7    ;input (active low).

;******************************************************************************
;                              BEGIN CODE SEGMENT
;******************************************************************************
                CSEG                               ;Program Memory, read-only.

;Reset and interrupt vectors.
                ORG        0000h                   ;power on/reset vector
                AJMP       RESET

                ORG        0003h                   ;external interrupt 0 vector
                RETI                               ;undefined

                ORG        000Bh                   ;Timer 0 overflow vector
FLASH_TMR_INT:  AJMP       SERV_TIMER0

                ;The code of vector above no longer overlaps into 13h.
;               ORG        0013h                   ;external interrupt 1 vector
;               RETI                               ;undefined

                ORG        001Bh                   ;Timer 1 overflow vector
                RETI                               ;undefined

                ORG        0023h                   ;serial I/O interrupt vector
                AJMP       SERV_UART_INT           ;Service UART INTerrupt.

                ORG        002Bh                   ;Timer 2 overflow vector.
                AJMP       SERV_NOTE_OUT           ;Service output note.

                ORG        0060h                   ;S/W version display string.

ASSEMBLY:       DB         'AD-0941-001 '          ;note no 0 terminator
VERSION_LONG:   DB         'v0.05',0               ;0 indicates end of string.
VERSION_SHORT:  DB         'VA05',0                ;0 indicates end of string.
                                                   ;'A' is 0 w/ decimal point.

;******************************************************************************
;                                 MAIN PROGRAM
;******************************************************************************
                ORG        0080h                   ;begin code space
                USING      0                       ;register bank zero

RESET:          ;The following resets are only done once per Power On Reset.
                MOV        SP,#(STACK-1)           ;initialize stack pointer

                ACALL      INIT_PORTS              ;Initialize I/O ports.
                ACALL      INIT_UART               ;Init baud generator & UART.
                ACALL      INIT_FLASHER            ;Init LED flasher.
                ACALL      INIT_VOL_TMR            ;Init VOLume TiMeR.
                                                   ;  for local volume control.

                SETB       EA                      ;Enable UART & TMR 0 intrpt.

                ;The following resets occur at P.O.R. or if ESC 0 is received.
  ESC0_RESET:   CLR        ES                      ;Disable UART interrupt.
                                                   ;  On reentry, disable UART,
                                                   ;  but not the flash timer.

                ACALL      INIT_MAX7219            ;Initialize 5x7 LED driver.
                ACALL      INIT_SOFTWARE           ;Init flags and variables.
                ACALL      INIT_SCC_BUS            ;Init SCC bus addresses.
                ACALL      PARSE_RESET             ;Reset the PARSER.
                ACALL      INIT_TX_BFR             ;Reset the transmit buffer.

                ACALL      LAMP_TEST               ;Test the output lamps.
                ACALL      DELAY_1_SEC             ;Delay.

                ACALL      LAMP_CLR                ;Clear the output lamps.
                ACALL      DELAY_1_SEC             ;Delay.

                ACALL      INIT_FLASHER            ;Re-Init LED flasher.

                ;Display the firmware version number.
                MOV        DPTR,#VERSION_SHORT     ;Point to message string.
                ACALL      MESS                    ;Display the message.

                ;The following 1/4 second delay is provided in order to allow
                ;any additional bytes to be received by the UART and then
                ;discarded (via CLR RI).  This is done because when PROCOMM is
                ;used to send an ESC0 command, it usually sends extra
                ;characters at the end of the packet.  These extra characters
                ;will immediately take us into DIRECT DRIVE mode which will
                ;overwrite any message on the display from the reset operation.
                ;ACALL      DELAY_QRT_SEC           ;Delay.
                ;CLR        RI                      ;Clear UART receive flag.

                ACALL      INIT_RX_BFR             ;Reset the receive buffer.
                SETB       ES                      ;Enable UART interrupt.

                ACALL      INIT_UART               ;Re-init baud gen & UART.

MAIN_LOOP:      ;Service UART receive buffer.
                JB         RX_BFR_EMT, MNLP1       ;JMP if rec buffer is empty
                LCALL      RD_BYTE_RX_BFR          ;Else get byte from the bufr
                ACALL      PARSE                   ;and call the PARSEr.

  MNLP1:        ;Service UART transmit buffer.
                JNB        UART_TX_EMT, MNLP3      ;JMP if transmitter is busy.
                JB         TX_ID_STR, MNLP1A       ;JMP if ID string requested.
                JB         TX_BFR_EMT, MNLP2       ;JMP if nothing to transmit.

    MNLP1A:     ;Disable downstream receiver and transmit next byte.
                SETB       RECV_DWNSTREAM_ENABLE_n ;Disable input from downstr.
                LCALL      TX_BYTE                 ;Transmit next byte.
                CLR        UART_TX_EMT             ;Flag UART xmitter as busy.
                AJMP       MNLP3                   ;Continue.

  MNLP2:        ;IF MSTR_RDWN_EN=1, THEN enable downstream receiver.
                JNB        MSTR_RDWN_EN, MNLP3
                CLR        RECV_DWNSTREAM_ENABLE_n ;Enable input from downstr.

  MNLP3:        MOV        A, SW_FAST_TIMER        ;Service LED flasher.
                JNZ        MNLP4
                ACALL      SERV_FLASHER

  MNLP4:        MOV        A, VOLUME_TIMER         ;Service volume.
                JNZ        MNLP5
                ACALL      SERV_VOLUME

  MNLP5:        JNB        REPLAY_TUNE, MNLP6      ;Play it again.
                CLR        REPLAY_TUNE             ;Re-init flag.
                LCALL      PLAY_TUNE               ;  Flag was set in
                                                   ;  SERV_NOTE_OUT routine.
  MNLP6:        AJMP       MAIN_LOOP

;******************************************************************************
;                          BEGINNING OF SUBROUTINES
;******************************************************************************

;******************************************************************************
;                          Subroutine: SERV_UART_INT
;******************************************************************************
;Service the UART interrupt.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing (PUSH/POPs everything that it uses)
;
;This code was moved here from the interrupt vector area because with the
;addition of the Timer 2 interrupt vector, the following code is too big
;(15 bytes) to fit into the available (8 byte) space at address 0023h.
;
;In the AT89S53, in addition to RI and TI, SPIF is also ORed into the ES
;interrupt.  Since it is not used as of this writing, it is not tested below.
;
SERV_UART_INT:

UART_INT:       ;The UART interrupt is the logical OR of the Receive and
                ;Transmit interrupt flags.  They need to be sorted out.

  TEST_UART_RX: JNB        RI, TEST_UART_TX        ;JMP if not UART RECVR INT.
  SERV_UART_RX: CLR        RI                      ;Clear receive flag.
                LCALL      WR_BYTE_RX_BFR          ;Post byte to RX buffer.

  TEST_UART_TX: JNB        TI, SERV_U_EXIT         ;JMP if not UART XMIT INT.
  SERV_UART_TX: CLR        TI                      ;Clear transmit flag.
                SETB       UART_TX_EMT             ;Set transmitter ready flag.

  SERV_U_EXIT:  RETI


;******************************************************************************
;                       Subroutine: Initialize I/O Ports
;******************************************************************************
;Initialize the I/O pins of the microcontroller.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
INIT_PORTS:     ;Initialize RS-422 transceivers.
                SETB       DATA_FROM_HEADEND       ;Set for input mode.

                SETB       DATA_TO_HEADEND         ;"tri-state" (input mode).

                SETB       XMIT_DWNSTREAM_ENABLE   ;Enable output to downstrm.
;               CLR        XMIT_DWNSTREAM_ENABLE   ;Disable output to downstrm.

;               CLR        RECV_DWNSTREAM_ENABLE_n ;Enable input from downstr.
                SETB       RECV_DWNSTREAM_ENABLE_n ;Disable input from downstr.
                CLR        MSTR_RDWN_EN            ;Disable master ctl bit too

                ;Initialize MAX7219 3-wire serial interface.
                CLR        MAX7219_DATA_LOAD       ;Clear to 0.
                CLR        MAX7219_DATA0           ;Clear to 0.
                CLR        MAX7219_DATA_CLOCK      ;Clear to 0.

                RET

;******************************************************************************
;                            Subroutine: INIT_UART
;******************************************************************************
;Initialize baud rate generator and UART.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A
;
INIT_UART:
  ;Configure the Baud Rate generator.
  ;  Use Timer 1 in mode 2 (8-bit auto-reload).
                MOV        TH1, #0FDh ;set reload value for     9,600 baud.
                MOV        PCON,#000h ;ensure baud rate not doubled.
;               MOV        PCON,#080h ;double the baud rate to 19,200 baud.

;               MOV        TH1, #0FEh ;set reload value for    14,400 baud.
;               MOV        PCON,#080h ;double the baud rate to 28,800 baud.

;               MOV        TH1, #0FFh ;set reload value for    28,800 baud.
;               MOV        PCON,#080h ;double the baud rate to 57,600 baud.

                MOV        TL1, #0FDh ;TL1 impacts first character only.

                MOV        A, TMOD
                ANL        A, #0Fh    ;Mask off timer 0 nibble.
                ORL        A, #20h    ;Set Timer 1 to mode 2.
                MOV        TMOD, A
                SETB       TR1        ;Start the timer.

  ;Configure the UART.
                CLR        SM0        ;Set serial port to mode 1 (8-bit UART).
                SETB       SM1        ; "    "      "  "   "   "   "     "
                CLR        SM2        ;Disable multiprocessor mode.

                CLR        RI         ;Clear receive interrupt flag.
                CLR        TI         ;Clear transmit interrupt flag.

                SETB       UART_TX_EMT             ;Set transmitter ready flag.

                SETB       ES         ;Enable serial port interrupts.
                SETB       REN        ;Enable serial port reception.

                RET

;******************************************************************************
;                           Subroutine: INIT_FLASHER
;******************************************************************************
;Initialize LED Flasher Timer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A
;
;Uses Timer #0 in mode #1 (16-bit timer).  Interrupt is used.
;When interrupt is serviced, the timer is reloaded via software and software
;counters are decremented.
;
;The number of oscillator ticks per interrupt period is:
;
;  12 * 2^8 * 36 = 110,592
;
;where:
;  12 ------- is built in OSC divider;
;  2^8=256 -- is 8-bit LSBs which I treat as a prescaler;
;  36 ------- is number of counts for 8-bit MSBs TH0 portion of timer.
;
;With an 11.0592 MHz oscillator the interrupt frequency is:
;  11.0592 MHz / [12 * 256 * 36] = 100.0 Hz.
;  This provides an interrupt every 1/100 Hz = 10 ms.
;  This value of 10 ms is used as the "resolution" of tune duration.
;
;This can then be divided down with software counters to yield 2 Hz and 4 Hz
;timers.  The flash rates are then 1/2 these values (i.e. 1 Hz and 2 Hz).
;
INIT_FLASHER:   CLR        ET0                     ;Disable timer 0 interrupt.

                MOV        A, TMOD
                ANL        A, #0F0h                ;Mask off Timer 1 nibble.
                ORL        A, #001h                ;Set Timer 0 to mode 1.
                MOV        TMOD, A

                MOV        TL0, #00                ;Load 8-bit LSBs.
                MOV        TH0, #TMR0_RLD          ;Load 8-bit MSBs.

                MOV        SW_FAST_TIMER,#TMRF_PRD ;Init FAST software timer.
                MOV        SW_SLOW_TIMER,#TMRS_PRD ;Init SLOW software timer.

                SETB       FAST_ON
                SETB       SLOW_ON

                SETB       TR0                     ;Start the timer.
                SETB       ET0                     ;Enable Timer 0 interrupt.

                RET

;******************************************************************************
;                            Subroutine: INIT_SOFTWARE
;******************************************************************************
;Initialize baud rate generator and UART.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A
;
INIT_SOFTWARE:  CLR        STX_WAS_SEEN            ;STX has never been seen.

                SELECT_DPTR0                       ;Init to "standard" DPTR

                ;INIT Timer 2.

                ;Disable timer 2 interrupt.
                CLR        ET2

                ;Clear all timer 2 control bits.
                ;  Among other things, this will stop the timer (TR2) and
                ;  cancel any pending timer 2 interrupt (TF2).
                MOV        T2CON, #00

                ;Setup Timer2 in 16-bit Auto Reload mode.
                MOV        T2MOD, #00
 
                CLR        TUNE_ENBL               ;Init flag to tunes off.
                CLR        PLAY_REST               ;Init flag to rest off.
                CLR        TUNE_MICRO_OUT          ;Init to no tune out.
                CLR        DBL_TIME                ;Init double-time flag off.
                CLR        RST_TUNE_ENBL           ;Init ReSeT TUNE_ENBL.
                                                   ; This is set when TUNE_ENBL
                                                   ; was on prior to a volume
                                                   ; adjustment during which 
                                                   ; it was disabled temporarily.

                CLR        REPLAY_TUNE             ;Init replay current tune flag 
                CLR        LAMP_TST_FLG            ;Init lamp test flag to off.

                CLR        INC_CHAN_1              ;Init channel 1 increment off.
                CLR        INC_CHAN_2              ;Init channel 2 increment off.
                CLR        INC_CHAN_3              ;Init channel 3 increment off.
                CLR        INC_CHAN_4              ;Init channel 4 increment off.
                CLR        DEC_CHAN_1              ;Init channel 1 decrement off.
                CLR        DEC_CHAN_2              ;Init channel 2 decrement off.
                CLR        DEC_CHAN_3              ;Init channel 3 decrement off.
                CLR        DEC_CHAN_4              ;Init channel 4 decrement off.

                CLR        TX_ID_STR               ;Clr flag to transmit PCB ID

                RET

;******************************************************************************
;                           Subroutine: INIT_SCC_BUS
;******************************************************************************
;Initialize the SCC bus addresses (i.e. the internal PCB addresses).
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
INIT_SCC_BUS:   MOV        LOC_AD1,     #0Fh       ;Preset local address to FFh
                MOV        LOC_AD0,     #0Fh       ;  Note: binary NOT ascii.

                MOV        LOC_DEV_AD1, #0Fh       ;Preset DEVICE SET to FFh
                MOV        LOC_DEV_AD0, #0Fh       ;  Note: binary NOT ascii.
                RET

;******************************************************************************
;                           Subroutine: INIT_TX_BFR
;******************************************************************************
;Initialize the serial data transmit buffer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
INIT_TX_BFR:    CLR        TX_BFR_FUL
                SETB       TX_BFR_EMT
                CLR        TX_BFR_OVF

                MOV        TX_BFR_WRPTR, #0
                MOV        TX_BFR_RDPTR, #0

                RET

;******************************************************************************
;                           Subroutine: INIT_RX_BFR
;******************************************************************************
;Initialize the serial data receive buffer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
INIT_RX_BFR:    CLR        RX_BFR_FUL
                SETB       RX_BFR_EMT
                CLR        RX_BFR_OVF

                MOV        RX_BFR_WRPTR, #0
                MOV        RX_BFR_RDPTR, #0

                RET

;******************************************************************************
;                           Subroutine: INIT_VOL_TMR
;******************************************************************************
;Initialize LED Flasher Timer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A
;
;Initialize the volume timer. This initialization value dictates
;  how much time elapses between services of the local volume input buttons
;  and remote volume serial commands.
;
INIT_VOL_TMR:     MOV        VOLUME_TIMER,#VOLUME_PRD;Init FAST software timer.


                  RET

;******************************************************************************
;                           Subroutine: INIT_MAX7219
;******************************************************************************
;Initialize the MAX7219 LED Driver.  All three 7-segment digits are driven by
;a single MAX7219 driver.  
;
INIT_MAX7219:
  ;Initialize the 3-wire serial interface.
                CLR        MAX7219_DATA_LOAD       ;Clear to 0.
                CLR        MAX7219_DATA0           ;Clear to 0.
                CLR        MAX7219_DATA_CLOCK      ;Clear to 0.

  ;Perform complete setup of MAX7219.
                ;Enter shutdown mode.
                MOV        A, #MAX7219_ADR_SHUTDOWN
                MOV        R0,#MAX7219_DAT_SHUTDOWN_ON
                ACALL      MAX7219_WRITE

                ;Clear all pixel RAM locations (8 registers).
                MOV        A, #MAX7219_ADR_DIGIT_0
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                MOV        A, #MAX7219_ADR_DIGIT_1
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                MOV        A, #MAX7219_ADR_DIGIT_2
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                MOV        A, #MAX7219_ADR_DIGIT_3
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                MOV        A, #MAX7219_ADR_DIGIT_4
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                MOV        A, #MAX7219_ADR_DIGIT_5
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                MOV        A, #MAX7219_ADR_DIGIT_6
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                MOV        A, #MAX7219_ADR_DIGIT_7
                MOV        R0,#00h
                ACALL      MAX7219_WRITE

                ;Set for non 7-segment decoding.
                MOV        A, #MAX7219_ADR_DECODE_MODE
                MOV        R0,#MAX7219_DAT_DECODE_ALL_OFF
                ACALL      MAX7219_WRITE

                ;Set intensity to < HALF brightness.
                MOV        A, #MAX7219_ADR_INTENSITY
                MOV        R0,#MAX7219_DAT_INTENSITY_13_32
                ACALL      MAX7219_WRITE

                ;Set scan limit to full (8 columns)
                MOV        A, #MAX7219_ADR_SCAN_LIMIT
                MOV        R0,#MAX7219_DAT_SCAN_LIMIT_8
                ACALL      MAX7219_WRITE

                ;End display test mode.
                MOV        A, #MAX7219_ADR_DSPLY_TEST
                MOV        R0,#MAX7219_DAT_DSPLY_TEST_NORM
                ACALL      MAX7219_WRITE

                ;Exit shutdown mode (enable display).
                MOV        A, #MAX7219_ADR_SHUTDOWN
                MOV        R0,#MAX7219_DAT_SHUTDOWN_NORMAL
                ACALL      MAX7219_WRITE

                RET

;******************************************************************************
;                          Subroutine: MAX7219_WRITE
;******************************************************************************
;Write data byte to MAX7219 LED driver.
;
;  PARAMETERS:
;    RECEIVES:
;      A --- ADDRESS of MAX7219 register to be updated.
;      R0 -- DATA value to be written to MAX7219 register.
;    RETURNS:
;      nothing
;      destroys A, R7
;
;The byte is embedded into a 16-bit packet.  The packet contains:
;
;              D15-D12 -- 4 bits of don't cares (MSBs),
;              D11-D08 -- 4 bits of address,
;              D07-D00 -- 8 bits of data        (LSBs).
;
;The packet is sent serially, MSB first.
;
MAX7219_WRITE:  
                CLR        MAX7219_DATA_CLOCK      ;Setup the control lines
                CLR        MAX7219_DATA0           ;

  ;Send 8 MSBs to MAX7219 (MSB first).
                MOV        R7,#8                   ;Init loop counter.

  MAX7219_LOOP1:RLC        A                       ;Rotate MSB into CARRY flag.
                MOV        MAX7219_DATA0,C         ;Send data bit to port.
                SETB       MAX7219_DATA_CLOCK      ;Clock data bit into MAX7219
                CLR        MAX7219_DATA_CLOCK
                DJNZ       R7,MAX7219_LOOP1        ;Repeat til 8 bits are done.

  ;Send 8 LSBs to MAX7219 (MSB first).
                MOV        A, R0                   ;Put DATA byte into ACC.
                MOV        R7,#8                   ;Init loop counter.

  MAX7219_LOOP2:RLC        A                       ;Rotate MSB into CARRY flag.
                MOV        MAX7219_DATA0,C         ;Send data bit to port.
                SETB       MAX7219_DATA_CLOCK      ;Clock data bit into MAX7219
                CLR        MAX7219_DATA_CLOCK
                DJNZ       R7,MAX7219_LOOP2        ;Repeat til 8 bits are done.

  ;Now finish the MAX7219 programming cycle for this register quartet.
                SETB       MAX7219_DATA_LOAD       ;Pulse MAX7219 load cntrl hi
                CLR        MAX7219_DATA_LOAD       ;...then back low.

                RET

;******************************************************************************
;                          Subroutine: SERV_NOTE_OUT
;******************************************************************************
;Service the TIMER 2 Overflow interrupt.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
;The Timer 2 interrupt is the logical OR of the TF2 and EXF2 flags.  However,
;since I do not use EXF2, the interrupt will always be due to TF2 (timer2
;overflow).
;
;Since TF2 occurs so frequently, it is important to minimize the time it takes
;to service it.
;
;This routine is called upon a timer 2 overflow. It's primary function is
;to toggle the audio output at the appropriate rate to get the desired
;output frequency (i.e. note or pitch) which is predetermined by the timer 2
;overflow value. 
;
;If a tune is not currently playing or a "rest" is desired
;(i.e. silence or output frequency = 0), this routine takes
;the output low and exits immediately. Otherwise it checks the remaining
;duration of the currently playing note. If it is not zero, then
;the output is toggled. If it is zero, the next note is determined. 
;
;The next note is either (1)the next note in the LUT for the currently playing
;tune, (2)a mandatory 1 second rest inserted by this routine at the end of a
;tune or - if we have completed the 1 second rest - (3)the first note of the
;currently playing tune because it needs to be replayed.
;
;If option 3 is required, this routine "cleans up" and then calls PLAY_TUNE,
;the same routine that was called when the tune initially began playing.
;
SERV_NOTE_OUT:  
                CLR        TF2                     ;Clear timer 2 overflow flag
                ;Save context for clean return from interrupt.
                PUSH       ACC
                PUSH       PSW

    SNO_1:      ;Check if rest flag is set. If so, disable output and exit.
                ;If rest flag is not set, check if output enable flag is set.
                ;If so, process note. If not, disable output and exit.
                JNB         PLAY_REST, SNO_1_A
                AJMP        SNO_1_B

      SNO_1_A:  JB          TUNE_ENBL, SNO_2        ;Jump & proc if flg set
                
      SNO_1_B:  CLR         TUNE_MICRO_OUT          ;  else kill tune output
                AJMP        SNO_5                   ;  and bail.


    SNO_2:      ;Change DPTR.
                ;Check remaining duration of currently playing note.
                ;If <> 0, continue note by toggling the output then exit
                ;  else disable timer 2 and determine the next note.
                SELECT_DPTR1                       ;Select local DPTR

                MOV         A, NOTE_DURATION
                CJNE        A, #00, SNO_4          ;Jump if note !ended.

                CLR        ET2                     ;Disable timer 2 interrupt.
                MOV        T2CON, #00              ;Clear timer 2 control bits
                ;Among other things, this will stop the timer (TR2) and
                ;  cancel any pending timer 2 interrupt (TF2).

                ;MOV        T2MOD, #00              ;Clear the mode bits

    SNO_3:      ;Check note count. 
                ;If = 1, all notes have been played. Insert 1sec rest
                ;  else if = 0, start tune over again
                ;  else continue current tune by getting the next note.
                MOV        A, NOTE_COUNT
                CJNE       A, #01, SNO_3_B

      SNO_3_A:  ;A=01 so we have played all notes of this tune.
                ;  Time for 1 second rest before starting over.
                ;  Kill output and turn on the PLAY_REST flag.
                ;  This routine will not toggle the output while
                ;  the PLAY_REST flag is on. The flag will be
                ;  cleared in the SERV_TIMER0 routine after the
                ;  duration set below expires.

                CLR        TUNE_MICRO_OUT
                SETB       PLAY_REST

                ;Set duration to 1 second.
                MOV        A, #100
                MOV        NOTE_DURATION, A

                ;Prepare for next Timer2 service.
                MOV        A, #00h
                MOV        NOTE_COUNT,A

                AJMP       SNO_3_D

      SNO_3_B:  ;NOTE_COUNT <> 01 so check to see if = 0
                ;  which would mean that the 1 second REST
                ;  has already been played and it is time
                ;  to start this tune over at the first note.

                CJNE       A,#00,SNO_3_C
                SETB       REPLAY_TUNE
                CLR        TF2                     ;Clear timer 2 overflow flag
                CLR        ET2                     ;Disable timer 2 interrupt.
                MOV        T2CON, #00              ;Clear timer 2 control bits
                ;Among other things, this will stop the timer (TR2) and
                ;  cancel any pending timer 2 interrupt (TF2).

                ;MOV        T2MOD, #00              ;Clear the mode bits
                AJMP       SNO_5

      SNO_3_C:  ;NOTE_COUNT <> 1 and <> 0 so just process the
                ;  next note.

                LCALL      GET_NOTE

                MOV        A,NOTE_COUNT             
                DEC        A
                MOV        NOTE_COUNT,A

                INC        DPTR                    ;Enter Timer2 at
                                                   ;  correct DPTR value.
      SNO_3_D:  ;Re-enable Timer 2.
                SETB       ET2                     ;Enable timer interrupt.
                SETB       TR2                     ;Start the timer.

                AJMP       SNO_5

    SNO_4:      ;Toggle output.
                JB         TUNE_MICRO_OUT, SNO_4_A ;
                SETB       TUNE_MICRO_OUT          ;
                AJMP       SNO_5                   ;

      SNO_4_A:  CLR        TUNE_MICRO_OUT          ;

    SNO_5:      ;Return to standard DPTR and check replay flag.
                SELECT_DPTR0                       ;Return to "standard" DPTR

    SNO_6:      ;Restore the context that existed prior to the interrupt.
                POP        PSW
                POP        ACC

    SNO_X:      
                RETI

;******************************************************************************
;                           Subroutine: SERV_FLASHER
;******************************************************************************
;Service the LED Flasher Timer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, C
;
SERV_FLASHER:   ;Save context for clean return from interrupt.
                PUSH       ACC
                PUSH       PSW

                ;Service the FAST flashing devices.
                MOV        SW_FAST_TIMER,#TMRF_PRD ;Reload FAST software timer.
                CPL        FAST_ON                 ;Toggle FAST devices.

  SF1:          ;Service the SLOW flashing devices.
                MOV        A, SW_SLOW_TIMER
                JNZ        SFX
                MOV        SW_SLOW_TIMER,#TMRS_PRD ;Reload SLOW software timer.
                CPL        SLOW_ON                 ;Toggle SLOW devices.

                MOV        C, SLOW_ON
                CPL        C
                MOV        STATUS_LED_n, C         ;Toggle the STATUS LED.
             
  SF2:          ;Slow flash "TUNE" LED if double-time tune is playing,
                ; but only if tune value is not zero (which is off).
                JNB        DBL_TIME, SF3
                MOV        A, TUNE_VAL
                JZ         SF3
                MOV        C, SLOW_ON
                CPL        C
                MOV        TUNE_LED_n, C           ;Toggle the TUNE LED.

  SF3:          ;Check to see if ANY SLOW flash flag bits are set.
                AJMP       SFX                     ;else skip SLOW service.

  SFX:          ;Restore the context that existed prior to the interrupt.
                POP        PSW
                POP        ACC
                RET

;******************************************************************************
;                           Subroutine: SERV_VOLUME
;******************************************************************************
;Service the LED Flasher Timer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
;The volume period value (VOLUME_PRD) dictates how much time elapses
;between services of the local volume input buttons and remote volume
;serial commands.
;
;Every x times timer 0 overflows (x = VOLUME_PRD), this routine is
;serviced. Remote volume control has priority over local volume control.
;If the digital pots are being incremented or decremented, local volume
;inputs are ignored and SERV_REM_VOL is called.
;
;When local volume control inputs are to be processed, SERV_LOC_VOL is called.
;
SERV_VOLUME:
                ;Save context for clean return from interrupt.
                PUSH       ACC
                PUSH       PSW

                MOV        VOLUME_TIMER, #VOLUME_PRD  ;Reload VOL software timer.

                ;If remote volume is being adjusted, skip local volume service.
                JB         INC_CHAN_1, SV_1A
                JB         DEC_CHAN_1, SV_1A
                JB         INC_CHAN_2, SV_1A
                JB         DEC_CHAN_2, SV_1A
                JB         INC_CHAN_3, SV_1A
                JB         DEC_CHAN_3, SV_1A
                JB         INC_CHAN_4, SV_1A
                JB         DEC_CHAN_4, SV_1A

                AJMP       SV_1B

       SV_1A:   ACALL      SERV_REM_VOL            ;Service remote volume.
                AJMP       SVX
                
       SV_1B:   ACALL      SERV_LOC_VOL            ;Service local volume.

  SVX:          ;Restore the context that existed prior to the interrupt.
                POP        PSW
                POP        ACC
                RET

;******************************************************************************
;                           Subroutine: SERV_REM_VOL
;******************************************************************************
;Service the LED Flasher Timer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, R4
;
;This routine checks the increment and decrement flags of each of the
;4 channels. If it finds that a flag is set, it activates/de-activates
;the appropriate control lines.
;
;The REMote VOLume CouNTeRs represent the number of increment
;pulses required to reach the volume level specified by the
;remote command. 4 variables are required for independent control.
;
;Because only increment OR decrement will be required at any time
;- not both simultaneously -, CHx_COUNT may be used for both
;increment and decrement.
;
;To ensure that both increment and decrement are not required
;at the same time, the PARSE routine will not process a new
;remote volume control command for a particular channel if either
;of that channel's INC_CHAN_x or DEC_CHAN_x flags are set.
;
SERV_REM_VOL:
                ;********** Channel 1 **************************************
                ;Check channel 1 (1 of 4) decrement flag.
    SRV_1A:     JNB        DEC_CHAN_1, SRV_1B        ;Jmp if dec flag not set.
                SETB       DEC_VOL1_n
                MOV        R4, #30                   ;Delay
      SRV_1A1:  DEC        R4
                CJNE       R4, #00, SRV_1A1

                DEC        CH1_COUNT
                MOV        A, CH1_COUNT
                JNZ        SRV_1A3                   ;Jmp if more decs req'd.

      SRV_1A2:  CLR        DEC_CHAN_1
                SETB       DEC_VOL1_n
                MOV        A, REM_VOL_CNTR1     
                JZ         SRV_2A                    ;Skip inc if vol to be 0.
                SETB       INC_CHAN_1
                CLR        INC_VOL1_n
                MOV        CH1_COUNT, REM_VOL_CNTR1
                AJMP       SRV_2A

      SRV_1A3:  CLR        DEC_VOL1_n
                AJMP       SRV_2A


                ;Check channel 1 (1 of 4) increment flag.
    SRV_1B:     JNB        INC_CHAN_1, SRV_2A        ;Jmp if dec flag not set.
                SETB       INC_VOL1_n
                MOV        R4, #30                   ;Delay
      SRV_1B1:  DEC        R4
                CJNE       R4, #00, SRV_1B1
                DEC        CH1_COUNT
                MOV        A, CH1_COUNT
                JNZ        SRV_1B3

      SRV_1B2:  CLR        INC_CHAN_1
                AJMP       SRV_2A

      SRV_1B3:  CLR        INC_VOL1_n
                AJMP       SRV_2A


                ;********** Channel 2 **************************************
                ;Check channel 2 (2 of 4) decrement flag.
    SRV_2A:     JNB        DEC_CHAN_2, SRV_2B        ;Jmp if dec flag not set.
                SETB       DEC_VOL2_n
                MOV        R4, #30                   ;Delay
      SRV_2A1:  DEC        R4
                CJNE       R4, #00, SRV_2A1

                DEC        CH2_COUNT
                MOV        A, CH2_COUNT
                JNZ        SRV_2A3                   ;Jmp if more decs req'd.

      SRV_2A2:  CLR        DEC_CHAN_2
                SETB       DEC_VOL2_n
                MOV        A, REM_VOL_CNTR2     
                JZ         SRV_2A                    ;Skip inc if vol to be 0.
                SETB       INC_CHAN_2
                CLR        INC_VOL2_n
                MOV        CH2_COUNT, REM_VOL_CNTR2
                AJMP       SRV_3A

      SRV_2A3:  CLR        DEC_VOL2_n
                AJMP       SRV_3A


                ;Check channel 2 (2 of 4) increment flag.
    SRV_2B:     JNB        INC_CHAN_2, SRV_3A        ;Jmp if dec flag not set.
                SETB       INC_VOL2_n
                MOV        R4, #30                   ;Delay
      SRV_2B1:  DEC        R4
                CJNE       R4, #00, SRV_2B1
                DEC        CH2_COUNT
                MOV        A, CH2_COUNT
                JNZ        SRV_2B3

      SRV_2B2:  CLR        INC_CHAN_2
                AJMP       SRV_3A

      SRV_2B3:  CLR        INC_VOL2_n
                AJMP       SRV_3A


                ;********** Channel 3 **************************************
                ;Check channel 3 (3 of 4) decrement flag.
    SRV_3A:     JNB        DEC_CHAN_3, SRV_3B        ;Jmp if dec flag not set.
                SETB       DEC_VOL3_n
                MOV        R4, #30                   ;Delay
      SRV_3A1:  DEC        R4
                CJNE       R4, #00, SRV_3A1

                DEC        CH3_COUNT
                MOV        A, CH3_COUNT
                JNZ        SRV_3A3                   ;Jmp if more decs req'd.

      SRV_3A2:  CLR        DEC_CHAN_3
                SETB       DEC_VOL3_n
                MOV        A, REM_VOL_CNTR3     
                JZ         SRV_3A                    ;Skip inc if vol to be 0.
                SETB       INC_CHAN_3
                CLR        INC_VOL3_n
                MOV        CH3_COUNT, REM_VOL_CNTR3
                AJMP       SRV_4A

      SRV_3A3:  CLR        DEC_VOL3_n
                AJMP       SRV_4A


                ;Check channel 3 (3 of 4) increment flag.
    SRV_3B:     JNB        INC_CHAN_3, SRV_4A        ;Jmp if dec flag not set.
                SETB       INC_VOL3_n
                MOV        R4, #30                   ;Delay
      SRV_3B1:  DEC        R4
                CJNE       R4, #00, SRV_3B1
                DEC        CH3_COUNT
                MOV        A, CH3_COUNT
                JNZ        SRV_3B3

      SRV_3B2:  CLR        INC_CHAN_3
                AJMP       SRV_4A

      SRV_3B3:  CLR        INC_VOL3_n
                AJMP       SRV_4A


                ;********** Channel 4 **************************************
                ;Check channel 4 (4 of 4) decrement flag.
    SRV_4A:     JNB        DEC_CHAN_4, SRV_4B        ;Jmp if dec flag not set.
                SETB       DEC_VOL4_n
                MOV        R4, #30                   ;Delay
      SRV_4A1:  DEC        R4
                CJNE       R4, #00, SRV_4A1

                DEC        CH4_COUNT
                MOV        A, CH4_COUNT
                JNZ        SRV_4A3                   ;Jmp if more decs req'd.

      SRV_4A2:  CLR        DEC_CHAN_4
                SETB       DEC_VOL4_n
                MOV        A, REM_VOL_CNTR4     
                JZ         SRV_4A                    ;Skip inc if vol to be 0.
                SETB       INC_CHAN_4
                CLR        INC_VOL4_n
                MOV        CH4_COUNT, REM_VOL_CNTR4
                AJMP       SRV_X

      SRV_4A3:  CLR        DEC_VOL4_n
                AJMP       SRV_X


                ;Check channel 4 (4 of 4) increment flag.
    SRV_4B:     JNB        INC_CHAN_4, SRV_X         ;Jmp if dec flag not set.
                SETB       INC_VOL4_n
                MOV        R4, #30                   ;Delay
      SRV_4B1:  DEC        R4
                CJNE       R4, #00, SRV_4B1
                DEC        CH4_COUNT
                MOV        A, CH4_COUNT
                JNZ        SRV_4B3

      SRV_4B2:  CLR        INC_CHAN_4
                AJMP       SRV_X

      SRV_4B3:  CLR        INC_VOL4_n

   SRV_X:       RET

;******************************************************************************
;                           Subroutine: SERV_LOC_VOL
;******************************************************************************
;Service the LED Flasher Timer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
;The control lines of the digital pots are taken high (inactive) to finish the 
;cycle of any lines taken low (active) during the last service.
;Then 8 inputs are checked (4 channels, each with an "UP" and "DOWN" button).
;If any individual input is low (active), it's respective control line to a
;digital pot is taken low (active). It will be taken high again at the
;beginning of the next service.
;
SERV_LOC_VOL:

;Take high(inactive) any increment/decrement output pins that are low(active).
                SETB       INC_VOL1_n
                SETB       DEC_VOL1_n
                SETB       INC_VOL2_n
                SETB       DEC_VOL2_n
                SETB       INC_VOL3_n
                SETB       DEC_VOL3_n
                SETB       INC_VOL4_n
                SETB       DEC_VOL4_n

;Poll local volume inputs. Activate output (active-low) as required.
  SLV_BTN1_INC: JB         INC_BTN1_n, SLV_BTN1_DEC  ;
                CLR        INC_VOL1_n                ;

  SLV_BTN1_DEC: JB         DEC_BTN1_n, SLV_BTN2_INC  ;
                CLR        DEC_VOL1_n                ;

  SLV_BTN2_INC: JB         INC_BTN2_n, SLV_BTN2_DEC  ;
                CLR        INC_VOL2_n                ;

  SLV_BTN2_DEC: JB         DEC_BTN2_n, SLV_BTN3_INC  ;
                CLR        DEC_VOL2_n                ;

  SLV_BTN3_INC: JB         INC_BTN3_n, SLV_BTN3_DEC  ;
                CLR        INC_VOL3_n                ;

  SLV_BTN3_DEC: JB         DEC_BTN3_n, SLV_BTN4_INC  ;
                CLR        DEC_VOL3_n                ;

  SLV_BTN4_INC: JB         INC_BTN4_n, SLV_BTN4_DEC  ;
                CLR        INC_VOL4_n                ;

  SLV_BTN4_DEC: JB         DEC_BTN4_n, SLVX          ;
                CLR        DEC_VOL4_n                ;

  SLVX:         RET

;******************************************************************************
;                           Subroutine: SERV_TIMER0
;******************************************************************************
;Service the LED Flasher Timer.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, C
;
SERV_TIMER0:    ;Service the Timer 0 overflow.

                ;Save context for clean return from interrupt.
                PUSH       ACC
                PUSH       PSW

                MOV        TH0, #TMR0_RLD          ;Reload 8 MSBs of timer.
                DEC        SW_FAST_TIMER           ;Decrement software timer.
                DEC        SW_SLOW_TIMER           ;Decrement software timer.
                DEC        DELAY_1_TIMER           ;Decrement software timer.
                DEC        VOLUME_TIMER            ;Decrement software timer.

                ;Check 4 channels increment and decrement flags.
                ; If any 1 or more are set, disable output.
                ; Set flag if output was enabled to begin with.
                JB         INC_CHAN_1, ST0_2A
                JB         DEC_CHAN_1, ST0_2A
                JB         INC_CHAN_2, ST0_2A
                JB         DEC_CHAN_2, ST0_2A
                JB         INC_CHAN_3, ST0_2A
                JB         DEC_CHAN_3, ST0_2A
                JB         INC_CHAN_4, ST0_2A
                JB         DEC_CHAN_4, ST0_2A

                JNB        RST_TUNE_ENBL, ST0_2C
                SETB       TUNE_ENBL
                CLR        RST_TUNE_ENBL
                AJMP       ST0_2C

       ST0_2A:  JNB        TUNE_ENBL, ST0_2B
                SETB       RST_TUNE_ENBL
                
       ST0_2B:  CLR        TUNE_ENBL

       ST0_2C:
                ;Decrement note duration if appropriate.
                JNB        TUNE_ENBL, STO_X

                MOV        A, NOTE_DURATION
                CJNE       A, #00, STO_DEC_DUR

                CLR        PLAY_REST               ;If a rest was being
                                                   ;  played, it's duration
                                                   ;  has expired.
ST0_SKIP_DEC:   AJMP       STO_X

 STO_DEC_DUR:   DEC        NOTE_DURATION            ;Decrement software timer.

       STO_X:   
                ;Restore the context that existed prior to the interrupt.
                POP        PSW
                POP        ACC
    
                RETI

;******************************************************************************
;                           Subroutine: LAMP_TEST
;******************************************************************************
;Turn all output lamps ON.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
;The notes regarding digit location have been modified to reflect
;  AD-0941-001. They are in reverse order as compared to previous
;  applications of the maxim driver(s). "1st digit" referred to below
;  is the most significant digit or left most digit for this application.
;
LAMP_TEST:      ;CLR        STATUS_LED_n            ;Turn status LED on.
                CLR        TUNE_LED_n              ;Turn tune LED on.

                ;Set all outputs ON.
                MOV        TUNE_VAL,     #0FFh

                LCALL      PLAY_TUNE

                ;Set digit attributes to ON.
                MOV        ATR_DIG, #'1'

                MOV        DAT_DIG3, #'I'          ;Store '0' for future use.
                MOV        DAT_DIG2, #'I'          ;Store '0' for future use.
                MOV        DAT_DIG1, #'I'          ;Store '0' for future use.

                ;Turn on all elements of 7-segment display.
                MOV        A, #MAX7219_ADR_DIGIT_0 ;Adr: 1ST digit (left most)
                MOV        R0, #'I'                ;Data: I is 8 with dec point
                ACALL      PUT_CHAR

                MOV        A, #MAX7219_ADR_DIGIT_1 ;Adr: 2ND digit.
                MOV        R0, #'I'                ;Data: I is 8 with dec point
                ACALL      PUT_CHAR

                MOV        A, #MAX7219_ADR_DIGIT_2 ;Adr: 3RD digit. (right most)
                MOV        R0, #'I'                ;Data: I is 8 with dec point
                ACALL      PUT_CHAR

                RET

;******************************************************************************
;                            Subroutine: LAMP_CLR
;******************************************************************************
;Turn all outputs off.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
;The notes regarding digit location have been modified to reflect
;  AD-0941-001. They are in reverse order as compared to previous
;  applications of the maxim driver(s). "1st digit" referred to below
;  is the most significant digit or left most digit for this application.
;
LAMP_CLR:       ;SETB       STATUS_LED_n            ;Turn status LED off.

                ;Set all outputs OFF.
                MOV        TUNE_VAL,     #000h

                LCALL      PLAY_TUNE

                ;Set digit attributes to OFF.
                MOV        ATR_DIG, #'0'

                MOV        DAT_DIG3, #'0'          ;Store '0' for future use.
                MOV        DAT_DIG2, #'0'          ;Store '0' for future use.
                MOV        DAT_DIG1, #'0'          ;Store '0' for future use.

                ;Turn all elements of 7-segment display to zero.
                MOV        A, #MAX7219_ADR_DIGIT_0 ;Adr: 1ST digit (left most)
                MOV        R0, #'0'                ;Data: display a '0'.
                ACALL      PUT_CHAR

                MOV        A, #MAX7219_ADR_DIGIT_1 ;Adr: 2ND digit.
                MOV        R0, #'0'                ;Data: display a '0'.
                ACALL      PUT_CHAR

                MOV        A, #MAX7219_ADR_DIGIT_2 ;Adr: 3RD digit (right most)
                MOV        R0, #'0'                ;Data: display a '0'.
                ACALL      PUT_CHAR

                RET

;******************************************************************************
;                               Subroutine: MESS
;******************************************************************************
;Display a four byte string on the three 7-segment digits.
;Starting address of string to display is passed in DPTR.
;
;The notes regarding digit location have been modified to reflect
;  AD-0941-001. They are in reverse order as compared to previous
;  applications of the maxim driver(s). "1st digit" referred to below
;  is the most significant digit or left most digit for this application.
;
MESS:           MOV        A,#3
                MOVC       A,@A+DPTR               ;Get right most character.
                MOV        R0, A                   ;Data: Move data byte to R0.
                MOV        A, #MAX7219_ADR_DIGIT_2 ;Adr: 3RD digit (right most)
                MOV        DAT_DIG3, R0            ;Store for future use.
                ACALL      PUT_CHAR

                MOV        A,#2
                MOVC       A,@A+DPTR               ;Get 2nd character.
                MOV        R0, A                   ;Data: Move data byte to R0.
                MOV        A, #MAX7219_ADR_DIGIT_1 ;Adr: 2ND digit
                MOV        DAT_DIG2, R0            ;Store for future use.
                ACALL      PUT_CHAR

                MOV        A,#1
                MOVC       A,@A+DPTR               ;Get left most character.
                MOV        R0, A                   ;Data: Move data byte to R0.
                MOV        A, #MAX7219_ADR_DIGIT_0 ;Adr: 1ST digit (left most)
                MOV        DAT_DIG1, R0            ;Store for future use.
                ACALL      PUT_CHAR

                RET

;******************************************************************************
;                          Subroutine: DELAY_1_SEC
;******************************************************************************
;Generate a 1 second delay.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, C
;
;Variable DELAY_1_TIMER is decremented each time the TIMER 0 interrupt is
;serviced.  The interrupt occurs at a 100 Hz rate.  Therefore 100 ticks will
;yield a 1 second delay.
;
DELAY_1_SEC:    MOV        DELAY_1_TIMER, #100     ;Initialize delay timer.

                MOV        A, #0
                CJNE       A, DELAY_1_TIMER, $     ;Stay here until D_1_TIMER=0

                RET

;******************************************************************************
;                          Subroutine: DELAY_QRT_SEC
;******************************************************************************
;Generate a 1 second delay.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, C
;
;Variable DELAY_1_TIMER is decremented each time the TIMER 0 interrupt is
;serviced.  The interrupt occurs at a 100 Hz rate.  Therefore 25 ticks will
;yield a 1/4 second delay.
;
DELAY_QRT_SEC:  MOV        DELAY_1_TIMER, #25      ;Initialize delay timer.

                MOV        A, #0
                CJNE       A, DELAY_1_TIMER, $     ;Stay here until D_1_TIMER=0

                RET


;******************************************************************************
;                            Subroutine: PUT_CHAR
;******************************************************************************
;Display a digit on one of the 7-segment cells.
;
;  PARAMETERS:
;    RECEIVES:
;      A --- ADDRESS of digit to be updated.
;      R0 -- DATA value of digit (ASCII).
;    RETURNS:
;      nothing
;      destroys A, R0
;
;The font table contains the digits (and a few characters) that can be
;displayed on the 7-segment LED matrix.  The table starts with the space
;character (20h) and contains 1 byte per character.  This one byte is used four
;times per character to light the four LEDs per segment.
;
;ASCII character codes 20h through 5Ah are supported, although most codes will
;produce a blank display.  The ASCII characters that are useful are:
;
;  0-9 -- To produce the digits 0-9
;  A-J -- To produce the digits 0-9 with a right hand decimal point
;  V   -- To produce a lower case u character.
;  -   -- To produce the dash character.
;
;Characters outside the range 20h to 5Ah are remapped to 20h.
;
;The character to be displayed is used as an index into a look up table.  The
;table starts with the space character (20h), therefore 1Fh (20h-1) is
;subtracted from each character before calling the table in order to have
;indexes that start at 1.
;
PUT_CHAR:       PUSH       ACC                     ;Save address.
                MOV        A, R0                   ;Move data to ACC.

                ;Remap character to 20h if CHAR < 20h, or  CHAR > 5Ah.
                CJNE       A, #5Ah, PCHR1          ;Use CJNE as  < comparator.
  PCHR1:        JNC        BAD_CHAR                ;Jump if char >= 5Ah

                CJNE       A, #20h, PCHR2          ;Use CJNE as  < comparator.
  PCHR2:        JNC        GOOD_CHAR               ;Jump if char >= 20h

  BAD_CHAR:     MOV        A, #20h                 ;Remap bad characters.

  GOOD_CHAR:    ;Get data byte from font table for the digit to be displayed.
                CLR        C
                SUBB       A, #1Fh                 ;Subtrct offset to 1st char
                LCALL      CHAR_FONT

                MOV        R0,A                    ;Put data (BINARY) into R0.
                POP        ACC                     ;Restore address to ACC.
                ACALL      MAX7219_WRITE           ;Write data to display drvr.
 
                ;Exit shutdown mode (enable display).
                ;  This is in case FLASH service routine has shutdown ON.
                MOV        A, #MAX7219_ADR_SHUTDOWN
                MOV        R0,#MAX7219_DAT_SHUTDOWN_NORMAL ;Turn digits ON.
                ACALL      MAX7219_WRITE


                RET

;******************************************************************************
;                           Subroutine: PARSE_RESET
;******************************************************************************
;Reset command parser state variable and flags.
;However, DO NOT change the state of the STX_WAS_SEEN flag in this routine.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
PARSE_RESET:    MOV        PAR_STATE,#00           ;Reset cmd parser to begin.

                ;Clear OP CODE flags.
                CLR        OP_ADR
                CLR        OP_ESC
                CLR        OP_TUN
                CLR        OP_URT
                CLR        OP_VOL
                CLR        OP_MAX

                RET

;******************************************************************************
;                              Subroutine: PARSE
;******************************************************************************
;Parse the received command packets and execute after EOT is received.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, C, R0
;
;This routine is called each time a byte is received.  The byte to process is
;passed in ACC.
;
;Following the reset of the microcontroller, the local board address is set to
;FF, and the downstream transmitter and receiver are enabled.
;
;Until the first STX character is received, the board operates in a SIMPLE
;mode, useful for debugging.  In this mode the incoming byte is used to
;determine the tune output (06/26/01, direct-drive disabled in ver 0.02).
;
;During the SIMPLE mode, the XMIT UART is not used.  However, since both the
;downstream transmitter and receiver are enabled, a loop back plug can be
;inserted into the downstream data connector and the headend will then hear
;itself as an echo.
;
;The first time that STX is detected, the downstream connection is disabled.
;This is done in anticipation of the PCB being commanded to take a new local
;address, after which it will re-enable the downstream link.
;
;The supported commands and their formats are:
;
;  Parser STATE-->    DEC DEC DEC DEC DEC DEC DEC DEC DEC DEC DEC DEC
;  Parser STATE-->     0   1   2   3   4   5   6   7   8   9   10  11
;
;  Set Address:       STX  a1  a0         'A'  n1  n0  m1  m0     EOT
;  Play tune, normal: STX  a1  a0         'T' '0'  d1  d0         EOT
;  Play tune, fast:   STX  a1  a0         'T' '2'  d1  d0         EOT
;  Set volume:        STX  a1  a0         'V'  c   v1  v0         EOT
;  Set baud 9600:     STX  a1  a0         'U' '0'                 EOT
;  Set baud 57,600:   STX  a1  a0         'U' '1'                 EOT
;  Set baud 19,200:   STX  a1  a0         'U' '2'                 EOT
;  Set baud 28,800:   STX  a1  a0         'U' '3'                 EOT
;  Write MAX7219:     STX  a1  a0         'M'  d3  d2  d1  d0     EOT
;  Disable RECV_DWNS: STX  a1  a0         'Z' '0'                 EOT
;  Enable RECV_DWNS:  STX  a1  a0         'Z' '1'                 EOT
;
;  Goto P.O.R. state: STX  a1  a0         ESC '0'                 EOT
;  Lamp Test:         STX  a1  a0         ESC '1'                 EOT
;  Clear:             STX  a1  a0         ESC '3'                 EOT
;  Return PCB ID:     STX  a1  a0         ESC '4'                 EOT
;  Reset timer:       STX  a1  a0         ESC '5'                 EOT
;
;In the above commands, the special GLOBAL address '0' '0' (inserted in the
;a1 a0 field) can be used to force an address match for ALL boards.  This
;allows a single command packet to be used to drive all PCBs.
;
;The DEVICE SET address command form can be used to cause all boards with the
;same DEVICE SET address (s1 s0 field) to match and respond to a command.  The
;DEVICE SET address is initially set using the Set Address command in the table
;above, with the "new" DEVICE SET address specified in field m1 m0.  The format
;of the commands when using the DEVICE SET option are shown below.  Note that
;the local address field (a1 a0) must be set to the special address 'F' 'E' to
;invoke the DEVICE SET addressing option:
;
;  Parser STATE-->    DEC DEC DEC DEC DEC DEC DEC DEC DEC DEC DEC DEC
;  Parser STATE-->     0   1   2   3   4   5   6   7   8   9   10  11
;
;  Set Address:       STX 'F' 'E'  s1  s0 'A'  n1  n0  m1  m0     EOT
;  Play tune, normal: STX 'F' 'E'  s1  s0 'T' '0'  d1  d0         EOT
;  Play tune, fast:   STX 'F' 'E'  s1  s0 'T' '2'  d1  d0         EOT
;  Set volume:        STX 'F' 'E'  s1  s0 'V'  c   v1  v0         EOT
;  Set baud 9600:     STX 'F' 'E'  s1  s0 'U' '0'                 EOT
;  Set baud 57,600:   STX 'F' 'E'  s1  s0 'U' '1'                 EOT
;  Set baud 19,200:   STX 'F' 'E'  s1  s0 'U' '2'                 EOT
;  Set baud 28,800:   STX 'F' 'E'  s1  s0 'U' '3'                 EOT
;  Write MAX7219:     STX 'F' 'E'  s1  s0 'M'  d3  d2  d1  d0     EOT
;  Disable RECV_DWNS: STX 'F' 'E'  s1  s0 'Z' '0'                 EOT
;  Enable RECV_DWNS:  STX 'F' 'E'  s1  s0 'Z' '1'                 EOT
;
;  Goto P.O.R. state: STX 'F' 'E'  s1  s0 ESC '0'                 EOT
;  Lamp Test:         STX 'F' 'E'  s1  s0 ESC '1'                 EOT
;  Clear:             STX 'F' 'E'  s1  s0 ESC '3'                 EOT
;  Return PCB ID:     STX 'F' 'E'  s1  s0 ESC '4'                 EOT
;  Reset timer:       STX 'F' 'E'  s1  s0 ESC '5'                 EOT
;
;The symbols in the above tables of commands have the following meanings:
;
;  STX,ESC,EOT --- ASCII control codes 02h, 1Bh and 04h, respectively.
;
;  'A' ----------- An ASCII literal. i.e. 'A' is the character 'A' (code 41h).
;
;  a1 a0 --------- The current local address. 2 bytes, ASCII HEX characters.
;  n1 n0 --------- The new local address. 2 bytes, ASCII HEX characters.
;                  The following local addresses are special:
;                    '0' '0' --- Global local address that always matches.
;                    'F' 'E' --- DEVICE SET local address.  Two more address
;                                bytes (the DEVICE SET address) will follow.
;                    'F' 'F' --- The power on reset (P.O.R.) local address.
;
;  s1 s0 --------- The current DEVICE SET number. 2 bytes, ASCII HEX char.
;  m1 m0 --------- The new DEVICE SET number. 2 bytes, ASCII HEX characters.
;
;  d1 d0 --------- 8 bits of data for tune number. 2 bytes, ASCII HEX char.
;
;  d3 d2 d1 d0 --- 16-bit data field.  4 bytes, ASCII HEX characters.
;
;    c   --------- 4 bits of data for channel number. 1 byte, ASCII HEX char.
;
;  v1 v0 --------- 8 bits of data for volume. 2 bytes, ASCII HEX char.
;
PARSE:          MOV        A, REC_BYTE             ;Parse the command.
                CJNE       A, #STX, NOT_STX        ;Check for STX (ALWAYS).

  GOT_STX:      ;Process STX byte.
                ;  Command parsing is reset to the beginning any time a new STX
                ;  is received.
;               MOV        SBUF,#' '               ;print a marker
                JB         STX_WAS_SEEN,NOT_1ST_STX ;Jump if not 1ST STX.

                SETB       STX_WAS_SEEN            ;1ST STX has now been seen.

                SETB       RECV_DWNSTREAM_ENABLE_n ;Disable input from downstr.
                CLR        XMIT_DWNSTREAM_ENABLE   ;Disable output to downstrm.

  NOT_1ST_STX:  ACALL      PARSE_RESET             ;Reset the PARSER.
                MOV        PAR_STATE,#01           ;Advance cmd parser state.
                RET

  NOT_STX:      JB         STX_WAS_SEEN,CONT_PARSE ;Has STX never been seen?

  SEND_DIRECT:  ;SIMPLE MODE.  Used for diagnostics.
                ;ACALL      DIRECT_DRV              ;Use direct drive mode.
                RET

  CONT_PARSE:   ;If we arrive here it means that an STX was previously seen
                ;and we are somewhere in the middle of a command packet.
                ;Therefore jump to the appropriate piece of code based on the
                ;present state of the PARSER machine state variable PAR_STATE.
                ;
                MOV        DPTR, #PAR_JMP_TBL      ;Load pointer to JMP TABLE.

                MOV        A, PAR_STATE            ;PARSER STATE x2 is index.
                CLR        C                       ;x2 because AJMP length is 2
                RLC        A                       ;

                JMP        @A+DPTR                 ;JMP to JMP table.

  PAR_JMP_TBL:  ;WARNING: DO NOT CHANGE THE AJMPS BELOW TO JMPs OR LJMPs.  The
                ;         entry index into the jump table below assumes that
                ;         the AJMP is 2 bytes long.
                ;
                AJMP       PAR_JMP_TBL_X           ;Bail out.
                AJMP       DECODE1                 ;Decode 1ST byte after STX.
                AJMP       DECODE2                 ;  "    2ND  "     "    " .
                AJMP       DECODE3                 ;  "    3RD  "     "    " .
                AJMP       DECODE4                 ;  "    4TH  "     "    " .
                AJMP       DECODE5                 ;  "    5TH  "     "    " .
                AJMP       DECODE6                 ;  "    6TH  "     "    " .
                AJMP       DECODE7                 ;  "    7TH  "     "    " .
                AJMP       DECODE8                 ;  "    8TH  "     "    " .
                AJMP       DECODE9                 ;  "    9TH  "     "    " .
                AJMP       DECODE10                ;  "   10TH  "     "    " .
                AJMP       DECODE11                ;  "   11TH  "     "    " .

 PAR_JMP_TBL_X: LJMP       DEC11_X                 ;Bail out.

  DECODE1:      ;Process 1ST byte following STX.
                ;  This is the MS nibble of the local address field.  Simply
                ;  store the first address nibble at this time.  Wait to do
                ;  address compares until the 2ND address nibble is received.
;               MOV        SBUF,#'1'               ;print a marker
                MOV        A, REC_BYTE             ;Get received byte,
                LCALL      HEX2BIN                 ;and convert it to binary.
                JC         DEC1_X                  ;Bail if invalid character.
                MOV        REC_BYTE_1, A           ;Save 1ST address nibble.

                MOV        PAR_STATE,#02           ;Advance cmd parser state.
                RET

    DEC1_X:     LJMP       DEC11_X                 ;Bail out.

  DECODE2:      ;Process 2ND byte following STX.
                ;  This is the LS nibble of the local address field.  Use this,
                ;  along with the previously stored MS address nibble to do
                ;  compares and determine if the PCB is being addressed.  If
                ;  address compare fails, then the incoming packet IS NOT
                ;  destined for this PCB, and the command parser is reset to
                ;  begin looking for a new STX.
                ;
                ;  Three address cases are tested:
                ;    00 -- Always matches (global mode).
                ;    FE -- Always matches (flag for DEVICE SET address mode).
                ;    xx -- All other addresses must directly match the stored
                ;          local address nibbles.
;               MOV        SBUF,#'2'               ;print a marker
                MOV        A, REC_BYTE             ;Get received byte,
                LCALL      HEX2BIN                 ;and convert it to binary.
                JC         DEC2_X                  ;Bail if invalid character.
                MOV        REC_BYTE_2, A           ;Save 2ND address nibble.

                ;Check to see if address is 00h.
                ORL        A, REC_BYTE_1           ;OR the two address nibbles.
                JZ         ADR_MATCH               ;if both 0, OR will be 0 too

                ;Check to see if address is 0FEh.
                MOV        A, REC_BYTE_1           ;Retrieve MS address nibble.
                SWAP       A                       ;Move to hi nibble position.
                ORL        A, REC_BYTE_2           ;Overlay LS address nibble.
                XRL        A, #0FEh                ;Compare to FEh.
                JZ         ADR_IS_FE               ;if FEh, then XOR will be 0.

                ;Check to see if address matches current local address.
                MOV        A, REC_BYTE_1           ;Retrieve MS address nibble.
                CJNE       A, LOC_AD1, DEC2_X      ;Bail if no address match.

                MOV        A, REC_BYTE_2           ;Retrieve LS address nibble.
                CJNE       A, LOC_AD0, DEC2_X      ;Bail if no address match.

    ADR_MATCH:  MOV        PAR_STATE,#05           ;Advance cmd parser state.
                RET

    ADR_IS_FE:  MOV        PAR_STATE,#03           ;Advance cmd parser state.
                RET

    DEC2_X:     LJMP       DEC11_X                 ;Bail out.

  DECODE3:      ;Process 3RD byte following STX.
                ;  This is the MS nibble of the DEVICE SET address field.
                ;  Simply store this nibble at this time.  Wait to do DEVICE
                ;  SET address compare until the 2ND DEVICE SET address nibble
                ;  is received.
;               MOV        SBUF,#'3'               ;print a marker
                MOV        A, REC_BYTE             ;Get received byte,
                LCALL      HEX2BIN                 ;and convert it to binary.
                JC         DEC3_X                  ;Bail if invalid character.
                MOV        REC_BYTE_3, A           ;Save 1ST DEVICE SET ad nibl

                MOV        PAR_STATE,#04           ;Advance cmd parser state.
                RET

    DEC3_X:     LJMP       DEC11_X                 ;Bail out.

  DECODE4:      ;Process 4TH byte following STX.
                ;  This is the LS nibble of the DEVICE SET address field.  Use
                ;  this, along with the previously stored MS nibble of the
                ;  DEVICE SET address field to compare and determine if the PCB
                ;  is being addressed in the DEVICE SET mode.  If address
                ;  compare fails, then the incoming packet IS NOT destined for
                ;  this PCB, and the command parser is reset to begin looking
                ;  for a new STX.
;               MOV        SBUF,#'4'               ;print a marker
                MOV        A, REC_BYTE             ;Get received byte,
                LCALL      HEX2BIN                 ;and convert it to binary.
                JC         DEC4_X                  ;Bail if invalid character.
                MOV        REC_BYTE_4, A           ;Save 2ND DEVICE SET ad nibl

                ;Check to see if address matches current local DEVICE SET addr.
                MOV        A, REC_BYTE_3           ;Retrieve MS address nibble.
                CJNE       A, LOC_DEV_AD1,DEC4_X   ;Bail if no address match.

                MOV        A, REC_BYTE_4           ;Retrieve LS address nibble.
                CJNE       A, LOC_DEV_AD0,DEC4_X   ;Bail if no address match.

                ;Else address matches.
                MOV        PAR_STATE,#05           ;Advance cmd parser state.
                RET

    DEC4_X:     LJMP       DEC11_X                 ;Bail out.

  DECODE5:      ;Process 5TH byte following STX.
                ;  This is the opcode byte.
                ;  Address is guaranteed to be valid if we have gotten this far
;               MOV        SBUF,#'5'               ;print a marker
                MOV        A, REC_BYTE             ;Reload ACC with recvd byte.
    DEC5_A:     CJNE       A, #CMD_ADR, DEC5_E     ;If not A, see if it's ESC.
                SETB       OP_ADR                  ;Set addr opcode flag.
                MOV        PAR_STATE,#06           ;Advance cmd parser state.
                RET

    DEC5_E:     CJNE       A, #ESC, DEC5_U         ;Not A or ESC, is it U?
                SETB       OP_ESC                  ;Set ESC opcode flag.
                MOV        PAR_STATE,#06           ;Advance cmd parser state.
                RET

    DEC5_U:     CJNE       A, #CMD_URT, DEC5_V     ;Not A,ESC,U is it V?
                SETB       OP_URT                  ;Set UART opcode flag.
                MOV        PAR_STATE,#06           ;Advance cmd parser state.
                RET

    DEC5_V:     CJNE       A, #CMD_VOL, DEC5_T     ;Not A,ESC,U,V is it T?
                SETB       OP_VOL                  ;Set volume opcode flag.
                MOV        PAR_STATE,#06           ;Advance cmd parser state.
                RET

    DEC5_T:     CJNE       A, #CMD_TUN, DEC5_M     ;Not A,ESC,U,V,T is it M?
                SETB       OP_TUN                  ;Set tune opcode flag.
                MOV        PAR_STATE,#06           ;Advance cmd parser state.
                RET

    DEC5_M:     CJNE       A, #CMD_MAX, DEC5_Z     ;Not A,ESC,U,V,T,M is it Z?
                SETB       OP_MAX                  ;Set Maxim opcode flag.
                MOV        PAR_STATE,#06           ;Advance cmd parser state.
                RET

    DEC5_Z:     CJNE       A, #CMD_ZTM, BAD_OPCODE ;If none of above->bad opcod
                SETB       OP_ZTM                  ;Set opcode type flag.
                MOV        PAR_STATE,#06           ;Advance cmd parser state.
                RET

    BAD_OPCODE: LJMP       DEC11_X                 ;Bail out.  Opcode is bad.

  DECODE6:      ;Process 6TH byte following STX.
                ;  This is the 1ST parameter byte.  Simply store and move on.
                ;  Opcode is guaranteed to be valid if we have gotten this far.
;               MOV        SBUF,#'6'               ;print a marker
                MOV        REC_BYTE_6, REC_BYTE    ;Get received byte.

                JB         OP_ADR,DEC6_A           ;Handle byte 6 as ADDR case.
                JB         OP_ESC,DEC6_E           ;Handle byte 6 as ESC case.
                JB         OP_URT,DEC6_U
                JB         OP_VOL,DEC6_V           ;Handle byte 6 as VOL case.
                JB         OP_TUN,DEC6_T           ;Handle byte 6 as TUNe case.
                JB         OP_MAX,DEC6_M           ;Handle byte 6 as MAXim case.
                JB         OP_ZTM,DEC6_Z           ;Handle byte 6 as 
                                                   ;  terminator packet.

    DEC6_A:
    DEC6_V:
    DEC6_M:
    DEC6_T:     MOV        PAR_STATE,#07           ;Advance cmd parser state.
                RET

    DEC6_Z:
    DEC6_U:
    DEC6_E:     MOV        PAR_STATE,#11           ;Advance cmd parser state.
                RET

  DECODE7:      ;Process 7TH byte following STX.
                ;  This is the 2ND parameter byte.  Simply store and move on.
                ;  Opcode is guaranteed to be valid if we have gotten this far.
;               MOV        SBUF,#'7'               ;print a marker
                MOV        REC_BYTE_7, REC_BYTE    ;Get received byte.

                JB         OP_ADR,DEC7_A           ;Handle byte 7 as ADDR case.
                JB         OP_VOL,DEC7_V           ;Handle byte 7 as VOL case.
                JB         OP_TUN,DEC7_T           ;Handle byte 7 as TUNe case.
                JB         OP_MAX,DEC7_M           ;Handle byte 7 as MAXim case.

    DEC7_A:
    DEC7_V:
    DEC7_M:
    DEC7_T:     MOV        PAR_STATE,#08           ;Advance cmd parser state.
                RET

  DECODE8:      ;Process 8TH byte following STX.
                ;  This is the 3RD parameter byte.  Simply store and move on.
                ;  Opcode is guaranteed to be valid if we have gotten this far.
;               MOV        SBUF,#'8'               ;print a marker
                MOV        REC_BYTE_8, REC_BYTE    ;Get received byte.

                JB         OP_ADR,DEC8_A           ;Handle byte 8 as ADDR case.
                JB         OP_VOL,DEC8_V           ;Handle byte 8 as VOL case.
                JB         OP_TUN,DEC8_T           ;Handle byte 8 as TUNe case.
                JB         OP_MAX,DEC8_M           ;Handle byte 8 as MAXim case.

    DEC8_M:
    DEC8_A:     MOV        PAR_STATE,#09           ;Advance cmd parser state.
                RET

    DEC8_V:
    DEC8_T:     MOV        PAR_STATE,#11           ;Advance cmd parser state.
                RET

  DECODE9:      ;Process 9TH byte following STX.
                ;  This is the 4TH parameter byte.  Simply store and move on.
                ;  Opcode is guaranteed to be valid if we have gotten this far.
;               MOV        SBUF,#'9'               ;print a marker
                MOV        REC_BYTE_9, REC_BYTE    ;Get received byte.

                JB         OP_ADR,DEC9_A           ;Handle byte 9 as ADDR case.
                JB         OP_MAX,DEC9_M           ;Handle byte 9 as MAXim case.

;TEMP: Try enabling downstream transmitter at this point BEFORE the EOT in
;      order to see if LOCAL ADDRESS setting delay requirement goes away.
    DEC9_A:     SETB       XMIT_DWNSTREAM_ENABLE   ;Enable output to downstrm.
                CLR        RECV_DWNSTREAM_ENABLE_n ;Enable input from downstr.
    DEC9_M:     MOV        PAR_STATE,#11           ;Advance cmd parser state.
                RET

  DECODE10:     ;Process 10TH byte following STX.
                ;  This is the 5TH parameter byte.  Simply store and move on.
                ;  Opcode is guaranteed to be valid if we have gotten this far.
;               MOV        SBUF,#'a'               ;print a marker
                MOV        REC_BYTE_10, REC_BYTE   ;Get received byte.

                RET

  DECODE11:     ;Process 11TH byte following STX.
                ;  This is the EOT byte (end of packet).
                ;  Packet is complete and good.  Go ahead and execute the
                ;  command.
;               MOV        SBUF,#'b'               ;print a marker
                MOV        A, REC_BYTE             ;Reload ACC with recvd byte.
                CJNE       A,#EOT, PROC_AX         ;jmp if no EOT at end

                JB         OP_ADR,PROC_A           ;Process new address.
                JB         OP_ESC,PROC_E           ;Process ESC sequence.
                JB         OP_URT,PRCU_JMP         ;Process UaRT packet.
                JB         OP_VOL,PRCV_JMP         ;Process VOL output.
                JB         OP_TUN,PRCT_JMP         ;Process TUNe output.
                JB         OP_MAX,PRCM_JMP         ;Process MAX7219 direct wr.
                JB         OP_ZTM,PRCZ_JMP         ;Process Terminator packet.

    PRCU_JMP:   AJMP       PROC_U
    PRCV_JMP:   AJMP       PROC_V
    PRCT_JMP:   AJMP       PROC_T
    PRCM_JMP:   AJMP       PROC_M
    PRCZ_JMP:   LJMP       PROC_Z

                ;Execute address update.
    PROC_A:     MOV        A, REC_BYTE_6           ;Convert adr nibl to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_AX                 ;Bail if invalid character.
                MOV        REC_BYTE_6, A           ;Resave as binary.

                MOV        A, REC_BYTE_7           ;Convert adr nibl to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_AX                 ;Bail if invalid character.
                MOV        REC_BYTE_7, A           ;Resave as binary.

                MOV        A, REC_BYTE_8           ;Convert adr nibl to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_AX                 ;Bail if invalid character.
                MOV        REC_BYTE_8, A           ;Resave as binary.

                MOV        A, REC_BYTE_9           ;Convert adr nibl to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_AX                 ;Bail if invalid character.
                MOV        REC_BYTE_9, A           ;Resave as binary.

                ;Now update local and device set addresses.
                MOV        LOC_AD1, REC_BYTE_6     ;Update the local address.
                MOV        LOC_AD0, REC_BYTE_7

                MOV        LOC_DEV_AD1, REC_BYTE_8 ;Update the dev set address.
                MOV        LOC_DEV_AD0, REC_BYTE_9

                ;AJMP       PROC_X

      PROC_AX:  LJMP       DEC11_X

   PROC_E:      MOV        A, REC_BYTE_6           ;Execute ESC code.
                CJNE       A,#'0',PRCE1
                ACALL      ESC0_RESET              ;Return to P.O.R. display.
                AJMP       PROC_EX

      PRCE1:    CJNE       A,#'1',PRCE2
                ACALL      LAMP_TEST               ;Turn all outputs ON.
                SETB       LAMP_TST_FLG            ;Turn on lamp test flag.
                CLR        DBL_TIME                ;clear double-time flag.
                AJMP       PROC_EX

      PRCE2:    CJNE       A,#'3',PRCE3
                ACALL      LAMP_CLR                ;Clear the outputs.
                CLR        LAMP_TST_FLG            ;Turn off lamp test flag.
                CLR        DBL_TIME                ;clear double-time flag.
                AJMP       PROC_EX

      PRCE3:    CJNE       A,#'4',PRCE4
                SETB       TX_ID_STR               ;Set flag to transmit PCB ID
                MOV        ID_INDEX, #1
                AJMP       PROC_EX

      PRCE4:    CJNE       A,#'5',PROC_EX
                ACALL      INIT_FLASHER            ;Resync the flasher.
                AJMP       PROC_EX

      PROC_EX:  LJMP       DEC11_X                 ;Bail out.

                ;Execute set baud rate command.
    PROC_U:     MOV        A, REC_BYTE_6           ;Load parameter.
                CLR        TR1                     ;Stop the timer.
                CLR        REN                     ;Disable serial port rcption
                CLR        ES                      ;Disable UART interrupt.

      PRCU0:    CJNE       A,#'0',PRCU1
                ;Parameter=0: Set baud rate to 9,600
                MOV        TH1, #0FDh              ;Set reload for   9,600 baud
                ANL        PCON,#01111111b         ;non-dbl rate to  9,600 baud
                MOV        TL1, TH1                ;TL1 impacts 1st char only.
                AJMP       PROC_UX

      PRCU1:    CJNE       A,#'1',PRCU2
                ;Parameter=1: Set baud rate to 57,600
                MOV        TH1, #0FFh              ;Set reload for  28,800 baud
                ORL        PCON,#10000000b         ;Double rate to  57,600 baud
                MOV        TL1, TH1                ;TL1 impacts 1st char only.
                AJMP       PROC_UX

      PRCU2:    CJNE       A,#'2',PRCU3
                ;Parameter=2: Set baud rate to 19,200
                MOV        TH1, #0FDh              ;Set reload for   9,600 baud
                ORL        PCON,#10000000b         ;Double rate to  19,200 baud
                MOV        TL1, TH1                ;TL1 impacts 1st char only.
                AJMP       PROC_UX

      PRCU3:    CJNE       A,#'3',PROC_UX
                ;Parameter=3: Set baud rate to 28,800
                MOV        TH1, #0FEh              ;Set reload for  14,400 baud
                ORL        PCON,#10000000b         ;Double rate to  28,800 baud
                MOV        TL1, TH1                ;TL1 impacts 1st char only.
                AJMP       PROC_UX

      PROC_UX:  SETB       TR1                     ;Start the timer.
                CLR        RI                      ;Clear receive intrpt flag.
                CLR        TI                      ;Clear transmit intrpt flag.
                ;SETB       UART_TX_EMT             ;Set transmitter ready flag.
                SETB       REN                     ;Enable serial port recption
                SETB       ES                      ;Enable UART interrupt.
                LJMP       DEC11_X

                ;Process volume control command.
    PROC_V:     MOV        A, REC_BYTE_6           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_VX                 ;Bail if invalid character.
                MOV        REC_BYTE_6, A           ;Resave as binary.

                MOV        A, REC_BYTE_7           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_VX                 ;Bail if invalid character.
                MOV        REC_BYTE_7, A           ;Resave as binary.

                MOV        A, REC_BYTE_8           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_VX                 ;Bail if invalid character.
                MOV        REC_BYTE_8, A           ;Resave as binary.

                ;Nibbles are good. Pack them and update volume.
                MOV        A, REC_BYTE_6           ;Load channel number.
                JZ         PRCV0                   ;Update volume 0.
                DEC        A
                JZ         PRCV1                   ;Update volume 1.
                DEC        A
                JZ         PRCV2                   ;Update volume 2.
                DEC        A
                JZ         PRCV3                   ;Update volume 3.
                AJMP       PROC_VX                 ;Else bail.

      PRCV0:    JB         DEC_CHAN_1, PROC_VX     ;If inc or dec already
                JB         INC_CHAN_1, PROC_VX     ; set, then bail.
                CLR        DEC_VOL1_n              ;Activate decrement.
                LCALL      SET_REM_VOL             ;
                MOV        REM_VOL_CNTR1, A        ;Store result from ACC.
                MOV        CH1_COUNT, #33          ;Init decrement amount.
                SETB       DEC_CHAN_1              ;Set dec channel 1 flag.
                AJMP       PROC_VX

      PRCV1:    JB         DEC_CHAN_2, PROC_VX     ;If inc or dec already
                JB         INC_CHAN_2, PROC_VX     ; set, then bail.
                CLR        DEC_VOL2_n              ;Activate decrement.
                LCALL      SET_REM_VOL             ;
                MOV        REM_VOL_CNTR2, A        ;Store result from ACC.
                MOV        CH2_COUNT, #33          ;Init decrement amount.
                SETB       DEC_CHAN_2              ;Set dec channel 2 flag.
                AJMP       PROC_VX

      PRCV2:    JB         DEC_CHAN_3, PROC_VX     ;If inc or dec already
                JB         INC_CHAN_3, PROC_VX     ; set, then bail.
                CLR        DEC_VOL3_n              ;Activate decrement.
                LCALL      SET_REM_VOL             ;
                MOV        REM_VOL_CNTR3, A        ;Store result from ACC.
                MOV        CH3_COUNT, #33          ;Init decrement amount.
                SETB       DEC_CHAN_3              ;Set dec channel 3 flag.
                AJMP       PROC_VX

      PRCV3:    JB         DEC_CHAN_4, PROC_VX     ;If inc or dec already
                JB         INC_CHAN_4, PROC_VX     ; set, then bail.
                CLR        DEC_VOL4_n              ;Activate decrement.
                LCALL      SET_REM_VOL             ;
                MOV        REM_VOL_CNTR4, A        ;Store result from ACC.
                MOV        CH4_COUNT, #33          ;Init decrement amount.
                SETB       DEC_CHAN_4              ;Set dec channel 4 flag.
                ;AJMP       PROC_VX

      PROC_VX:  LJMP       DEC11_X

                ;Process Tune command.
    PROC_T:     MOV        A, REC_BYTE_6           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_TX                 ;Bail if invalid character.
                MOV        REC_BYTE_6, A           ;Resave as binary.

                MOV        A, REC_BYTE_7           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_TX                 ;Bail if invalid character.
                MOV        REC_BYTE_7, A           ;Resave as binary.

                MOV        A, REC_BYTE_8           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_TX                 ;Bail if invalid character.
                MOV        REC_BYTE_8, A           ;Resave as binary.

                ;Nibbles are good. Pack them and send tune data to PLAY_TUNE.
                MOV        A, REC_BYTE_6           ;Load tempo number.
                JZ         PRCT0                   ;Update tune, normal speed.
                DEC        A
                JZ         PRCT1                   ;Not used.
                DEC        A
                JZ         PRCT2                   ;Update tune, fast speed.
                AJMP       PROC_TX                 ;Else bail.

      PRCT0:    MOV        A, REC_BYTE_7           ;Load MS nibble
                SWAP       A                       ;and move to high position.
                ORL        A, REC_BYTE_8           ;and overlay next nibble.
                MOV        TUNE_VAL, A             ;save them in variable,
                CLR        DBL_TIME                ;clear double-time flag.
                CLR        LAMP_TST_FLG            ;Turn off lamp test flag.
                CLR        PLAY_REST               ;CLeaR PLAY_REST flag.
                LCALL      PLAY_TUNE               ;Proc 8-bit tune number.
                AJMP       PROC_TX

      PRCT1:    AJMP       PROC_TX

      PRCT2:    MOV        A, REC_BYTE_7           ;Load MS nibble
                SWAP       A                       ;and move to high position.
                ORL        A, REC_BYTE_8           ;and overlay next nibble.
                MOV        TUNE_VAL, A             ;save them in variable,
                SETB       DBL_TIME                ;set double-time flag.
                CLR        LAMP_TST_FLG            ;Turn off lamp test flag.
                CLR        PLAY_REST               ;CLeaR PLAY_REST flag.
                LCALL      PLAY_TUNE               ;Proc 8-bit tune number.
                ;AJMP       PROC_TX

      PROC_TX:  LJMP       DEC11_X

                ;Execute MAX7219 direct write command.
    PROC_M:     MOV        A, REC_BYTE_6           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_MX                 ;Bail if invalid character.
                MOV        REC_BYTE_6, A           ;Resave as binary.

                MOV        A, REC_BYTE_7           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_MX                 ;Bail if invalid character.
                MOV        REC_BYTE_7, A           ;Resave as binary.

                MOV        A, REC_BYTE_8           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_MX                 ;Bail if invalid character.
                MOV        REC_BYTE_8, A           ;Resave as binary.

                MOV        A, REC_BYTE_9           ;Convert nibble to binary.
                LCALL      HEX2BIN                 ;
                JC         PROC_MX                 ;Bail if invalid character.
                MOV        REC_BYTE_9, A           ;Resave as binary.

                ;Now pack nibbles and send data to MAX7219.
                MOV        A, REC_BYTE_6           ;Load MS nibble
                SWAP       A                       ;and move to high position.
                ORL        A, REC_BYTE_7           ;and overlay next nibble.

                PUSH       ACC                     ;Save this byte.

                MOV        A, REC_BYTE_8           ;Load next nibble
                SWAP       A                       ;and move to high position.
                ORL        A, REC_BYTE_9           ;and overlay LS nibble.

                MOV        R0, A                   ;Load bytes into R0 and ACC
                POP        ACC
                LCALL      MAX7219_WRITE           ;and send to MAX7219.

      PROC_MX:  AJMP       DEC11_X

                ;Execute Set Terminator (Receive Downstream Enable) command.
    PROC_Z:     MOV        A, REC_BYTE_6           ;Load parameter.

      PRCZ0:    CJNE       A,#'0',PRCZ1
                ;Parameter=0: Disable reception from downstream
                CLR        MSTR_RDWN_EN
                SETB       RECV_DWNSTREAM_ENABLE_n ;Disable input from downstr.
                AJMP       PROC_ZX

      PRCZ1:    CJNE       A,#'1',PROC_ZX
                ;Parameter=1: Enable reception from downstream
                SETB       MSTR_RDWN_EN
                CLR        RECV_DWNSTREAM_ENABLE_n ;Enable input from downstr.

      PROC_ZX:  AJMP       DEC11_X

    PROC_X:
    DEC11_X:
;               MOV        SBUF,#'X'               ;print a marker
                LCALL      PARSE_RESET             ;Reset the PARSER.
                RET

;******************************************************************************
;                          Subroutine: WR_BYTE_RX_BFR
;******************************************************************************
;Read from the single byte UART receive buffer and write it to the multi-byte
;receive buffer in SRAM.  Then update buffer write pointer and flags.
;
;  PARAMETERS:
;    RECEIVES:
;      SBUF -- byte received by the UART, to be written to the receive buffer
;    RETURNS:
;      RX_BFR_OVF, RX_BFR_FUL, RX_BFR_EMT -- flags.
;      RX_BFR @ previous RX_BFR_WRPTR ------ Updated buffer location.
;      RX_BFR_WRPTR ------------------------ New buffer write pointer.
;    DESTROYS:
;      nothing (PUSHes and POPs all that it uses)
;
WR_BYTE_RX_BFR: ;Save context for clean return from interrupt.
                PUSH       ACC
                PUSH       PSW
                PUSH       AR0

                ;Test to see if there is room in the buffer for the new byte.
                JNB        RX_BFR_FUL, WBRXB2

  WBRXB1:       ;Buffer is full.  Discard data, set overflow flag and exit.
                SETB       RX_BFR_OVF
                LJMP       WBRXBX

  WBRXB2:       ;Write byte to buffer.
                MOV        A,   #RX_BFR            ;Load buffer base address.
                ADD        A,   RX_BFR_WRPTR       ;Add write pointer offset.
                MOV        R0,  A                  ;Move address to index reg.
                MOV        @R0, SBUF               ;Store UART byte to buffer.

                ;Update buffer write pointer.
                INC        RX_BFR_WRPTR
                MOV        A, RX_BFR_WRPTR
                CJNE       A, #RX_BFR_LEN, WBRXB3  ;Check for wrap condition.
                MOV        RX_BFR_WRPTR, #0        ;Wrap pointer back to start.

  WBRXB3:       ;Set full flag, if appropriate.
                MOV        A, RX_BFR_WRPTR
                CJNE       A, RX_BFR_RDPTR, WBRXBX ;RDPTR==WRPTR ?
                SETB       RX_BFR_FUL              ;  If equal, buffer is full.

  WBRXBX:       CLR        RX_BFR_EMT              ;Buffer is no longer empty.

                ;Restore the context that existed prior to the interrupt.
                POP        AR0
                POP        PSW
                POP        ACC

                RET

;******************************************************************************
;                          Subroutine: RD_BYTE_RX_BFR
;******************************************************************************
;Read byte from Receive Buffer and return it in REC_BYTE.  Then update buffer
;read pointer and flags.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      RX_BFR_FUL, RX_BFR_EMT -- flags.
;      RX_BFR_RDPTR ------------ New buffer read pointer.
;      REC_BYTE ---------------- Next byte read from buffer.
;    DESTROYS:
;      A, C, R0
;
RD_BYTE_RX_BFR: ;This routine should only be called after testing the
                ;RX_BFR_EMT flag to be sure the buffer contains data.
                ;Therefore it can be assumed that the buffer is not empty.

  RBRXB1:       ;Read next byte from buffer.
                MOV        A,        #RX_BFR       ;Load buffer base address.
                ADD        A,        RX_BFR_RDPTR  ;Add read pointer offset.
                MOV        R0,       A             ;Move address to R0.
                MOV        REC_BYTE, @R0           ;Read byte into REC_BYTE.

                ;Update buffer read pointer.
                CLR        EA                      ;Disable interrupts.
                INC        RX_BFR_RDPTR
                MOV        A, RX_BFR_RDPTR
                CJNE       A, #RX_BFR_LEN, RBRXB2  ;Check for wrap condition.
                MOV        RX_BFR_RDPTR, #0        ;Wrap pointer back to start.

  RBRXB2:       ;Set empty flag, if appropriate.
                MOV        A, RX_BFR_WRPTR
                CJNE       A, RX_BFR_RDPTR, RBRXBX ;RDPTR==WRPTR ?
                SETB       RX_BFR_EMT              ;  If equal, buffer is empty

  RBRXBX:       CLR        RX_BFR_FUL              ;Buffer is no longer full.
                SETB       EA                      ;Enable interrupts.
                RET

;******************************************************************************
;                          Subroutine: WR_BYTE_TX_BFR
;******************************************************************************
;Write byte passed in ACC to the Transmit Buffer.  Then update buffer write
;pointer and flags.
;
;  PARAMETERS:
;    RECEIVES:
;      A -- byte to be written to transmit buffer
;    RETURNS:
;      TX_BFR_OVF, TX_BFR_FUL, TX_BFR_EMT -- flags.
;      TX_BFR @ previous TX_BFR_WRPTR     -- Updated buffer location.
;      TX_BFR_WRPTR                       -- New buffer write pointer.
;    DESTROYS:
;      A, C, R0
;
WR_BYTE_TX_BFR: ;Test to see if there is room in the buffer for the new byte.
                JNB        TX_BFR_FUL, WBTXB2

  WBTXB1:       ;Buffer is full.
                ;  1) Discard the data,
                ;  2) Set overflow flag,
                ;  3) Overwrite a 'Z' into the buffer,
                ;  4) and exit.
                SETB       TX_BFR_OVF
                MOV        A,   #TX_BFR            ;Load buffer base address.
                ADD        A,   TX_BFR_WRPTR       ;Add write pointer offset.
                MOV        R0,  A                  ;Move result to indirect reg
                MOV        @R0, #'Z'               ;Overwrite 'Z' into buffer.
                AJMP       WBTXBX                  ;exit.

  WBTXB2:       ;Write byte to buffer.
                MOV        R0,  A                  ;Store the byte.
                MOV        A,   #TX_BFR            ;Load buffer base address.
                ADD        A,   TX_BFR_WRPTR       ;Add write pointer offset.
                XCH        A,   R0                 ;Swap data and address.
                MOV        @R0, A                  ;Store byte to buffer.

                ;Update buffer write pointer.
                INC        TX_BFR_WRPTR
                MOV        A, TX_BFR_WRPTR
                CJNE       A, #TX_BFR_LEN, WBTXB3  ;Check for wrap condition.
                MOV        TX_BFR_WRPTR, #0        ;Wrap pointer back to start.

  WBTXB3:       ;Set full flag, if appropriate.
                MOV        A, TX_BFR_WRPTR
                CJNE       A, TX_BFR_RDPTR, WBTXBX ;RDPTR==WRPTR ?
                SETB       TX_BFR_FUL              ;  If equal, buffer is full.

  WBTXBX:       CLR        TX_BFR_EMT              ;Buffer is no longer empty.
                RET

;******************************************************************************
;                         Subroutine: WR_STRING_TX_BFR
;******************************************************************************
;Write string to the Transmit Buffer.
;
;  PARAMETERS:
;    RECEIVES:
;      DPTR -- starting address of string to be written to transmit buffer.
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, DPTR
;
WR_STRING_TX_BFR:
                CLR        A
                MOVC       A, @A+DPTR              ;Get character.
                JZ         WSX                     ;0 indicates end of string.

                ACALL      WR_BYTE_TX_BFR

                INC        DPTR                    ;Inc pointer to next char.
                AJMP       WR_STRING_TX_BFR

  WSX:          RET

;******************************************************************************
;                              Subroutine: TX_BYTE
;******************************************************************************
;Read byte from Transmit Buffer and write to UART transmit register.  Then
;update buffer read pointer and flags.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      TX_BFR_FUL, TX_BFR_EMT, TX_ID_STR -- flags.
;      TX_BFR_RDPTR ----------------------- New buffer read pointer.
;    DESTROYS:
;      A, B, C, R0, DPTR
;
;This routine should only be called if:
;
;  1) TX_BFR_EMT=0 (i.e. the buffer is NOT empty)
;       --or--
;  2) TX_ID_STR=1 (PCB ID string xmit has been requested)
;
TX_BYTE:        JNB        TX_ID_STR, TXB1         ;Test for xmit type.

                ;Byte to xmit is from PCB ID string packet.
  TXI1:         ;  String packet is of the form:
                ;
                ;    STX A A AD-0941-001 v0.03 EOT
                ;
                ;  where STX and EOT are constants, A A is a two byte address
                ;  stored in SRAM, and the main text string is stored in ROM
                ;  at address "ASSEMBLY".
                ;
                ;  The ID_INDEX is used to walk through the packet to be
                ;  transmitted.  The INDEX value indicates which byte is to be
                ;  sent next, as follows:
                ;
                ;    0 -- string xmit is idle
                ;    1 -- send STX
                ;    2 -- send 1st address byte
                ;    3 -- send 2nd address byte
                ;    4 -- send 1st byte of string starting at location ASSEMBLY
                ;    5 to 0xFE send remaining bytes of string until 0 is seen
                ;    0xFF -- Send EOT, then terminate.
                ;
                ;  Typically the string will contain the PCB assembly number
                ;  followed by the firmware version number.
                ;
                MOV        A, ID_INDEX             ;Load ID_INDEX into ACC.

    TXIT1:      CJNE       A, #001h, TXIT2         ;Test ID_INDEX.
                MOV        SBUF, #STX              ;Write to UART: STX
                INC        ID_INDEX                ;Advance state.
                RET                                ;exit

    TXIT2:      CJNE       A, #002h, TXIT3         ;Test ID_INDEX.
                MOV        A, LOC_AD1              ;Get 1st address nibble
                LCALL      BIN2ASC                 ;convert to ASCII
                MOV        SBUF, A                 ;Write to UART: A1
                INC        ID_INDEX                ;Advance state.
                RET                                ;exit

    TXIT3:      CJNE       A, #003h, TXITFF        ;Test ID_INDEX.
                MOV        A, LOC_AD0              ;Get 2nd address nibble
                LCALL      BIN2ASC                 ;convert to ASCII
                MOV        SBUF, A                 ;Write to UART: A0
                INC        ID_INDEX                ;Advance state.
                RET                                ;exit

    TXITFF:     CJNE       A, #0FFh, TXIT4         ;Test ID_INDEX.
    TXITFFA:    MOV        SBUF, #EOT              ;Write to UART: EOT
                MOV        ID_INDEX, #0            ;Clear index.
                CLR        TX_ID_STR               ;Clear flag.
                RET                                ;exit

    TXIT4:      ;ID_INDEX is between 0x04 and 0xFE, inclusive.  Send next byte
                ;from ROM string at address ASSEMBLY.
                MOV        DPTR, #ASSEMBLY         ;Load start of string.
                CLR        C                       ;Subtract 4 from copy of
                SUBB       A, #4                   ;index to adjust for header.
                MOVC       A, @A+DPTR              ;Get byte from string.
                JZ         TXITFFA                 ;End of string detected.

                MOV        SBUF, A                 ;Write to UART: nxt str byte
                INC        ID_INDEX                ;Advance state.
                RET                                ;exit

                ;Byte to xmit is from TX_BFR (buffer in SRAM)
  TXB1:         ;Read next byte from buffer.
                MOV        A,    #TX_BFR           ;Load buffer base address.
                ADD        A,    TX_BFR_RDPTR      ;Add read pointer offset.
                MOV        R0,   A                 ;Move address to R0.
                MOV        SBUF, @R0               ;Read byte and write to UART

                ;Update buffer read pointer.
                INC        TX_BFR_RDPTR
                MOV        A, TX_BFR_RDPTR
                CJNE       A, #TX_BFR_LEN, TXB2    ;Check for wrap condition.
                MOV        TX_BFR_RDPTR, #0        ;Wrap pointer back to start.

  TXB2:         ;Set empty and clear request flags, if appropriate.
                MOV        A, TX_BFR_WRPTR
                CJNE       A, TX_BFR_RDPTR, TXBX   ;RDPTR==WRPTR ?
                SETB       TX_BFR_EMT              ;  If equal, buffer is empty

  TXBX:         CLR        TX_BFR_FUL              ;Buffer is no longer full.
                RET

;******************************************************************************
;                            Subroutine: DIRECT_DRV
;******************************************************************************
;Directly drive the outputs.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      nothing
;
;This routine is used to directly drive the outputs.  This routine is
;intended to be used as the default operating mode for the AD-0941-001 PCB up
;UNTIL its local address is set.
;
;The single received byte is interpreted as follows:
;
;  [(Decimal value of received byte) - (48 decimal)]
;
;This allows for keyboard keys "0" to "9" to be used to play tunes 0 to 9.
;
DIRECT_DRV:     ;

                ;If STX was seen, we should not be here. Bail.
                JB         STX_WAS_SEEN, DD_X

                MOV        TUNE_VAL, REC_BYTE

                MOV        A, TUNE_VAL             ;Move recv'd byte to ACC.
                CLR        C                       ;Clear carry for subtract.
                SUBB       A, #48                  ;Subtract ASCII offset.
                MOV        TUNE_VAL, A

                LCALL      PLAY_TUNE               ;Proc 8-bit tune number.

     DD_X:      

                RET

;******************************************************************************
;                              Subroutine: HEX2BIN
;******************************************************************************
;Convert an ASCII hex digit in ACC into its binary equivalent.
;
;
;  PARAMETERS:
;    RECEIVES:
;      A -- ASCII byte to be converted to HEX
;    RETURNS:
;      A -- converted HEX byte.
;      C -- set to indicate invalid hex digit.
;    DESTROYS:
;      A, C
;
HEX2BIN:        CLR        C
                SUBB       A,#'A'                  ;Lower to uppercase convrsn.
                JC         HEX01
                ANL        A,#0DFh
  HEX01:        ADD        A,#'A'

                ;The char is now uppercase and/or numeric
                CLR        C
                SUBB       A,#'0'                  ;if CHAR < '0'
                JC         HEXBIX                  ;  then EXIT
                SUBB       A,#10                   ;if CHAR <= '9'
                JC         HEX03                   ;  then IT IS NUMERIC

                ;If we get here, the char may be alphabetic.
                ADD        A,#10
                SUBB       A,#('A'-'0')            ;IF '9' < char < 'A'
                JC         HEXBIX                  ;  then error
                SUBB       A,#6
                JC         HEX02                   ;IF 'F' < char,
                SETB       C                       ;  then error
                AJMP       HEXBIX

  HEX02:        ADD        A,#6

  HEX03:        ADD        A,#10
                CLR        C

  HEXBIX:       RET

;******************************************************************************
;                              Subroutine: BIN2ASC
;******************************************************************************
;Convert the low nibble in ACC into its ASCII hex equivalent.
;
;  PARAMETERS:
;    RECEIVES:
;      A -- binary nibble (low 4 bits of byte) to be converted to ASCII.
;    RETURNS:
;      A -- converted ASCII byte.
;    DESTROYS:
;      A, C
;
BIN2ASC:        ANL        A, #0Fh                 ;Blank the high nibble.
                ADD        A, #30h                 ;Offset to ASCII "0".
                CJNE       A, #3Ah, B2A1           ;Use CJNE as  < comparator.
  B2A1:         JC         B2AX                    ;Jump if value < '9'+1
                ADD        A, #07h                 ;Add more to get to "A".
  B2AX:         RET

;******************************************************************************
;                          Subroutine: UPDATE_DISP
;******************************************************************************
;Send TUNE_VAL to Numeric Display.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, B, R0
;
;This routine updates the 3-digit numeric display with the tune number value.
;
UPDATE_DISP:      
                MOV        A, TUNE_VAL
                MOV        B, #100
                DIV        AB
                
                ADD        A,#'0'
                MOV        R0, A
                MOV        DAT_DIG1, A
                MOV        A, #MAX7219_ADR_DIGIT_0 ;Adr: MSD
                LCALL      PUT_CHAR

                MOV        A, B
                MOV        B, #10
                DIV        AB
 
                ADD        A,#'0'
                MOV        R0, A
                MOV        DAT_DIG2, A
                MOV        A, #MAX7219_ADR_DIGIT_1 ;Adr: 2nd digit
                LCALL      PUT_CHAR

                MOV        A, B
                ADD        A,#'0'
                MOV        R0, A
                MOV        DAT_DIG3, A
                MOV        A, #MAX7219_ADR_DIGIT_2 ;Adr: LSD
                LCALL      PUT_CHAR

                RET

;******************************************************************************
;                          Subroutine: GET_NOTE
;******************************************************************************
;Send TUNE_VAL to outputs.
;
;  PARAMETERS:
;    RECEIVES:
;      DPTR
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, R2, R3
;
;This routine is passed the location of the next note in DPTR. If the 
;note is number 49, the rest flag is set. This routine then retrieves 
;the duration that this note is to be played for. The double-time flag
;DBL_TIME is checked to determine if the duration must be halved.
;The final value is stored in NOTE_DURATION.
;
;The note number is used as an offset to the note storage
;look-up-table, NOTE_STOR_LUT. The timer 2 overflow is set with
;the value associated with the desired note.
;
GET_NOTE: 
                CLR        A
                MOVC       A,@A+DPTR               ;1st note number.

                DEC        A                       ;A=[(A-1)*2]+1
                MOV        B,#02                   ;
                MUL        AB                      ;
                INC        A                       ;             
                MOV        R2,A                    ;Save offset.

;Check to see if note number is 49 (REST). If it is, set
;  the PLAY_REST flag. Note number will be 97 after modifications above.

                CJNE       A,#97,GN_REST_NO_A
                SETB       PLAY_REST
                AJMP       GN_REST_NO_B

  GN_REST_NO_A: CLR        PLAY_REST

  GN_REST_NO_B: CLR        A
                INC        DPTR
                MOVC       A,@A+DPTR

                ;Check double time flag. If set, cut duration in half.
                JNB        DBL_TIME, GN_DBL_NO     ;Jump for no double-time.

   GN_DBL_YES:  MOV        B, #2                   ;Divide duration by 2,
                DIV        AB                      ; store in ACC.

    GN_DBL_NO:

     GN_DBL_X:

                MOV        NOTE_DURATION,A
                MOV        A,R2                    ;Restore offset.
                CALL       NOTE_STOR_LUT

                MOV        TH2, A                  ;Load hi byte T2 val.
                MOV        RCAP2H, A               ;Load hi byte T2 val.

                MOV        A,R2                    ;Restore offset.
                INC        A
                CALL       NOTE_STOR_LUT

                MOV        TL2, A                  ;Load lo byte T2 val.
                MOV        RCAP2L, A               ;Load lo byte T2 val.

                RET  

;******************************************************************************
;                          Subroutine: SET_REM_VOL
;******************************************************************************
;Determine number of increment pulses required to achieve desired volume.
;
;  PARAMETERS:
;    RECEIVES:
;      nothing
;    RETURNS:
;      A
;    DESTROYS:
;      A, B, R0
;
SET_REM_VOL:   
                MOV        A, REC_BYTE_7           ;Load MS nibble
                SWAP       A                       ;and move to high position.
                ORL        A, REC_BYTE_8           ;and overlay next nibble.
                MOV        R0, A                   ;Store recvd volume level.

                MOV        B, #3                   ;Divide received volume
                DIV        AB                      ; level (0-100) by 3.

                RET

;******************************************************************************
;                          Subroutine: PLAY_TUNE
;******************************************************************************
;Send TUNE_VAL to outputs.
;
;  PARAMETERS:
;    RECEIVES:
;      TUNE_VAL
;    RETURNS:
;      nothing
;    DESTROYS:
;      A, B, C, R0, R1, R2, DPTR
;
;This routine receives the desired tune number, TUNE_VAL. It first disables
;any currently playing tune in preparation of playing a new tune. It then
;checks the value of TUNE_VAL. If it is zero, it exits. Otherwise it
;initiates the process of playing a new tune, following these steps:
;
;(1) Select local data pointer, DPTR 1. 
;(2) Determine if tune < 128 and set flag and offset accordingly.
;(3) Store in DPTR the address of the appropriate LUT, TUNE_REF_LUTx.
;(4) Use DPTR to update DPTR with address of LUT + offset, TUNE_xxx.
;(5) Use DPTR to find the number of notes in the tune. Store this.
;(6) Call GET_NOTE to get the first note of the tune.
;(7) Enable flags as required to start playing the tune.
;(8) Return to standard data pointer, DPTR 0.
;
PLAY_TUNE:      ;Disable currently playing tune in preparation of new tune.
                CLR        ET2                     ;Disable timer 2 interrupt.
                CLR        TF2                     ;Clear timer 2 overflow flag
                MOV        T2CON, #00              ;Clear timer 2 control bits
                ;Among other things, this will stop the timer (TR2) and
                ;  cancel any pending timer 2 interrupt (TF2).

                ;MOV        T2MOD, #00              ;Clear the mode bits
                CLR        TUNE_ENBL               ;Prevent output toggling.
                CLR        TUNE_MICRO_OUT          ;Kill tune output.
                JB         LAMP_TST_FLG, PT_1      ;Skip display update if
                                                   ; lamp test is desired.  
                LCALL      UPDATE_DISP             ;Update numeric display.

    PT_1:       ;If tune = 0, turn off tune LED and exit else process tune.
                MOV        A, TUNE_VAL
                CJNE       A, #00, PT_2
                CLR        A
                SETB       TUNE_LED_n              ;Turn off tune LED.
                LJMP       PT_X                    ;Exit.

    PT_2:       ;Change to local DPTR and prepare to process recv'd byte.
                CLR        A 
                SELECT_DPTR1                       ;Select local DPTR.
                JB         DBL_TIME, PT_2_A        ;If double-time, skip
                                                   ; tune LED. Set in
                                                   ; SERV_FLASHER.
                CLR        TUNE_LED_n              ;Turn on Tune LED.
      PT_2_A:   MOV        A, TUNE_VAL             ;Move recv'd byte to ACC.

    PT_3:       ;Determine if  (0 < tune < 128) or  (127 < tune < 256).
                ;  Subtract 127 if (tune > 127).
                ;  Will use 1 of 2 LUTs. Set flag accordingly.
                CJNE       A, #128, PT_3_A         ;Use CJNE as < comparator.
      PT_3_A:   JC         PT_3_B                  ;Jump if tune # < 128.
                CLR        C
                SUBB       A, #127                 ;Offset tune to (1<--128).
                SETB       TUNE_OFFSET             ;Set tune offset flag.
                AJMP       PT_3_C

      PT_3_B:   CLR        TUNE_OFFSET             ;Clear tune offset flag.

      PT_3_C:   DEC        A                       ;Tune #'s 1-based.

      PT_3_D:   ;x = (recv'd byte - offset). The next few lines perform
                ;  x = (x*2), leaving the result in ACC and B (LSB,MSB).
                ;  LSB stored in R0. MSB discarded because x < 128. 
                MOV        B, #02
                MUL        AB
                MOV        R0,A

    PT_4:       ;Using DPTR, look up address of selected tune number
                ;  (e.g. address of TUNE_001). Store in R1 and R2.
                ;  Load DPTR with address.
                JNB        TUNE_OFFSET, PT_4_B
                                                   
      PT_4_A:   CLR        A                       ;*********************
                ADD        A,#LOW(TUNE_REF_LUT1)   ;Put address of 
                MOV        DP1L,A                  ;  TUNE_REF_LUT1
                CLR        A                       ;  into DPTR.
                ADDC       A,#HIGH(TUNE_REF_LUT1)  ;  (tunes 128-255)
                MOV        DP1H,A                  ;
                                                   ;*********************
                LJMP       PT_4_C
                                                   ;*********************
      PT_4_B:   CLR        A                       ;
                ADD        A,#LOW(TUNE_REF_LUT0)   ;Put address of 
                MOV        DP1L,A                  ;  TUNE_REF_LUT0
                CLR        A                       ;  into DPTR.
                ADDC       A,#HIGH(TUNE_REF_LUT0)  ;  (tunes 1-127)
                MOV        DP1H,A                  ;*********************

      PT_4_C:   MOV        A,R0                    ;*********************
                MOVC       A,@A+DPTR               ;Restore offset.
                MOV        R1,A                    ;
                INC        DPTR                    ;Store TUNExxx address
                MOV        A,R0                    ;  in R1,R2(MSB,LSB).
                MOVC       A,@A+DPTR               ;
                MOV        R2,A                    ;*********************

                CLR        A                       ;*********************
                ADD        A,R2                    ;
                MOV        DP1L,A                  ;Load DPTR with TUNExxx
                CLR        A                       ;  address.
                ADDC       A,R1                    ;
                MOV        DP1H,A                  ;*********************

    PT_5:       ;Using DPTR, look up the number of notes in the desired tune.
                CLR        A
                MOVC       A,@A+DPTR
                MOV        R1,A
                MOV        NOTE_COUNT,A
                INC        DPTR

                ;Call the routine to process the first note.
                LCALL      GET_NOTE

                ;Increment DPTR to prepare for next note.
                INC        DPTR

                SETB       TUNE_ENBL               ;Enable tune play.
                SETB       ET2                     ;Enable timer interrupt.
                SETB       TR2                     ;Start the timer.

    PT_X:       SELECT_DPTR0                       ;Return to "standard" DPTR

                RET

;******************************************************************************
;                   Look Up Table: TUNE_STOR_LUT
;******************************************************************************
;
TUNE_STOR_LUT:  ;255 Tunes with a variable number of notes per tune.
                ;The first byte of each record indicates the number of notes.

                ;Tune number 1 *****************************************
  TUNE_001:     DB    21       ;Length of this tune(notes).
                DB    26,77    ;Note 1 val(0-47),dur(10ms inc's).
                DB    23,77    ;Note 2
                DB    21,77    ;Note 3
                DB    14,77    ;Note 4
                DB    16,26    ;Note 5
                DB    18,26    ;Note 6
                DB    19,26    ;Note 7
                DB    16,58    ;Note 8
                DB    19,19    ;Note 9
                DB    14,77    ;Note 10
                DB    49,77    ;Note 11  (REST)
                DB    21,77    ;Note 12
                DB    26,77    ;Note 13
                DB    23,77    ;Note 14
                DB    19,77    ;Note 15
                DB    16,26    ;Note 16
                DB    18,26    ;Note 17
                DB    19,26    ;Note 18
                DB    21,58    ;Note 19
                DB    23,19    ;Note 20
                DB    21,77    ;Note 21

                ;Tune number 2 *****************************************
TUNE_002:       DB    14       ;Length of this tune(notes).
                DB    05,20    ;Note 1 val(0-47),dur(10ms inc's).
                DB    05,59    ;Note 2
                DB    05,20    ;Note 3
                DB    05,59    ;Note 4
                DB    03,20    ;Note 5
                DB    02,59    ;Note 6
                DB    05,20    ;Note 7
                DB    10,59    ;Note 8
                DB    12,20    ;Note 9
                DB    14,59    ;Note 10
                DB    14,20    ;Note 11
                DB    14,59    ;Note 12
                DB    12,20    ;Note 13
                DB    10,59    ;Note 14

                ;Tune number 3 *****************************************
  TUNE_003:     DB    15       ;Length of this tune(notes).
                DB    22,23    ;Note 1 val(0-47),dur(10ms inc's).
                DB    20,23    ;Note 2
                DB    18,46    ;Note 3
                DB    18,46    ;Note 4
                DB    18,23    ;Note 5
                DB    20,23    ;Note 6
                DB    22,23    ;Note 7
                DB    23,23    ;Note 8
                DB    25,46    ;Note 9
                DB    25,46    ;Note 10
                DB    25,46    ;Note 11
                DB    22,46    ;Note 12
                DB    27,46    ;Note 13
                DB    27,46    ;Note 14
                DB    27,46    ;Note 15

                ;Tune number 4 *****************************************
  TUNE_004:     DB    10       ;Length of this tune(notes).
                DB    11,23    ;Note 1 val(0-47),dur(10ms inc's).
                DB    16,23    ;Note 2
                DB    18,23    ;Note 3
                DB    20,23    ;Note 4
                DB    18,23    ;Note 5
                DB    16,23    ;Note 6
                DB    18,23    ;Note 7
                DB    20,46    ;Note 8
                DB    16,46    ;Note 9
                DB    16,46    ;Note 10

                ;Tune number 5 *****************************************
  TUNE_005:     DB    15       ;Length of this tune(notes).
                DB    14,15    ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,45    ;Note 2
                DB    23,15    ;Note 3
                DB    23,20    ;Note 4
                DB    21,20    ;Note 5
                DB    23,20    ;Note 6
                DB    24,61    ;Note 7
                DB    23,61    ;Note 8
                DB    21,45    ;Note 9
                DB    21,15    ;Note 10
                DB    21,20    ;Note 11
                DB    19,20    ;Note 12
                DB    21,20    ;Note 13
                DB    23,61    ;Note 14
                DB    19,61    ;Note 15

                ;Tune number 6 *****************************************                    
  TUNE_006:     DB    14       ;Length of this tune (notes).
                DB    7,44     ;Note 1 val (1-48), dur (10ms inc's).
                DB    9,44     ;Note 2
                DB    11,44    ;Note 3
                DB    7,44     ;Note 4
                DB    7,44     ;Note 5
                DB    9,44     ;Note 6
                DB    11,44    ;Note 7
                DB    7,44     ;Note 8
                DB    11,44    ;Note 9
                DB    12,44    ;Note 10
                DB    14,88    ;Note 11
                DB    11,44    ;Note 12
                DB    12,44    ;Note 13
                DB    14,88    ;Note 14

                ;Tune number 7 *****************************************                    
  TUNE_007:     DB    15        ;Length of this tune (notes).
                DB    5,20      ;Note 1 val (1-48), dur (10ms inc's).
                DB    7,39      ;Note 2
                DB    9,20      ;Note 3
                DB    10,39     ;Note 4
                DB    12,39     ;Note 5
                DB    10,79     ;Note 6
                DB    9,59      ;Note 7
                DB    5,20      ;Note 8
                DB    7,20      ;Note 9
                DB    7,20      ;Note 10
                DB    7,20      ;Note 11
                DB    7,20      ;Note 12
                DB    9,39      ;Note 13
                DB    9,39      ;Note 14
                DB    5,39      ;Note 15

                ;Tune number 8 *****************************************                    
  TUNE_008:     DB    19        ;Length of this tune (notes).
                DB    17,42     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,83     ;Note 2
                DB    22,42     ;Note 3
                DB    24,63     ;Note 4
                DB    25,21     ;Note 5
                DB    24,42     ;Note 6
                DB    22,83     ;Note 7
                DB    19,42     ;Note 8
                DB    15,63     ;Note 9
                DB    17,21     ;Note 10
                DB    19,42     ;Note 11
                DB    20,83     ;Note 12
                DB    17,42     ;Note 13
                DB    17,63     ;Note 14
                DB    16,21     ;Note 15
                DB    17,42     ;Note 16
                DB    19,83     ;Note 17
                DB    16,42     ;Note 18
                DB    12,83     ;Note 19

                ;Tune number 9 *****************************************                    
  TUNE_009:     DB    12        ;Length of this tune (notes).
                DB    28,83     ;Note 1 val (1-48), dur (10ms inc's).
                DB    28,63     ;Note 2
                DB    26,21     ;Note 3
                DB    26,42     ;Note 4
                DB    24,42     ;Note 5
                DB    19,83     ;Note 6
                DB    19,31     ;Note 7
                DB    21,10     ;Note 8
                DB    23,31     ;Note 9
                DB    19,10     ;Note 10
                DB    21,42     ;Note 11
                DB    19,42     ;Note 12

                ;Tune number 10 *****************************************                    
  TUNE_010:     DB    14        ;Length of this tune (notes).
                DB    13,83     ;Note 1 val (1-48), dur (10ms inc's).
                DB    9,28      ;Note 2
                DB    11,28     ;Note 3
                DB    9,28      ;Note 4
                DB    49,55     ;Note 5
                DB    13,83     ;Note 6
                DB    9,28      ;Note 7
                DB    11,28     ;Note 8
                DB    9,28      ;Note 9
                DB    49,83     ;Note 10
                DB    9,14      ;Note 11
                DB    9,14      ;Note 12
                DB    11,28     ;Note 13
                DB    9,28      ;Note 14

                ;Tune number 11 *****************************************                    
  TUNE_011:     DB    12        ;Length of this tune (notes).
                DB    18,25     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,25     ;Note 2
                DB    20,50     ;Note 3
                DB    18,50     ;Note 4
                DB    23,50     ;Note 5
                DB    22,100    ;Note 6
                DB    18,25     ;Note 7
                DB    18,25     ;Note 8
                DB    20,50     ;Note 9
                DB    18,50     ;Note 10
                DB    25,50     ;Note 11
                DB    23,100    ;Note 12

                ;Tune number 12 *****************************************                    
  TUNE_012:     DB    16        ;Length of this tune (notes).
                DB    25,20     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,39     ;Note 2
                DB    25,20     ;Note 3
                DB    27,39     ;Note 4
                DB    25,39     ;Note 5
                DB    22,20     ;Note 6
                DB    23,39     ;Note 7
                DB    22,20     ;Note 8
                DB    23,39     ;Note 9
                DB    22,39     ;Note 10
                DB    20,20     ;Note 11
                DB    22,39     ;Note 12
                DB    20,20     ;Note 13
                DB    22,39     ;Note 14
                DB    20,39     ;Note 15
                DB    15,79     ;Note 16

                ;Tune number 13 *****************************************                    
  TUNE_013:     DB    13        ;Length of this tune (notes).
                DB    16,23     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,23     ;Note 2
                DB    20,93     ;Note 3
                DB    21,47     ;Note 4
                DB    25,47     ;Note 5
                DB    23,93     ;Note 6
                DB    20,47     ;Note 7
                DB    23,47     ;Note 8
                DB    21,70     ;Note 9
                DB    20,23     ;Note 10
                DB    21,47     ;Note 11
                DB    18,47     ;Note 12
                DB    16,93     ;Note 13

                ;Tune number 14 *****************************************                    
  TUNE_014:     DB    16        ;Length of this tune (notes).
                DB    24,38     ;Note 1 val (1-48), dur (10ms inc's).
                DB    29,38     ;Note 2
                DB    31,38     ;Note 3
                DB    33,75     ;Note 4
                DB    24,38     ;Note 5
                DB    29,38     ;Note 6
                DB    31,38     ;Note 7
                DB    33,75     ;Note 8
                DB    33,38     ;Note 9
                DB    34,38     ;Note 10
                DB    33,38     ;Note 11
                DB    31,75     ;Note 12
                DB    31,38     ;Note 13
                DB    33,38     ;Note 14
                DB    31,38     ;Note 15
                DB    29,75     ;Note 16

                ;Tune number 15 *****************************************                    
  TUNE_015:     DB    14        ;Length of this tune (notes).
                DB    16,56     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,19     ;Note 2
                DB    16,19     ;Note 3
                DB    13,56     ;Note 4
                DB    16,56     ;Note 5
                DB    21,19     ;Note 6
                DB    25,75     ;Note 7
                DB    23,56     ;Note 8
                DB    21,19     ;Note 9
                DB    21,19     ;Note 10
                DB    23,56     ;Note 11
                DB    21,56     ;Note 12
                DB    18,19     ;Note 13
                DB    16,75     ;Note 14

                ;Tune number 16 *****************************************                    
  TUNE_016:     DB    13        ;Length of this tune (notes).
                DB    24,75     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,11     ;Note 2
                DB    24,32     ;Note 3
                DB    19,11     ;Note 4
                DB    24,32     ;Note 5
                DB    26,11     ;Note 6
                DB    28,86     ;Note 7
                DB    24,86     ;Note 8
                DB    29,75     ;Note 9
                DB    29,11     ;Note 10
                DB    24,43     ;Note 11
                DB    26,43     ;Note 12
                DB    28,86     ;Note 13

                ;Tune number 17 *****************************************                    
  TUNE_017:     DB    11        ;Length of this tune (notes).
                DB    23,28     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,28     ;Note 2
                DB    23,56     ;Note 3
                DB    23,28     ;Note 4
                DB    23,28     ;Note 5
                DB    23,56     ;Note 6
                DB    23,28     ;Note 7
                DB    26,28     ;Note 8
                DB    19,42     ;Note 9
                DB    21,14     ;Note 10
                DB    23,56     ;Note 11

                ;Tune number 18 *****************************************                    
  TUNE_018:     DB    10        ;Length of this tune (notes).
                DB    19,19     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,19     ;Note 2
                DB    19,19     ;Note 3
                DB    24,58     ;Note 4
                DB    28,58     ;Note 5
                DB    19,19     ;Note 6
                DB    19,19     ;Note 7
                DB    19,19     ;Note 8
                DB    24,58     ;Note 9
                DB    28,58     ;Note 10

                ;Tune number 19 *****************************************                    
  TUNE_019:     DB    14        ;Length of this tune (notes).
                DB    22,86     ;Note 1 val (1-48), dur (10ms inc's).
                DB    22,86     ;Note 2
                DB    22,28     ;Note 3
                DB    19,28     ;Note 4
                DB    27,28     ;Note 5
                DB    22,28     ;Note 6
                DB    19,28     ;Note 7
                DB    15,28     ;Note 8
                DB    17,28     ;Note 9
                DB    19,28     ;Note 10
                DB    20,28     ;Note 11
                DB    19,65     ;Note 12
                DB    17,22     ;Note 13
                DB    15,86     ;Note 14

                ;Tune number 20 *****************************************                    
  TUNE_020:     DB    15        ;Length of this tune (notes).
                DB    14,54     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,27     ;Note 2
                DB    19,27     ;Note 3
                DB    19,54     ;Note 4
                DB    21,54     ;Note 5
                DB    23,27     ;Note 6
                DB    23,27     ;Note 7
                DB    23,54     ;Note 8
                DB    23,54     ;Note 9
                DB    21,27     ;Note 10
                DB    23,27     ;Note 11
                DB    24,54     ;Note 12
                DB    18,54     ;Note 13
                DB    21,54     ;Note 14
                DB    19,54     ;Note 15

                ;Tune number 21 *****************************************                    
  TUNE_021:     DB    8         ;Length of this tune (notes).
                DB    24,43     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,21     ;Note 2
                DB    19,21     ;Note 3
                DB    21,43     ;Note 4
                DB    19,43     ;Note 5
                DB    49,43     ;Note 6
                DB    23,43     ;Note 7
                DB    24,43     ;Note 8

                ;Tune number 22 *****************************************                    
  TUNE_022:     DB    14        ;Length of this tune (notes).
                DB    21,40     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,40     ;Note 2
                DB    28,40     ;Note 3
                DB    28,40     ;Note 4
                DB    30,40     ;Note 5
                DB    30,40     ;Note 6
                DB    28,80     ;Note 7
                DB    26,40     ;Note 8
                DB    26,40     ;Note 9
                DB    25,40     ;Note 10
                DB    25,40     ;Note 11
                DB    23,40     ;Note 12
                DB    23,40     ;Note 13
                DB    21,40     ;Note 14

                ;Tune number 23 *****************************************                    
  TUNE_023:     DB    9         ;Length of this tune (notes).
                DB    31,61     ;Note 1 val (1-48), dur (10ms inc's).
                DB    24,61     ;Note 2
                DB    26,61     ;Note 3
                DB    19,61     ;Note 4
                DB    49,61     ;Note 5
                DB    19,61     ;Note 6
                DB    26,61     ;Note 7
                DB    31,61     ;Note 8
                DB    24,61     ;Note 9

                ;Tune number 24 *****************************************                    
  TUNE_024:     DB    14        ;Length of this tune (notes).
                DB    12,40     ;Note 1 val (1-48), dur (10ms inc's).
                DB    12,40     ;Note 2
                DB    14,40     ;Note 3
                DB    16,40     ;Note 4
                DB    12,40     ;Note 5
                DB    16,40     ;Note 6
                DB    14,40     ;Note 7
                DB    49,40     ;Note 8
                DB    12,40     ;Note 9
                DB    12,40     ;Note 10
                DB    14,40     ;Note 11
                DB    16,40     ;Note 12
                DB    12,80     ;Note 13
                DB    11,40     ;Note 14

                ;Tune number 25 *****************************************                    
  TUNE_025:     DB    8         ;Length of this tune (notes).
                DB    16,30     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,30     ;Note 2
                DB    23,30     ;Note 3
                DB    25,60     ;Note 4
                DB    16,30     ;Note 5
                DB    21,30     ;Note 6
                DB    23,30     ;Note 7
                DB    25,60     ;Note 8

                ;Tune number 26 *****************************************                    
  TUNE_026:     DB    7         ;Length of this tune (notes).
                DB    16,56     ;Note 1 val (1-48), dur (10ms inc's).
                DB    28,19     ;Note 2
                DB    25,25     ;Note 3
                DB    23,25     ;Note 4
                DB    21,25     ;Note 5
                DB    23,75     ;Note 6
                DB    18,75     ;Note 7

                ;Tune number 27 *****************************************                    
  TUNE_027:     DB    6         ;Length of this tune (notes).
                DB    12,12     ;Note 1 val (1-48), dur (10ms inc's).
                DB    17,12     ;Note 2
                DB    21,12     ;Note 3
                DB    24,66     ;Note 4
                DB    21,9      ;Note 5
                DB    24,38     ;Note 6

                ;Tune number 28 *****************************************                    
  TUNE_028:     DB    7         ;Length of this tune (notes).
                DB    16,64     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,21     ;Note 2
                DB    20,64     ;Note 3
                DB    16,21     ;Note 4
                DB    20,43     ;Note 5
                DB    16,43     ;Note 6
                DB    20,43     ;Note 7

                ;Tune number 29 *****************************************                    
  TUNE_029:     DB    8         ;Length of this tune (notes).
                DB    23,25     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,25     ;Note 2
                DB    23,25     ;Note 3
                DB    19,75     ;Note 4
                DB    21,25     ;Note 5
                DB    21,25     ;Note 6
                DB    21,25     ;Note 7
                DB    18,75     ;Note 8

                ;Tune number 30 *****************************************                    
  TUNE_030:     DB    14        ;Length of this tune (notes).
                DB    21,56     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,19     ;Note 2
                DB    25,38     ;Note 3
                DB    21,38     ;Note 4
                DB    23,56     ;Note 5
                DB    23,19     ;Note 6
                DB    23,75     ;Note 7
                DB    23,56     ;Note 8
                DB    23,19     ;Note 9
                DB    26,38     ;Note 10
                DB    23,38     ;Note 11
                DB    25,56     ;Note 12
                DB    25,19     ;Note 13
                DB    25,75     ;Note 14

                ;Tune number 31 *****************************************                    
  TUNE_031:     DB    8         ;Length of this tune (notes).
                DB    20,42     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,42     ;Note 2
                DB    22,21     ;Note 3
                DB    25,21     ;Note 4
                DB    24,21     ;Note 5
                DB    22,21     ;Note 6
                DB    27,42     ;Note 7
                DB    27,42     ;Note 8

                ;Tune number 32 *****************************************                    
  TUNE_032:     DB    8         ;Length of this tune (notes).
                DB    18,38     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,38     ;Note 2
                DB    21,19     ;Note 3
                DB    23,56     ;Note 4
                DB    18,38     ;Note 5
                DB    18,38     ;Note 6
                DB    16,19     ;Note 7
                DB    13,56     ;Note 8

                ;Tune number 33 *****************************************                    
  TUNE_033:     DB    12        ;Length of this tune (notes).
                DB    24,35     ;Note 1 val (1-48), dur (10ms inc's).
                DB    22,12     ;Note 2
                DB    20,23     ;Note 3
                DB    22,23     ;Note 4
                DB    24,23     ;Note 5
                DB    23,23     ;Note 6
                DB    24,23     ;Note 7
                DB    20,23     ;Note 8
                DB    22,23     ;Note 9
                DB    22,23     ;Note 10
                DB    22,23     ;Note 11
                DB    22,46     ;Note 12

                ;Tune number 34 *****************************************                    
  TUNE_034:     DB    24        ;Length of this tune (notes).
                DB    19,44     ;Note 1 val (1-48), dur (10ms inc's).
                DB    49,11     ;Note 2
                DB    24,19     ;Note 3
                DB    49,08     ;Note 4
                DB    28,19     ;Note 5
                DB    49,08     ;Note 6
                DB    24,19     ;Note 7
                DB    49,08     ;Note 8
                DB    19,19     ;Note 9
                DB    49,08     ;Note 10
                DB    17,18     ;Note 11
                DB    26,36     ;Note 12
                DB    49,10     ;Note 13
                DB    22,33     ;Note 14
                DB    23,22     ;Note 16
                DB    49,05     ;Note 17
                DB    26,22     ;Note 18
                DB    49,05     ;Note 19
                DB    23,22     ;Note 20
                DB    49,05     ;Note 21
                DB    19,22     ;Note 22
                DB    49,08     ;Note 23
                DB    16,18     ;Note 24
                DB    24,36     ;Note 25

                ;Tune number 35 *****************************************                    
  TUNE_035:     DB    11        ;Length of this tune (notes).
                DB    16,17     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,17     ;Note 2
                DB    16,33     ;Note 3
                DB    16,17     ;Note 4
                DB    16,17     ;Note 5
                DB    16,33     ;Note 6
                DB    16,17     ;Note 7
                DB    16,17     ;Note 8
                DB    21,33     ;Note 9
                DB    23,33     ;Note 10
                DB    25,67     ;Note 11

                ;Tune number 36 *****************************************                    
  TUNE_036:     DB    13        ;Length of this tune (notes).
                DB    23,43     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,14     ;Note 2
                DB    23,43     ;Note 3
                DB    23,14     ;Note 4
                DB    23,43     ;Note 5
                DB    23,14     ;Note 6
                DB    23,43     ;Note 7
                DB    23,14     ;Note 8
                DB    25,43     ;Note 9
                DB    23,14     ;Note 10
                DB    22,43     ;Note 11
                DB    20,14     ;Note 12
                DB    18,57     ;Note 13

                ;Tune number 37 *****************************************                    
  TUNE_037:     DB    15        ;Length of this tune (notes).
                DB    25,38     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,13     ;Note 2
                DB    29,25     ;Note 3
                DB    32,25     ;Note 4
                DB    32,38     ;Note 5
                DB    34,13     ;Note 6
                DB    32,25     ;Note 7
                DB    29,25     ;Note 8
                DB    25,38     ;Note 9
                DB    27,13     ;Note 10
                DB    29,25     ;Note 11
                DB    29,25     ;Note 12
                DB    27,25     ;Note 13
                DB    27,25     ;Note 14
                DB    25,50     ;Note 15

                ;Tune number 38 *****************************************                    
  TUNE_038:     DB    18        ;Length of this tune (notes).
                DB    23,36     ;Note 1 val (1-48), dur (10ms inc's).
                DB    28,36     ;Note 2
                DB    23,36     ;Note 3
                DB    16,36     ;Note 4
                DB    23,36     ;Note 5
                DB    28,36     ;Note 6
                DB    23,63     ;Note 7
                DB    24,2      ;Note 8
                DB    25,2      ;Note 9
                DB    26,2      ;Note 10
                DB    27,2      ;Note 11
                DB    28,63     ;Note 12
                DB    25,18      ;Note 13
                DB    23,22     ;Note 14
                DB    21,42     ;Note 15
                DB    20,36     ;Note 16
                DB    18,36     ;Note 17
                DB    16,36     ;Note 18

                ;Tune number 39 *****************************************                    
  TUNE_039:     DB    21        ;Length of this tune (notes).
                DB    25,22     ;Note 1 val (1-48), dur (10ms inc's).
                DB    24,22     ;Note 2
                DB    25,22     ;Note 3
                DB    22,22     ;Note 4
                DB    21,22     ;Note 5
                DB    22,22     ;Note 6
                DB    18,22     ;Note 7
                DB    17,22     ;Note 8
                DB    18,22     ;Note 9
                DB    13,43     ;Note 10
                DB    49,22     ;Note 11
                DB    10,22     ;Note 12
                DB    11,22     ;Note 13
                DB    13,22     ;Note 14
                DB    15,22     ;Note 15
                DB    17,22     ;Note 16
                DB    18,22     ;Note 17
                DB    20,22     ;Note 18
                DB    22,22     ;Note 19
                DB    23,22     ;Note 20
                DB    20,43     ;Note 21

                ;Tune number 40 *****************************************                    
  TUNE_040:     DB    12        ;Length of this tune (notes).
                DB    11,43     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,43     ;Note 2
                DB    15,43     ;Note 3
                DB    16,43     ;Note 4
                DB    18,43     ;Note 5
                DB    13,43     ;Note 6
                DB    18,86     ;Note 7
                DB    16,43     ;Note 8
                DB    15,43     ;Note 9
                DB    13,43     ;Note 10
                DB    15,43     ;Note 11
                DB    16,86     ;Note 12

                ;Tune number 41 *****************************************                    
  TUNE_041:     DB    14        ;Length of this tune (notes).
                DB    13,37     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,55     ;Note 2
                DB    17,18     ;Note 3
                DB    18,37     ;Note 4
                DB    22,37     ;Note 5
                DB    20,55     ;Note 6
                DB    18,18     ;Note 7
                DB    20,37     ;Note 8
                DB    22,37     ;Note 9
                DB    18,55     ;Note 10
                DB    18,18     ;Note 11
                DB    22,37     ;Note 12
                DB    25,37     ;Note 13
                DB    27,73     ;Note 14

                ;Tune number 42 *****************************************                    
  TUNE_042:     DB    16        ;Length of this tune (notes).
                DB    20,48     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,48     ;Note 2
                DB    16,96     ;Note 3
                DB    20,48     ;Note 4
                DB    18,24     ;Note 5
                DB    18,24     ;Note 6
                DB    16,96     ;Note 7
                DB    23,48     ;Note 8
                DB    28,36     ;Note 9
                DB    28,12     ;Note 10
                DB    27,16     ;Note 11
                DB    25,16     ;Note 12
                DB    27,16     ;Note 13
                DB    28,36     ;Note 14
                DB    23,12     ;Note 15
                DB    23,24     ;Note 16

                ;Tune number 43 *****************************************                    
  TUNE_043:     DB    11        ;Length of this tune (notes).
                DB    13,33     ;Note 1 val (1-48), dur (10ms inc's).
                DB    17,33     ;Note 2
                DB    20,50     ;Note 3
                DB    17,17     ;Note 4
                DB    20,17     ;Note 5
                DB    22,50     ;Note 6
                DB    20,67     ;Note 7
                DB    17,33     ;Note 8
                DB    20,33     ;Note 9
                DB    22,133    ;Note 10
                DB    20,33     ;Note 11

                ;Tune number 44 *****************************************                    
  TUNE_044:     DB    15        ;Length of this tune (notes).
                DB    18,40     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,40     ;Note 2
                DB    22,40     ;Note 3
                DB    23,40     ;Note 4
                DB    25,40     ;Note 5
                DB    27,40     ;Note 6
                DB    29,40     ;Note 7
                DB    30,40     ;Note 8
                DB    29,40     ;Note 9
                DB    27,40     ;Note 10
                DB    25,40     ;Note 11
                DB    23,40     ;Note 12
                DB    22,40     ;Note 13
                DB    20,40     ;Note 14
                DB    18,40     ;Note 15

                ;Tune number 45 *****************************************                    
  TUNE_045:     DB    16        ;Length of this tune (notes).
                DB    16,20     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,40     ;Note 2
                DB    16,20     ;Note 3
                DB    20,40     ;Note 4
                DB    18,30     ;Note 5
                DB    16,10     ;Note 6
                DB    16,10     ;Note 7
                DB    21,30     ;Note 8
                DB    49,40     ;Note 9
                DB    9,13      ;Note 10
                DB    11,13     ;Note 11
                DB    13,13     ;Note 12
                DB    11,30     ;Note 13
                DB    13,10     ;Note 14
                DB    9,40      ;Note 15
                DB    9,40      ;Note 16

                ;Tune number 46 *****************************************                    
  TUNE_046:     DB    24        ;Length of this tune (notes).
                DB    18,33     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,33     ;Note 2
                DB    20,33     ;Note 3
                DB    18,33     ;Note 4
                DB    21,17     ;Note 5
                DB    22,17     ;Note 6
                DB    18,33     ;Note 7
                DB    23,33     ;Note 8
                DB    22,33     ;Note 9
                DB    18,33     ;Note 10
                DB    18,33     ;Note 11
                DB    20,33     ;Note 12
                DB    18,33     ;Note 13
                DB    21,17     ;Note 14
                DB    22,17     ;Note 15
                DB    18,33     ;Note 16
                DB    23,33     ;Note 17
                DB    22,33     ;Note 18
                DB    16,100    ;Note 19
                DB    13,33     ;Note 20
                DB    49,133    ;Note 21
                DB    16,133    ;Note 22
                DB    25,33     ;Note 23
                DB    19,33     ;Note 24

                ;Tune number 47 *****************************************                    
  TUNE_047:     DB    17        ;Length of this tune (notes).
                DB    16,15     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,46     ;Note 2
                DB    25,15     ;Note 3
                DB    21,46     ;Note 4
                DB    16,15     ;Note 5
                DB    21,46     ;Note 6
                DB    25,15     ;Note 7
                DB    21,46     ;Note 8
                DB    25,15     ;Note 9
                DB    21,46     ;Note 10
                DB    23,15     ;Note 11
                DB    25,20     ;Note 12
                DB    23,20     ;Note 13
                DB    21,20     ;Note 14
                DB    20,46     ;Note 15
                DB    21,15     ;Note 16
                DB    23,61     ;Note 17

                ;Tune number 48 *****************************************                    
  TUNE_048:     DB    10        ;Length of this tune (notes).
                DB    16,44     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,44     ;Note 2
                DB    16,44     ;Note 3
                DB    13,44     ;Note 4
                DB    9,67      ;Note 5
                DB    18,22     ;Note 6
                DB    16,22     ;Note 7
                DB    13,22     ;Note 8
                DB    16,44     ;Note 9
                DB    11,44     ;Note 10

                ;Tune number 49 *****************************************                    
  TUNE_049:     DB    18        ;Length of this tune (notes).
                DB    16,56     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,28     ;Note 2
                DB    25,14     ;Note 3
                DB    21,14     ;Note 4
                DB    16,28     ;Note 5
                DB    25,28     ;Note 6
                DB    21,28     ;Note 7
                DB    25,14     ;Note 8
                DB    21,14     ;Note 9
                DB    16,28     ;Note 10
                DB    25,28     ;Note 11
                DB    21,28     ;Note 12
                DB    25,14     ;Note 13
                DB    21,14     ;Note 14
                DB    16,28     ;Note 15
                DB    21,28     ;Note 16
                DB    25,56     ;Note 17
                DB    21,56     ;Note 18

                ;Tune number 50 *****************************************                    
  TUNE_050:     DB    13        ;Length of this tune (notes).
                DB    29,29     ;Note 1 val (1-48), dur (10ms inc's).
                DB    28,29     ;Note 2
                DB    29,29     ;Note 3
                DB    28,29     ;Note 4
                DB    29,29     ;Note 5
                DB    24,29     ;Note 6
                DB    27,29     ;Note 7
                DB    25,29     ;Note 8
                DB    22,88     ;Note 9
                DB    13,29     ;Note 10
                DB    17,29     ;Note 11
                DB    22,29     ;Note 12
                DB    24,88     ;Note 13

                ;Tune number 51 *****************************************                    
  TUNE_051:     DB    14        ;Length of this tune (notes).
                DB    22,18     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,36     ;Note 2
                DB    30,36     ;Note 3
                DB    29,36     ;Note 4
                DB    27,18     ;Note 5
                DB    25,36     ;Note 6
                DB    23,71     ;Note 7
                DB    20,18     ;Note 8
                DB    23,36     ;Note 9
                DB    29,36     ;Note 10
                DB    27,36     ;Note 11
                DB    25,18     ;Note 12
                DB    23,36     ;Note 13
                DB    22,71     ;Note 14

                ;Tune number 52 *****************************************                    
  TUNE_052:     DB    13        ;Length of this tune (notes).
                DB    27,38     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,13     ;Note 2
                DB    23,25     ;Note 3
                DB    25,25     ;Note 4
                DB    27,25     ;Note 5
                DB    27,25     ;Note 6
                DB    27,50     ;Note 7
                DB    25,25     ;Note 8
                DB    25,25     ;Note 9
                DB    25,50     ;Note 10
                DB    27,25     ;Note 11
                DB    30,25     ;Note 12
                DB    30,50     ;Note 13

                ;Tune number 53 *****************************************                    
  TUNE_053:     DB    16        ;Length of this tune (notes).
                DB    16,41     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,41     ;Note 2
                DB    16,41     ;Note 3
                DB    15,61     ;Note 4
                DB    16,20     ;Note 5
                DB    18,41     ;Note 6
                DB    20,41     ;Note 7
                DB    20,41     ;Note 8
                DB    21,41     ;Note 9
                DB    20,61     ;Note 10
                DB    18,20     ;Note 11
                DB    16,41     ;Note 12
                DB    18,41     ;Note 13
                DB    16,41     ;Note 14
                DB    15,41     ;Note 15
                DB    16,41     ;Note 16

                ;Tune number 54 *****************************************                    
  TUNE_054:     DB    19        ;Length of this tune (notes).
                DB    28,35     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,35     ;Note 2
                DB    28,35     ;Note 3
                DB    23,70     ;Note 4
                DB    20,70     ;Note 5
                DB    18,35     ;Note 6
                DB    17,35     ;Note 7
                DB    18,35     ;Note 8
                DB    25,104    ;Note 9
                DB    23,35     ;Note 10
                DB    27,35     ;Note 11
                DB    25,35     ;Note 12
                DB    25,35     ;Note 13
                DB    23,35     ;Note 14
                DB    21,35     ;Note 15
                DB    21,35     ;Note 16
                DB    20,35     ;Note 17
                DB    18,35     ;Note 18
                DB    16,35     ;Note 19

                ;Tune number 55 *****************************************                    
  TUNE_055:     DB    20        ;Length of this tune (notes).
                DB    16,21     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,21     ;Note 2
                DB    21,21     ;Note 3
                DB    23,21     ;Note 4
                DB    25,42     ;Note 5
                DB    21,11     ;Note 6
                DB    20,11     ;Note 7
                DB    18,21     ;Note 8
                DB    26,21     ;Note 9
                DB    26,21     ;Note 10
                DB    26,42     ;Note 11
                DB    26,11     ;Note 12
                DB    26,11     ;Note 13
                DB    25,32     ;Note 14
                DB    23,11     ;Note 15
                DB    21,11     ;Note 16
                DB    20,11     ;Note 17
                DB    21,11     ;Note 18
                DB    23,11     ;Note 19
                DB    21,42     ;Note 20

                ;Tune number 56 *****************************************                    
  TUNE_056:     DB    18        ;Length of this tune (notes).
                DB    16,35     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,35     ;Note 2
                DB    25,35     ;Note 3
                DB    23,70     ;Note 4
                DB    21,35     ;Note 5
                DB    13,35     ;Note 6
                DB    16,35     ;Note 7
                DB    21,35     ;Note 8
                DB    20,104    ;Note 9
                DB    16,35     ;Note 10
                DB    20,35     ;Note 11
                DB    26,35     ;Note 12
                DB    25,70     ;Note 13
                DB    23,35     ;Note 14
                DB    23,35     ;Note 15
                DB    21,35     ;Note 16
                DB    18,35     ;Note 17
                DB    16,70     ;Note 18

                ;Tune number 57 *****************************************                    
  TUNE_057:     DB    11        ;Length of this tune (notes).
                DB    23,47     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,16     ;Note 2
                DB    23,31     ;Note 3
                DB    21,31     ;Note 4
                DB    20,31     ;Note 5
                DB    21,31     ;Note 6
                DB    23,63     ;Note 7
                DB    18,63     ;Note 8
                DB    23,63     ;Note 9
                DB    20,31     ;Note 10
                DB    16,94     ;Note 11

                ;Tune number 58 *****************************************                    
  TUNE_058:     DB    15        ;Length of this tune (notes).
                DB    13,40     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,27     ;Note 2
                DB    22,26     ;Note 3
                DB    13,26     ;Note 4
                DB    18,26     ;Note 5
                DB    22,26     ;Note 6
                DB    13,26     ;Note 7
                DB    18,20     ;Note 8
                DB    22,100    ;Note 9
                DB    23,40     ;Note 10
                DB    22,60     ;Note 11
                DB    22,20     ;Note 12
                DB    20,60     ;Note 13
                DB    20,20     ;Note 14
                DB    18,80     ;Note 15

                ;Tune number 59 *****************************************                    
  TUNE_059:     DB    10        ;Length of this tune (notes).
                DB    16,58     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,117    ;Note 2
                DB    25,19     ;Note 3
                DB    23,19     ;Note 4
                DB    21,19     ;Note 5
                DB    25,117    ;Note 6
                DB    23,58     ;Note 7
                DB    21,117    ;Note 8
                DB    18,58     ;Note 9
                DB    16,117    ;Note 10

                ;Tune number 60 *****************************************                    
  TUNE_060:     DB    19        ;Length of this tune (notes).
                DB    16,32     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,32     ;Note 2
                DB    19,32     ;Note 3
                DB    21,32     ;Note 4
                DB    49,64     ;Note 5
                DB    21,32     ;Note 6
                DB    24,32     ;Note 7
                DB    24,32     ;Note 8
                DB    23,32     ;Note 9
                DB    49,64     ;Note 10
                DB    19,32     ;Note 11
                DB    21,32     ;Note 12
                DB    21,32     ;Note 13
                DB    19,32     ;Note 14
                DB    49,64     ;Note 15
                DB    19,32     ;Note 16
                DB    26,32     ;Note 17
                DB    26,32     ;Note 18
                DB    24,32     ;Note 19

                ;Tune number 61 *****************************************                    
  TUNE_061:     DB    12        ;Length of this tune (notes).
                DB    9,25      ;Note 1 val (1-48), dur (10ms inc's).
                DB    9,25      ;Note 2
                DB    12,25     ;Note 3
                DB    13,25     ;Note 4
                DB    13,25     ;Note 5
                DB    16,25     ;Note 6
                DB    16,25     ;Note 7
                DB    18,25     ;Note 8
                DB    18,25     ;Note 9
                DB    16,25     ;Note 10
                DB    13,25     ;Note 11
                DB    9,25      ;Note 12

                ;Tune number 62 *****************************************                    
  TUNE_062:     DB    15        ;Length of this tune (notes).
                DB    9,42      ;Note 1 val (1-48), dur (10ms inc's).
                DB    9,42      ;Note 2
                DB    10,42     ;Note 3
                DB    11,64     ;Note 4
                DB    11,21     ;Note 5
                DB    15,64     ;Note 6
                DB    15,21     ;Note 7
                DB    16,64     ;Note 8
                DB    16,21     ;Note 9
                DB    17,64     ;Note 10
                DB    17,21     ;Note 11
                DB    18,64     ;Note 12
                DB    20,42     ;Note 13
                DB    22,42     ;Note 14
                DB    23,85     ;Note 15

                ;Tune number 63 *****************************************                    
  TUNE_063:     DB    25        ;Length of this tune (notes).
                DB    23,31     ;Note 1 val (1-48), dur (10ms inc's).
                DB    22,31     ;Note 2
                DB    21,16     ;Note 3
                DB    22,16     ;Note 4
                DB    21,16     ;Note 5
                DB    22,16     ;Note 6
                DB    21,31     ;Note 7
                DB    20,31     ;Note 8
                DB    19,31     ;Note 9
                DB    20,31     ;Note 10
                DB    20,31     ;Note 11
                DB    19,31     ;Note 12
                DB    18,16     ;Note 13
                DB    19,16     ;Note 14
                DB    18,16     ;Note 15
                DB    19,16     ;Note 16
                DB    18,31     ;Note 17
                DB    17,31     ;Note 18
                DB    16,31     ;Note 19
                DB    17,31     ;Note 20
                DB    20,31     ;Note 21
                DB    15,16     ;Note 22
                DB    15,16     ;Note 23
                DB    14,31     ;Note 24
                DB    15,31     ;Note 25

                ;Tune number 64 *****************************************                    
  TUNE_064:     DB    13        ;Length of this tune (notes).
                DB    25,38     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,19     ;Note 2
                DB    22,19     ;Note 3
                DB    20,19     ;Note 4
                DB    25,38     ;Note 5
                DB    22,58     ;Note 6
                DB    49,115    ;Note 7
                DB    20,38     ;Note 8
                DB    20,19     ;Note 9
                DB    17,19     ;Note 10
                DB    20,19     ;Note 11
                DB    17,38     ;Note 12
                DB    15,58     ;Note 13

                ;Tune number 65 *****************************************                    
  TUNE_065:     DB    12        ;Length of this tune (notes).
                DB    15,33     ;Note 1 val (1-48), dur (10ms inc's).
                DB    17,33     ;Note 2
                DB    20,17     ;Note 3
                DB    20,33     ;Note 4
                DB    20,17     ;Note 5
                DB    20,33     ;Note 6
                DB    20,33     ;Note 7
                DB    17,33     ;Note 8
                DB    15,33     ;Note 9
                DB    12,33     ;Note 10
                DB    15,33     ;Note 11
                DB    20,67     ;Note 12

                ;Tune number 66 *****************************************                    
  TUNE_066:     DB    12        ;Length of this tune (notes).
                DB    18,36     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,36     ;Note 2
                DB    18,36     ;Note 3
                DB    13,36     ;Note 4
                DB    15,36     ;Note 5
                DB    15,36     ;Note 6
                DB    13,71     ;Note 7
                DB    22,36     ;Note 8
                DB    22,36     ;Note 9
                DB    20,36     ;Note 10
                DB    20,36     ;Note 11
                DB    18,71     ;Note 12

                ;Tune number 67 *****************************************                    
  TUNE_067:     DB    17        ;Length of this tune (notes).
                DB    28,88     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,44     ;Note 2
                DB    49,44     ;Note 3
                DB    28,44     ;Note 4
                DB    27,22     ;Note 5
                DB    25,22     ;Note 6
                DB    23,22     ;Note 7
                DB    20,66     ;Note 8
                DB    49,44     ;Note 9
                DB    16,22     ;Note 10
                DB    16,44     ;Note 11
                DB    20,44     ;Note 12
                DB    20,22     ;Note 13
                DB    16,44     ;Note 14
                DB    18,22     ;Note 15
                DB    18,44     ;Note 16
                DB    16,66     ;Note 17

                ;Tune number 68 *****************************************                    
  TUNE_068:     DB    19        ;Length of this tune (notes).
                DB    13,66     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,44     ;Note 2
                DB    18,44     ;Note 3
                DB    49,44     ;Note 4
                DB    18,44     ;Note 5
                DB    49,44     ;Note 6
                DB    13,44     ;Note 7
                DB    16,44     ;Note 8
                DB    18,44     ;Note 9
                DB    49,88     ;Note 10
                DB    13,44     ;Note 11
                DB    16,44     ;Note 12
                DB    18,44     ;Note 13
                DB    49,44     ;Note 14
                DB    18,44     ;Note 15
                DB    49,44     ;Note 16
                DB    21,44     ;Note 17
                DB    20,44     ;Note 18
                DB    18,44     ;Note 19

                ;Tune number 69 *****************************************                    
  TUNE_069:     DB    16        ;Length of this tune (notes).
                DB    16,17     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,17     ;Note 2
                DB    16,17     ;Note 3
                DB    16,38     ;Note 4
                DB    20,13     ;Note 5
                DB    23,38     ;Note 6
                DB    20,13     ;Note 7
                DB    16,38     ;Note 8
                DB    16,13     ;Note 9
                DB    18,38     ;Note 10
                DB    18,13     ;Note 11
                DB    11,17     ;Note 12
                DB    13,17     ;Note 13
                DB    15,17     ;Note 14
                DB    16,50     ;Note 15
                DB    16,50     ;Note 16

                ;Tune number 70 *****************************************                    
  TUNE_070:     DB    15        ;Length of this tune (notes).
                DB    16,54     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,18     ;Note 2
                DB    18,54     ;Note 3
                DB    20,18     ;Note 4
                DB    16,54     ;Note 5
                DB    20,18     ;Note 6
                DB    18,24     ;Note 7
                DB    20,24     ;Note 8
                DB    21,24     ;Note 9
                DB    23,24     ;Note 10
                DB    20,24     ;Note 11
                DB    16,24     ;Note 12
                DB    18,54     ;Note 13
                DB    20,18     ;Note 14
                DB    16,71     ;Note 15

                ;Tune number 71 *****************************************                    
  TUNE_071:     DB    13        ;Length of this tune (notes).
                DB    20,24     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,47     ;Note 2
                DB    20,24     ;Note 3
                DB    21,24     ;Note 4
                DB    23,47     ;Note 5
                DB    20,24     ;Note 6
                DB    21,24     ;Note 7
                DB    23,24     ;Note 8
                DB    23,24     ;Note 9
                DB    20,24     ;Note 10
                DB    25,24     ;Note 11
                DB    23,47     ;Note 12
                DB    20,47     ;Note 13

                ;Tune number 72 *****************************************                    
  TUNE_072:     DB    23        ;Length of this tune (notes).
                DB    15,14     ;Note 1 val (1-48), dur (10ms inc's).
                DB    17,14     ;Note 2
                DB    20,29     ;Note 3
                DB    17,29     ;Note 4
                DB    18,29     ;Note 5
                DB    15,29     ;Note 6
                DB    17,29     ;Note 7
                DB    13,29     ;Note 8
                DB    15,86     ;Note 9
                DB    27,14     ;Note 10
                DB    29,14     ;Note 11
                DB    32,29     ;Note 12
                DB    29,29     ;Note 13
                DB    30,29     ;Note 14
                DB    27,29     ;Note 15
                DB    29,29     ;Note 16
                DB    25,14     ;Note 17
                DB    27,86     ;Note 18
                DB    13,14     ;Note 19
                DB    13,14     ;Note 20
                DB    13,29     ;Note 21
                DB    15,29     ;Note 22
                DB    13,57     ;Note 23

                ;Tune number 73 *****************************************                    
  TUNE_073:     DB    19        ;Length of this tune (notes).
                DB    20,21     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,63     ;Note 2
                DB    20,21     ;Note 3
                DB    18,63     ;Note 4
                DB    49,42     ;Note 5
                DB    20,63     ;Note 6
                DB    25,42     ;Note 7
                DB    20,42     ;Note 8
                DB    18,84     ;Note 9
                DB    16,21     ;Note 10
                DB    18,21     ;Note 11
                DB    20,42     ;Note 12
                DB    23,42     ;Note 13
                DB    27,63     ;Note 14
                DB    25,21     ;Note 15
                DB    22,21     ;Note 16
                DB    25,63     ;Note 17
                DB    20,84     ;Note 18
                DB    16,84     ;Note 19

                ;Tune number 74 *****************************************                    
  TUNE_074:     DB    16        ;Length of this tune (notes).
                DB    16,49     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,16     ;Note 2
                DB    19,49     ;Note 3
                DB    16,16     ;Note 4
                DB    21,16     ;Note 5
                DB    19,98     ;Note 6
                DB    14,49     ;Note 7
                DB    19,16     ;Note 8
                DB    19,49     ;Note 9
                DB    16,16     ;Note 10
                DB    19,98     ;Note 11
                DB    14,33     ;Note 12
                DB    16,49     ;Note 13
                DB    11,16     ;Note 14
                DB    11,65     ;Note 15
                DB    11,65     ;Note 16

                ;Tune number 75 *****************************************                    
  TUNE_075:     DB    17        ;Length of this tune (notes).
                DB    22,12     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,12     ;Note 2
                DB    22,12     ;Note 3
                DB    27,12     ;Note 4
                DB    22,93     ;Note 5
                DB    18,47     ;Note 6
                DB    20,47     ;Note 7
                DB    15,93     ;Note 8
                DB    49,47     ;Note 9
                DB    22,12     ;Note 10
                DB    27,12     ;Note 11
                DB    22,12     ;Note 12
                DB    27,12     ;Note 13
                DB    22,93     ;Note 14
                DB    18,47     ;Note 15
                DB    20,47     ;Note 16
                DB    25,93     ;Note 17

                ;Tune number 76 *****************************************                    
  TUNE_076:     DB    17        ;Length of this tune (notes).
                DB    4,23      ;Note 1 val (1-48), dur (10ms inc's).
                DB    4,23      ;Note 2
                DB    7,23      ;Note 3
                DB    8,23      ;Note 4
                DB    9,23      ;Note 5
                DB    10,23     ;Note 6
                DB    16,46     ;Note 7
                DB    12,23     ;Note 8
                DB    11,23     ;Note 9
                DB    9,92      ;Note 10
                DB    11,35     ;Note 11
                DB    9,12      ;Note 12
                DB    7,46      ;Note 13
                DB    4,46      ;Note 14
                DB    49,46     ;Note 15
                DB    15,46     ;Note 16
                DB    16,46     ;Note 17

                ;Tune number 77 *****************************************                    
  TUNE_077:     DB    16        ;Length of this tune (notes).
                DB    21,35     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,35     ;Note 2
                DB    21,35     ;Note 3
                DB    25,104    ;Note 4
                DB    26,35     ;Note 5
                DB    25,35     ;Note 6
                DB    21,35     ;Note 7
                DB    23,104    ;Note 8
                DB    25,35     ;Note 9
                DB    26,35     ;Note 10
                DB    23,35     ;Note 11
                DB    25,104    ;Note 12
                DB    23,35     ;Note 13
                DB    21,35     ;Note 14
                DB    19,35     ;Note 15
                DB    18,70     ;Note 16

                ;Tune number 78 *****************************************                    
  TUNE_078:     DB    14        ;Length of this tune (notes).
                DB    16,47     ;Note 1 val (1-48), dur (10ms inc's).
                DB    13,16     ;Note 2
                DB    15,47     ;Note 3
                DB    13,16     ;Note 4
                DB    16,47     ;Note 5
                DB    13,16     ;Note 6
                DB    15,63     ;Note 7
                DB    15,47     ;Note 8
                DB    11,16     ;Note 9
                DB    13,47     ;Note 10
                DB    11,16     ;Note 11
                DB    15,47     ;Note 12
                DB    11,16     ;Note 13
                DB    13,63     ;Note 14

                ;Tune number 79 *****************************************                    
  TUNE_079:     DB    19        ;Length of this tune (notes).
                DB    18,66     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,22     ;Note 2
                DB    22,44     ;Note 3
                DB    22,44     ;Note 4
                DB    20,22     ;Note 5
                DB    18,22     ;Note 6
                DB    20,22     ;Note 7
                DB    22,22     ;Note 8
                DB    18,44     ;Note 9
                DB    13,44     ;Note 10
                DB    18,66     ;Note 11
                DB    20,22     ;Note 12
                DB    22,44     ;Note 13
                DB    22,44     ;Note 14
                DB    20,22     ;Note 15
                DB    18,22     ;Note 16
                DB    20,22     ;Note 17
                DB    22,22     ;Note 18
                DB    18,88     ;Note 19

                ;Tune number 80 *****************************************                    
  TUNE_080:     DB    15        ;Length of this tune (notes).
                DB    21,30     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,30     ;Note 2
                DB    23,30     ;Note 3
                DB    21,30     ;Note 4
                DB    23,90     ;Note 5
                DB    21,30     ;Note 6
                DB    23,30     ;Note 7
                DB    25,30     ;Note 8
                DB    23,60     ;Note 9
                DB    21,60     ;Note 10
                DB    20,30     ;Note 11
                DB    21,30     ;Note 12
                DB    21,30     ;Note 13
                DB    20,30     ;Note 14
                DB    21,60     ;Note 15

                ;Tune number 81 *****************************************                    
  TUNE_081:     DB    17        ;Length of this tune (notes).
                DB    9,38      ;Note 1 val (1-48), dur (10ms inc's).
                DB    13,38     ;Note 2
                DB    16,38     ;Note 3
                DB    49,38     ;Note 4
                DB    9,56      ;Note 5
                DB    13,19     ;Note 6
                DB    16,38     ;Note 7
                DB    49,38     ;Note 8
                DB    9,38      ;Note 9
                DB    13,38     ;Note 10
                DB    16,19     ;Note 11
                DB    16,19     ;Note 12
                DB    18,19     ;Note 13
                DB    16,19     ;Note 14
                DB    19,38     ;Note 15
                DB    16,38     ;Note 16
                DB    13,75     ;Note 17

                ;Tune number 82 *****************************************                    
  TUNE_082:     DB    16        ;Length of this tune (notes).
                DB    25,26     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,78     ;Note 2
                DB    26,26     ;Note 3
                DB    25,26     ;Note 4
                DB    21,26     ;Note 5
                DB    23,78     ;Note 6
                DB    21,26     ;Note 7
                DB    20,26     ;Note 8
                DB    21,26     ;Note 9
                DB    25,39     ;Note 10
                DB    23,13     ;Note 11
                DB    16,78     ;Note 12
                DB    25,26     ;Note 13
                DB    23,26     ;Note 14
                DB    21,26     ;Note 15
                DB    23,52     ;Note 16

                ;Tune number 83 *****************************************                    
  TUNE_083:     DB    18        ;Length of this tune (notes).
                DB    27,35     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,35     ;Note 2
                DB    23,35     ;Note 3
                DB    27,35     ;Note 4
                DB    25,35     ;Note 5
                DB    23,35     ;Note 6
                DB    27,35     ;Note 7
                DB    25,35     ;Note 8
                DB    23,35     ;Note 9
                DB    27,35     ;Note 10
                DB    25,35     ;Note 11
                DB    23,35     ;Note 12
                DB    22,35     ;Note 13
                DB    20,35     ;Note 14
                DB    17,12     ;Note 15
                DB    18,12     ;Note 16
                DB    17,12     ;Note 17
                DB    13,71     ;Note 18

                ;Tune number 84 *****************************************                    
  TUNE_084:     DB    12        ;Length of this tune (notes).
                DB    18,43     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,43     ;Note 2
                DB    22,43     ;Note 3
                DB    25,43     ;Note 4
                DB    30,86     ;Note 5
                DB    27,43     ;Note 6
                DB    49,43     ;Note 7
                DB    23,43     ;Note 8
                DB    23,43     ;Note 9
                DB    25,43     ;Note 10
                DB    27,43     ;Note 11
                DB    25,86     ;Note 12

                ;Tune number 85 *****************************************                    
  TUNE_085:     DB    14        ;Length of this tune (notes).
                DB    18,29     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,29     ;Note 2
                DB    21,57     ;Note 3
                DB    18,29     ;Note 4
                DB    19,29     ;Note 5
                DB    21,57     ;Note 6
                DB    18,29     ;Note 7
                DB    19,29     ;Note 8
                DB    21,29     ;Note 9
                DB    18,29     ;Note 10
                DB    26,100    ;Note 11
                DB    23,14     ;Note 12
                DB    21,29     ;Note 13
                DB    19,114    ;Note 14

                ;Tune number 86 *****************************************                    
  TUNE_086:     DB    17        ;Length of this tune (notes).
                DB    25,29     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,10     ;Note 2
                DB    25,29     ;Note 3
                DB    25,10     ;Note 4
                DB    21,29     ;Note 5
                DB    21,10     ;Note 6
                DB    21,38     ;Note 7
                DB    16,77     ;Note 8
                DB    21,77     ;Note 9
                DB    25,29     ;Note 10
                DB    25,10     ;Note 11
                DB    25,29     ;Note 12
                DB    25,10     ;Note 13
                DB    21,29     ;Note 14
                DB    21,10     ;Note 15
                DB    21,38     ;Note 16
                DB    23,38     ;Note 17

                ;Tune number 87 *****************************************                    
  TUNE_087:     DB    9         ;Length of this tune (notes).
                DB    25,53     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,53     ;Note 2
                DB    16,27     ;Note 3
                DB    21,80     ;Note 4
                DB    25,53     ;Note 5
                DB    21,53     ;Note 6
                DB    23,13     ;Note 7
                DB    23,13     ;Note 8
                DB    23,53     ;Note 9

                ;Tune number 88 *****************************************                    
  TUNE_088:     DB    10        ;Length of this tune (notes).
                DB    21,63     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,63     ;Note 2
                DB    21,31     ;Note 3
                DB    23,94     ;Note 4
                DB    20,31     ;Note 5
                DB    20,31     ;Note 6
                DB    16,31     ;Note 7
                DB    25,63     ;Note 8
                DB    23,31     ;Note 9
                DB    21,63     ;Note 10

                ;Tune number 89 *****************************************                    
  TUNE_089:     DB    16        ;Length of this tune (notes).
                DB    20,46     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,34     ;Note 2
                DB    18,11     ;Note 3
                DB    17,34     ;Note 4
                DB    18,11     ;Note 5
                DB    20,11     ;Note 6
                DB    25,34     ;Note 7
                DB    49,11     ;Note 8
                DB    27,23     ;Note 9
                DB    29,15     ;Note 10
                DB    29,15     ;Note 11
                DB    29,15     ;Note 12
                DB    27,15     ;Note 13
                DB    29,15     ;Note 14
                DB    27,15     ;Note 15
                DB    25,91     ;Note 16

                ;Tune number 90 *****************************************                    
  TUNE_090:     DB    22        ;Length of this tune (notes).
                DB    20,59     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,20     ;Note 2
                DB    23,59     ;Note 3
                DB    20,20     ;Note 4
                DB    16,59     ;Note 5
                DB    18,20     ;Note 6
                DB    20,20     ;Note 7
                DB    16,137    ;Note 8
                DB    18,39     ;Note 9
                DB    20,59     ;Note 10
                DB    16,20     ;Note 11
                DB    13,59     ;Note 12
                DB    15,20     ;Note 13
                DB    16,20     ;Note 14
                DB    13,118    ;Note 15
                DB    13,20     ;Note 16
                DB    13,20     ;Note 17
                DB    11,39     ;Note 18
                DB    11,78     ;Note 19
                DB    11,20     ;Note 20
                DB    11,20     ;Note 21
                DB    11,78     ;Note 22

                ;Tune number 91 *****************************************                    
  TUNE_091:     DB    10        ;Length of this tune (notes).
                DB    16,93     ;Note 1 val (1-48), dur (10ms inc's).
                DB    28,93     ;Note 2
                DB    27,47     ;Note 3
                DB    23,23     ;Note 4
                DB    25,23     ;Note 5
                DB    27,47     ;Note 6
                DB    28,47     ;Note 7
                DB    16,93     ;Note 8
                DB    25,93     ;Note 9
                DB    23,140    ;Note 10

                ;Tune number 92 *****************************************                    
  TUNE_092:     DB    17        ;Length of this tune (notes).
                DB    23,59     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,30     ;Note 2
                DB    15,30     ;Note 3
                DB    13,30     ;Note 4
                DB    11,59     ;Note 5
                DB    13,30     ;Note 6
                DB    15,59     ;Note 7
                DB    18,59     ;Note 8
                DB    23,30     ;Note 9
                DB    22,59     ;Note 10
                DB    23,30     ;Note 11
                DB    25,59     ;Note 12
                DB    13,30     ;Note 13
                DB    15,30     ;Note 14
                DB    17,59     ;Note 15
                DB    20,59     ;Note 16
                DB    25,89     ;Note 17

                ;Tune number 93 *****************************************                    
  TUNE_093:     DB    14        ;Length of this tune (notes).
                DB    23,53     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,80     ;Note 2
                DB    20,27     ;Note 3
                DB    20,53     ;Note 4
                DB    23,53     ;Note 5
                DB    23,80     ;Note 6
                DB    18,27     ;Note 7
                DB    18,53     ;Note 8
                DB    20,53     ;Note 9
                DB    21,53     ;Note 10
                DB    23,53     ;Note 11
                DB    25,53     ;Note 12
                DB    27,53     ;Note 13
                DB    23,107    ;Note 14

                ;Tune number 94 *****************************************                    
  TUNE_094:     DB    10        ;Length of this tune (notes).
                DB    18,44     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,15     ;Note 2
                DB    27,44     ;Note 3
                DB    27,15     ;Note 4
                DB    25,44     ;Note 5
                DB    25,15     ;Note 6
                DB    27,58     ;Note 7
                DB    22,29     ;Note 8
                DB    20,29     ;Note 9
                DB    18,58     ;Note 10

                ;Tune number 95 *****************************************                    
  TUNE_095:     DB    13        ;Length of this tune (notes).
                DB    18,67     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,22     ;Note 2
                DB    23,22     ;Note 3
                DB    23,22     ;Note 4
                DB    30,33     ;Note 5
                DB    27,100    ;Note 6
                DB    27,33     ;Note 7
                DB    27,67     ;Note 8
                DB    27,33     ;Note 9
                DB    28,67     ;Note 10
                DB    27,33     ;Note 11
                DB    23,33     ;Note 12
                DB    25,67     ;Note 13

                ;Tune number 96 *****************************************                    
  TUNE_096:     DB    13        ;Length of this tune (notes).
                DB    25,36     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,36     ;Note 2
                DB    25,36     ;Note 3
                DB    29,36     ;Note 4
                DB    27,36     ;Note 5
                DB    25,36     ;Note 6
                DB    24,36     ;Note 7
                DB    23,71     ;Note 8
                DB    23,36     ;Note 9
                DB    22,36     ;Note 10
                DB    23,36     ;Note 11
                DB    25,36     ;Note 12
                DB    27,36     ;Note 13
                          
                ;Tune number 97 *****************************************                    
  TUNE_097:     DB    13        ;Length of this tune (notes).
                DB    17,25     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,25     ;Note 2
                DB    19,25     ;Note 3
                DB    20,150    ;Note 4
                DB    20,25     ;Note 5
                DB    22,25     ;Note 6
                DB    24,25     ;Note 7
                DB    25,25     ;Note 8
                DB    27,25     ;Note 9
                DB    25,25     ;Note 10
                DB    17,25     ;Note 11
                DB    22,25     ;Note 12
                DB    20,175    ;Note 13

                ;Tune number 98 *****************************************                    
  TUNE_098:     DB    14        ;Length of this tune (notes).
                DB    16,38     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,38     ;Note 2
                DB    20,58     ;Note 3
                DB    21,19     ;Note 4
                DB    20,19     ;Note 5
                DB    20,19     ;Note 6
                DB    16,58     ;Note 7
                DB    21,19     ;Note 8
                DB    20,19     ;Note 9
                DB    20,19     ;Note 10
                DB    16,58     ;Note 11
                DB    21,38     ;Note 12
                DB    20,38     ;Note 13
                DB    18,58     ;Note 14

                ;Tune number 99 *****************************************                    
  TUNE_099:     DB    16        ;Length of this tune (notes).
                DB    15,19     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,19     ;Note 2
                DB    49,38     ;Note 3
                DB    20,75     ;Note 4
                DB    20,19     ;Note 5
                DB    22,19     ;Note 6
                DB    49,38     ;Note 7
                DB    15,75     ;Note 8
                DB    15,19     ;Note 9
                DB    18,19     ;Note 10
                DB    49,38     ;Note 11
                DB    20,75     ;Note 12
                DB    20,19     ;Note 13
                DB    22,19     ;Note 14
                DB    49,38     ;Note 15
                DB    15,75     ;Note 16

                ;Tune number 100 *****************************************                    
  TUNE_100:     DB    25        ;Length of this tune (notes).
                DB    27,50     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,25     ;Note 2
                DB    25,25     ;Note 3
                DB    27,25     ;Note 4
                DB    23,25     ;Note 5
                DB    25,25     ;Note 6
                DB    27,25     ;Note 7
                DB    28,50     ;Note 8
                DB    25,25     ;Note 9
                DB    27,25     ;Note 10
                DB    28,25     ;Note 11
                DB    25,25     ;Note 12
                DB    27,25     ;Note 13
                DB    28,25     ;Note 14
                DB    30,50     ;Note 15
                DB    32,50     ;Note 16
                DB    27,25     ;Note 17
                DB    28,25     ;Note 18
                DB    23,25     ;Note 19
                DB    25,25     ;Note 20
                DB    27,25     ;Note 21
                DB    30,25     ;Note 22
                DB    27,38     ;Note 23
                DB    25,13     ;Note 24
                DB    23,100    ;Note 25

                ;Tune number 101 *****************************************                    
  TUNE_101:     DB    16        ;Length of this tune (notes).
                DB    20,49     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,49     ;Note 2
                DB    16,49     ;Note 3
                DB    15,24     ;Note 4
                DB    13,61     ;Note 5
                DB    18,24     ;Note 6
                DB    18,24     ;Note 7
                DB    20,24     ;Note 8
                DB    21,24     ;Note 9
                DB    21,24     ;Note 10
                DB    20,24     ;Note 11
                DB    18,24     ;Note 12
                DB    18,24     ;Note 13
                DB    16,49     ;Note 14
                DB    16,49     ;Note 15
                DB    20,73     ;Note 16

                ;Tune number 102 *****************************************                    
  TUNE_102:     DB    13        ;Length of this tune (notes).
                DB    28,86     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,64     ;Note 2
                DB    23,21     ;Note 3
                DB    25,43     ;Note 4
                DB    23,43     ;Note 5
                DB    21,43     ;Note 6
                DB    18,43     ;Note 7
                DB    16,43     ;Note 8
                DB    15,21     ;Note 9
                DB    16,21     ;Note 10
                DB    20,64     ;Note 11
                DB    18,21     ;Note 12
                DB    16,86     ;Note 13

                ;Tune number 103 *****************************************                    
  TUNE_103:     DB    16        ;Length of this tune (notes).
                DB    16,60     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,60     ;Note 2
                DB    21,20     ;Note 3
                DB    20,20     ;Note 4
                DB    18,20     ;Note 5
                DB    28,60     ;Note 6
                DB    23,60     ;Note 7
                DB    21,20     ;Note 8
                DB    20,20     ;Note 9
                DB    18,20     ;Note 10
                DB    28,60     ;Note 11
                DB    23,60     ;Note 12
                DB    21,20     ;Note 13
                DB    20,20     ;Note 14
                DB    21,20     ;Note 15
                DB    18,60     ;Note 16

                ;Tune number 104 *****************************************                    
  TUNE_104:     DB    10        ;Length of this tune (notes).
                DB    27,27     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,80     ;Note 2
                DB    22,27     ;Note 3
                DB    23,27     ;Note 4
                DB    22,27     ;Note 5
                DB    20,27     ;Note 6
                DB    18,27     ;Note 7
                DB    22,53     ;Note 8
                DB    17,53     ;Note 9
                DB    18,53     ;Note 10

                ;Tune number 105 *****************************************                    
  TUNE_105:     DB     7        ;Length of this tune (notes).
                DB    30,43     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,14     ;Note 2
                DB    23,43     ;Note 3
                DB    18,14     ;Note 4
                DB    23,14     ;Note 5
                DB    27,14     ;Note 6
                DB    30,57     ;Note 7

                ;Tune number 106 *****************************************                    
  TUNE_106:     DB    10        ;Length of this tune (notes).
                DB    18,27     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,27     ;Note 2
                DB    25,27     ;Note 3
                DB    27,53     ;Note 4
                DB    27,80     ;Note 5
                DB    27,27     ;Note 6
                DB    26,27     ;Note 7
                DB    27,27     ;Note 8
                DB    23,53     ;Note 9
                DB    23,53     ;Note 10

                ;Tune number 107 *****************************************                    
  TUNE_107:     DB    19        ;Length of this tune (notes).
                DB    18,28     ;Note 1 val (1-48), dur (10ms inc's).
                DB    19,28     ;Note 2
                DB    20,28     ;Note 3
                DB    28,55     ;Note 4
                DB    20,28     ;Note 5
                DB    28,55     ;Note 6
                DB    20,28     ;Note 7
                DB    28,110    ;Note 8
                DB    49,28     ;Note 9
                DB    28,28     ;Note 10
                DB    30,28     ;Note 11
                DB    31,28     ;Note 12
                DB    32,28     ;Note 13
                DB    28,28     ;Note 14
                DB    30,28     ;Note 15
                DB    32,55     ;Note 16
                DB    28,28     ;Note 17
                DB    30,55     ;Note 18
                DB    28,110    ;Note 19

                ;Tune number 108 *****************************************                    
  TUNE_108:     DB    12        ;Length of this tune (notes).
                DB    28,25     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,25     ;Note 2
                DB    21,50     ;Note 3
                DB    25,50     ;Note 4
                DB    28,50     ;Note 5
                DB    33,100    ;Note 6
                DB    37,25     ;Note 7
                DB    35,25     ;Note 8
                DB    33,50     ;Note 9
                DB    25,50     ;Note 10
                DB    27,50     ;Note 11
                DB    28,100    ;Note 12

                ;Tune number 109 *****************************************                    
  TUNE_109:     DB    12        ;Length of this tune (notes).
                DB    25,86     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,64     ;Note 2
                DB    24,21     ;Note 3
                DB    25,21     ;Note 4
                DB    27,150    ;Note 5
                DB    25,21     ;Note 6
                DB    27,43     ;Note 7
                DB    25,21     ;Note 8
                DB    27,21     ;Note 9
                DB    25,43     ;Note 10
                DB    24,21     ;Note 11
                DB    23,86     ;Note 12

                ;Tune number 110 *****************************************                    
  TUNE_110:     DB    11        ;Length of this tune (notes).
                DB    21,27     ;Note 1 val (1-48), dur (10ms inc's).
                DB    28,27     ;Note 2
                DB    28,27     ;Note 3
                DB    23,27     ;Note 4
                DB    21,27     ;Note 5
                DB    21,27     ;Note 6
                DB    16,27     ;Note 7
                DB    21,27     ;Note 8
                DB    21,27     ;Note 9
                DB    23,27     ;Note 10
                DB    21,133    ;Note 11

                ;Tune number 111 *****************************************                    
  TUNE_111:     DB    13        ;Length of this tune (notes).
                DB    18,25     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,25     ;Note 2
                DB    21,25     ;Note 3
                DB    23,25     ;Note 4
                DB    25,25     ;Note 5
                DB    24,25     ;Note 6
                DB    25,50     ;Note 7
                DB    24,25     ;Note 8
                DB    20,25     ;Note 9
                DB    24,50     ;Note 10
                DB    23,25     ;Note 11
                DB    19,25     ;Note 12
                DB    23,50     ;Note 13

                ;Tune number 112 *****************************************                    
  TUNE_112:     DB    10        ;Length of this tune (notes).
                DB    18,95     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,32     ;Note 2
                DB    21,32     ;Note 3
                DB    18,32     ;Note 4
                DB    49,63     ;Note 5
                DB    18,95     ;Note 6
                DB    20,32     ;Note 7
                DB    21,63     ;Note 8
                DB    18,63     ;Note 9
                DB    24,95     ;Note 10

                ;Tune number 113 *****************************************                    
  TUNE_113:     DB    8         ;Length of this tune (notes).
                DB    16,43     ;Note 1 val (1-48), dur (10ms inc's).
                DB    12,43     ;Note 2
                DB    12,43     ;Note 3
                DB    19,21     ;Note 4
                DB    19,21     ;Note 5
                DB    16,43     ;Note 6
                DB    12,43     ;Note 7
                DB    12,43     ;Note 8

                ;Tune number 114 *****************************************                    
  TUNE_114:     DB    9         ;Length of this tune (notes).
                DB    21,20     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,20     ;Note 2
                DB    23,20     ;Note 3
                DB    25,40     ;Note 4
                DB    28,60     ;Note 5
                DB    25,20     ;Note 6
                DB    25,20     ;Note 7
                DB    21,20     ;Note 8
                DB    23,80     ;Note 9

                ;Tune number 115 *****************************************                    
  TUNE_115:     DB    13        ;Length of this tune (notes).
                DB    23,31     ;Note 1 val (1-48), dur (10ms inc's).
                DB    20,31     ;Note 2
                DB    23,63     ;Note 3
                DB    23,31     ;Note 4
                DB    20,31     ;Note 5
                DB    23,63     ;Note 6
                DB    25,31     ;Note 7
                DB    23,31     ;Note 8
                DB    21,31     ;Note 9
                DB    20,31     ;Note 10
                DB    18,31     ;Note 11
                DB    20,31     ;Note 12
                DB    21,63     ;Note 13

                ;Tune number 116 *****************************************                    
  TUNE_116:     DB    11        ;Length of this tune (notes).
                DB    23,54     ;Note 1 val (1-48), dur (10ms inc's).
                DB    25,54     ;Note 2
                DB    23,27     ;Note 3
                DB    21,27     ;Note 4
                DB    20,162    ;Note 5
                DB    18,27     ;Note 6
                DB    16,27     ;Note 7
                DB    15,54     ;Note 8
                DB    13,54     ;Note 9
                DB    15,108    ;Note 10
                DB    15,108    ;Note 11

                ;Tune number 117 *****************************************                    
  TUNE_117:     DB    13        ;Length of this tune (notes).
                DB    20,75     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,38     ;Note 2
                DB    30,113    ;Note 3
                DB    28,75     ;Note 4
                DB    23,38     ;Note 5
                DB    21,113    ;Note 6
                DB    20,75     ;Note 7
                DB    20,38     ;Note 8
                DB    20,38     ;Note 9
                DB    21,38     ;Note 10
                DB    23,38     ;Note 11
                DB    25,113    ;Note 12
                DB    23,113    ;Note 13

                ;Tune number 118 *****************************************                    
  TUNE_118:     DB    11        ;Length of this tune (notes).
                DB    23,33     ;Note 1 val (1-48), dur (10ms inc's).
                DB    18,33     ;Note 2
                DB    18,33     ;Note 3
                DB    25,33     ;Note 4
                DB    23,33     ;Note 5
                DB    23,33     ;Note 6
                DB    18,33     ;Note 7
                DB    23,33     ;Note 8
                DB    23,33     ;Note 9
                DB    25,33     ;Note 10
                DB    23,67     ;Note 11

                ;Tune number 119 *****************************************                    
  TUNE_119:     DB    22        ;Length of this tune (notes).
                DB    23,23     ;Note 1 val (1-48), dur (10ms inc's).
                DB    21,23     ;Note 2
                DB    23,23     ;Note 3
                DB    21,23     ;Note 4
                DB    20,46     ;Note 5
                DB    16,46     ;Note 6
                DB    49,46     ;Note 7
                DB    20,23     ;Note 8
                DB    21,23     ;Note 9
                DB    23,23     ;Note 10
                DB    21,23     ;Note 11
                DB    23,23     ;Note 12
                DB    21,23     ;Note 13
                DB    20,23     ;Note 14
                DB    21,23     ;Note 15
                DB    23,23     ;Note 16
                DB    25,23     ;Note 17
                DB    26,23     ;Note 18
                DB    25,23     ;Note 19
                DB    26,23     ;Note 20
                DB    25,23     ;Note 21
                DB    23,46     ;Note 22

                ;Tune number 120 *****************************************                    
  TUNE_120:     DB    14        ;Length of this tune (notes).
                DB    23,50     ;Note 1 val (1-48), dur (10ms inc's).
                DB    27,25     ;Note 2
                DB    28,75     ;Note 3
                DB    28,25     ;Note 4
                DB    27,25     ;Note 5
                DB    23,25     ;Note 6
                DB    28,75     ;Note 7
                DB    27,25     ;Note 8
                DB    23,25     ;Note 9
                DB    30,38     ;Note 10
                DB    27,13     ;Note 11
                DB    30,38     ;Note 12
                DB    27,13     ;Note 13
                DB    23,50     ;Note 14

                ;Tune number 121 *****************************************                    
  TUNE_121:     DB    22        ;Length of this tune (notes).
                DB    11,21     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,42     ;Note 2
                DB    16,21     ;Note 3
                DB    18,21     ;Note 4
                DB    20,21     ;Note 5
                DB    21,21     ;Note 6
                DB    18,21     ;Note 7
                DB    16,21     ;Note 8
                DB    18,42     ;Note 9
                DB    18,42     ;Note 10
                DB    11,85     ;Note 11
                DB    15,42     ;Note 12
                DB    15,21     ;Note 13
                DB    16,21     ;Note 14
                DB    18,42     ;Note 15
                DB    16,21     ;Note 16
                DB    15,21     ;Note 17
                DB    16,21     ;Note 18
                DB    18,21     ;Note 19
                DB    16,21     ;Note 20
                DB    13,21     ;Note 21
                DB    11,85     ;Note 22

                ;Tune number 122 *****************************************                    
  TUNE_122:     DB    16        ;Length of this tune (notes).
                DB    11,47     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,70     ;Note 2
                DB    20,23     ;Note 3
                DB    20,47     ;Note 4
                DB    22,23     ;Note 5
                DB    23,23     ;Note 6
                DB    22,70     ;Note 7
                DB    18,23     ;Note 8
                DB    18,47     ;Note 9
                DB    11,47     ;Note 10
                DB    23,70     ;Note 11
                DB    20,23     ;Note 12
                DB    20,47     ;Note 13
                DB    22,23     ;Note 14
                DB    23,23     ;Note 15
                DB    22,93     ;Note 16

                ;Tune number 123 *****************************************                    
  TUNE_123:     DB    14        ;Length of this tune (notes).
                DB    13,63     ;Note 1 val (1-48), dur (10ms inc's).
                DB    13,47     ;Note 2
                DB    15,16     ;Note 3
                DB    17,31     ;Note 4
                DB    13,31     ;Note 5
                DB    17,31     ;Note 6
                DB    20,31     ;Note 7
                DB    25,63     ;Note 8
                DB    25,47     ;Note 9
                DB    24,16     ;Note 10
                DB    25,31     ;Note 11
                DB    20,31     ;Note 12
                DB    17,31     ;Note 13
                DB    13,31     ;Note 14

                ;Tune number 124 *****************************************                    
  TUNE_124:     DB    12        ;Length of this tune (notes).
                DB    16,32     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,32     ;Note 2
                DB    18,32     ;Note 3
                DB    20,95     ;Note 4
                DB    18,32     ;Note 5
                DB    20,32     ;Note 6
                DB    25,32     ;Note 7
                DB    23,95     ;Note 8
                DB    20,32     ;Note 9
                DB    18,32     ;Note 10
                DB    16,32     ;Note 11
                DB    13,126    ;Note 12

                ;Tune number 125 *****************************************                    
  TUNE_125:     DB    14        ;Length of this tune (notes).
                DB    18,69     ;Note 1 val (1-48), dur (10ms inc's).
                DB    49,23     ;Note 2
                DB    13,69     ;Note 3
                DB    49,23     ;Note 4
                DB    18,23     ;Note 5
                DB    18,46     ;Note 6
                DB    20,23     ;Note 7
                DB    22,46     ;Note 8
                DB    25,46     ;Note 9
                DB    22,46     ;Note 10
                DB    22,46     ;Note 11
                DB    20,23     ;Note 12
                DB    20,46     ;Note 13
                DB    18,69     ;Note 14

                ;Tune number 126 *****************************************                    
  TUNE_126:     DB    10        ;Length of this tune (notes).
                DB    11,23     ;Note 1 val (1-48), dur (10ms inc's).
                DB    16,23     ;Note 2
                DB    18,23     ;Note 3
                DB    20,23     ;Note 4
                DB    18,23     ;Note 5
                DB    16,23     ;Note 6
                DB    18,23     ;Note 7
                DB    20,46     ;Note 8
                DB    16,46     ;Note 9
                DB    16,46     ;Note 10

                ;Tune number 127 *****************************************                    
  TUNE_127:     DB    17        ;Length of this tune (notes).
                DB    20,41     ;Note 1 val (1-48), dur (10ms inc's).
                DB    29,41     ;Note 2
                DB    27,41     ;Note 3
                DB    25,41     ;Note 4
                DB    27,41     ;Note 5
                DB    25,41     ;Note 6
                DB    22,41     ;Note 7
                DB    20,41     ;Note 8
                DB    17,164    ;Note 9
                DB    20,41     ;Note 10
                DB    29,41     ;Note 11
                DB    27,41     ;Note 12
                DB    25,41     ;Note 13
                DB    25,41     ;Note 14
                DB    24,41     ;Note 15
                DB    25,41     ;Note 16
                DB    27,123    ;Note 17

                ;Tune number 128 *****************************************                    
  TUNE_128:     DB    10        ;Length of this tune (notes).
                DB    32,55     ;Note 1 val (1-48), dur (10ms inc's).
                DB    32,109    ;Note 2
                DB    31,27     ;Note 3
                DB    31,27     ;Note 4
                DB    31,55     ;Note 5
                DB    31,27     ;Note 6
                DB    31,55     ;Note 7
                DB    29,55     ;Note 8
                DB    27,82     ;Note 9
                DB    20,109    ;Note 10

                ;Tune number 129 *****************************************                    
  TUNE_129:     DB    14        ;Length of this tune (notes).
                DB    23,47     ;Note 1 val (1-48), dur (10ms inc's).
                DB    23,47     ;Note 2
                DB    23,23     ;Note 3
                DB    25,47     ;Note 4
                DB    20,47     ;Note 5
                DB    18,47     ;Note 6
                DB    16,117    ;Note 7
                DB    28,47     ;Note 8
                DB    28,47     ;Note 9
                DB    28,47     ;Note 10
                DB    30,47     ;Note 11
                DB    25,23     ;Note 12
                DB    23,47     ;Note 13
                DB    21,70     ;Note 14

                ;Tune number 130 *****************************************                    
  TUNE_130:     DB    11        ;Length of this tune (notes).
                DB    15,38     ;Note 1 val (1-48), dur (10ms inc's).
                DB    17,75     ;Note 2
                DB    20,38     ;Note 3
                DB    24,56     ;Note 4
                DB    23,19     ;Note 5
                DB    24,38     ;Note 6
                DB    27,38     ;Note 7
                DB    15,38     ;Note 8
                DB    17,75     ;Note 9
                DB    20,38     ;Note 10
                DB    24,150    ;Note 11

                ;Tune number 131 *****************************************
  TUNE_131:     DB    10        ;Length of this tune(notes).
                DB    23,67     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,67     ;Note 2
                DB    23,33     ;Note 3
                DB    23,100    ;Note 4
                DB    23,33     ;Note 5
                DB    25,33     ;Note 6
                DB    25,33     ;Note 7
                DB    23,33     ;Note 8
                DB    22,33     ;Note 9
                DB    23,67     ;Note 10

                ;Tune number 132 *****************************************
  TUNE_132:     DB    15        ;Length of this tune(notes).
                DB    23,35     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,18     ;Note 2
                DB    23,18     ;Note 3
                DB    21,35     ;Note 4
                DB    20,35     ;Note 5
                DB    21,35     ;Note 6
                DB    23,106    ;Note 7
                DB    49,35     ;Note 8
                DB    23,35     ;Note 9
                DB    25,18     ;Note 10
                DB    23,18     ;Note 11
                DB    21,35     ;Note 12
                DB    20,35     ;Note 13
                DB    21,35     ;Note 14
                DB    23,106    ;Note 15

                ;Tune number 133 *****************************************
  TUNE_133:     DB    14        ;Length of this tune(notes).
                DB    16,38     ;Note 1 val(1-48),dur(10ms inc's).
                DB    14,38     ;Note 2
                DB    15,75     ;Note 3
                DB    15,38     ;Note 4
                DB    13,38     ;Note 5
                DB    15,38     ;Note 6
                DB    21,38     ;Note 7
                DB    20,38     ;Note 8
                DB    19,38     ;Note 9
                DB    20,19     ;Note 10
                DB    20,19     ;Note 11
                DB    20,75     ;Note 12
                DB    11,38     ;Note 13
                DB    20,75     ;Note 14

                ;Tune number 134 *****************************************
  TUNE_134:     DB    13        ;Length of this tune(notes).
                DB    23,18     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,18     ;Note 2
                DB    23,107    ;Note 3
                DB    16,54     ;Note 4
                DB    16,18     ;Note 5
                DB    20,36     ;Note 6
                DB    23,36     ;Note 7
                DB    18,18     ;Note 8
                DB    18,18     ;Note 9
                DB    18,36     ;Note 10
                DB    15,18     ;Note 11
                DB    18,54     ;Note 12
                DB    11,71     ;Note 13

                ;Tune number 135 *****************************************
  TUNE_135:     DB    11        ;Length of this tune(notes).
                DB    30,36     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,36     ;Note 2
                DB    27,36     ;Note 3
                DB    22,36     ;Note 4
                DB    25,36     ;Note 5
                DB    18,36     ;Note 6
                DB    20,36     ;Note 7
                DB    18,36     ;Note 8
                DB    17,71     ;Note 9
                DB    27,71     ;Note 10
                DB    25,71     ;Note 11

                ;Tune number 136 *****************************************
  TUNE_136:     DB    12        ;Length of this tune(notes).
                DB    17,56     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,56     ;Note 2
                DB    24,56     ;Note 3
                DB    22,28     ;Note 4
                DB    24,28     ;Note 5
                DB    17,28     ;Note 6
                DB    17,28     ;Note 7
                DB    15,28     ;Note 8
                DB    17,28     ;Note 9
                DB    19,56     ;Note 10
                DB    15,56     ;Note 11
                DB    12,56     ;Note 12

                ;Tune number 137 *****************************************
  TUNE_137:     DB    11        ;Length of this tune(notes).
                DB    15,100    ;Note 1 val(1-48),dur(10ms inc's).
                DB    22,50     ;Note 2
                DB    22,100    ;Note 3
                DB    22,25     ;Note 4
                DB    24,25     ;Note 5
                DB    25,100    ;Note 6
                DB    27,50     ;Note 7
                DB    24,25     ;Note 8
                DB    20,75     ;Note 9
                DB    22,50     ;Note 10
                DB    15,100    ;Note 11

                ;Tune number 138 *****************************************
  TUNE_138:     DB    8         ;Length of this tune(notes).
                DB    11,31     ;Note 1 val(1-48),dur(10ms inc's).
                DB    15,31     ;Note 2
                DB    18,31     ;Note 3
                DB    20,62     ;Note 4
                DB    18,31     ;Note 5
                DB    20,92     ;Note 6
                DB    11,62     ;Note 7
                DB    11,62     ;Note 8

                ;Tune number 139 *****************************************
  TUNE_139:     DB    17        ;Length of this tune(notes).
                DB    23,19     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,19     ;Note 2
                DB    23,19     ;Note 3
                DB    18,19     ;Note 4
                DB    18,19     ;Note 5
                DB    18,19     ;Note 6
                DB    23,19     ;Note 7
                DB    23,19     ;Note 8
                DB    23,19     ;Note 9
                DB    23,57     ;Note 10
                DB    25,19     ;Note 11
                DB    25,19     ;Note 12
                DB    25,19     ;Note 13
                DB    20,19     ;Note 14
                DB    20,19     ;Note 15
                DB    20,19     ;Note 16
                DB    25,57     ;Note 17

                ;Tune number 140 *****************************************
  TUNE_140:     DB    9         ;Length of this tune(notes).
                DB    18,55     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,55     ;Note 2
                DB    27,109    ;Note 3
                DB    18,55     ;Note 4
                DB    18,55     ;Note 5
                DB    27,55     ;Note 6
                DB    25,55     ;Note 7
                DB    22,55     ;Note 8
                DB    19,109    ;Note 9

                ;Tune number 141 *****************************************
  TUNE_141:     DB    6         ;Length of this tune(notes).
                DB    30,33     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,100    ;Note 2
                DB    26,33     ;Note 3
                DB    30,33     ;Note 4
                DB    31,33     ;Note 5
                DB    28,67     ;Note 6

                ;Tune number 142 *****************************************
  TUNE_142:     DB    14        ;Length of this tune(notes).
                DB    9,54      ;Note 1 val(1-48),dur(10ms inc's).
                DB    9,54      ;Note 2
                DB    11,27     ;Note 3
                DB    14,81     ;Note 4
                DB    11,27     ;Note 5
                DB    9,54      ;Note 6
                DB    9,54      ;Note 7
                DB    11,27     ;Note 8
                DB    14,81     ;Note 9
                DB    16,27     ;Note 10
                DB    18,54     ;Note 11
                DB    18,54     ;Note 12
                DB    16,27     ;Note 13
                DB    14,81     ;Note 14

                ;Tune number 143 *****************************************
  TUNE_143:     DB    14        ;Length of this tune(notes).
                DB    16,44     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,44     ;Note 2
                DB    23,44     ;Note 3
                DB    21,44     ;Note 4
                DB    20,44     ;Note 5
                DB    21,44     ;Note 6
                DB    26,88     ;Note 7
                DB    20,44     ;Note 8
                DB    26,44     ;Note 9
                DB    25,44     ;Note 10
                DB    23,44     ;Note 11
                DB    22,44     ;Note 12
                DB    23,44     ;Note 13
                DB    28,88     ;Note 14

                ;Tune number 144 *****************************************
  TUNE_144:     DB    15        ;Length of this tune(notes).
                DB    18,41     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,14     ;Note 2
                DB    18,55     ;Note 3
                DB    13,55     ;Note 4
                DB    22,41     ;Note 5
                DB    22,14     ;Note 6
                DB    22,55     ;Note 7
                DB    18,55     ;Note 8
                DB    18,41     ;Note 9
                DB    22,14     ;Note 10
                DB    25,82     ;Note 11
                DB    25,27     ;Note 12
                DB    23,27     ;Note 13
                DB    22,27     ;Note 14
                DB    20,55     ;Note 15

                ;Tune number 145 *****************************************
  TUNE_145:     DB    9         ;Length of this tune(notes).
                DB    10,53     ;Note 1 val(1-48),dur(10ms inc's).
                DB    11,53     ;Note 2
                DB    13,53     ;Note 3
                DB    10,27     ;Note 4
                DB    13,27     ;Note 5
                DB    18,27     ;Note 6
                DB    49,27     ;Note 7
                DB    18,27     ;Note 8
                DB    15,107    ;Note 9

                ;Tune number 146 *****************************************
  TUNE_146:     DB    11        ;Length of this tune(notes).
                DB    4,38      ;Note 1 val(1-48),dur(10ms inc's).
                DB    9,38      ;Note 2
                DB    11,38     ;Note 3
                DB    13,38     ;Note 4
                DB    11,38     ;Note 5
                DB    9,77      ;Note 6
                DB    4,38      ;Note 7
                DB    9,38      ;Note 8
                DB    13,38     ;Note 9
                DB    13,38     ;Note 10
                DB    11,77     ;Note 11

                ;Tune number 147 *****************************************
  TUNE_147:     DB    14        ;Length of this tune(notes).
                DB    30,48     ;Note 1 val(1-48),dur(10ms inc's).
                DB    26,16     ;Note 2
                DB    22,48     ;Note 3
                DB    19,16     ;Note 4
                DB    17,48     ;Note 5
                DB    19,16     ;Note 6
                DB    22,48     ;Note 7
                DB    26,16     ;Note 8
                DB    28,48     ;Note 9
                DB    26,16     ;Note 10
                DB    29,48     ;Note 11
                DB    26,16     ;Note 12
                DB    31,16     ;Note 13
                DB    26,97     ;Note 14

                ;Tune number 148 *****************************************
  TUNE_148:     DB    14        ;Length of this tune(notes).
                DB    25,44     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,44     ;Note 2
                DB    25,22     ;Note 3
                DB    25,66     ;Note 4
                DB    27,44     ;Note 5
                DB    25,44     ;Note 6
                DB    22,88     ;Note 7
                DB    20,44     ;Note 8
                DB    20,44     ;Note 9
                DB    20,22     ;Note 10
                DB    20,66     ;Note 11
                DB    22,44     ;Note 12
                DB    20,44     ;Note 13
                DB    17,88     ;Note 14

                ;Tune number 149 *****************************************
  TUNE_149:     DB    11        ;Length of this tune(notes).
                DB    16,26     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,53     ;Note 2
                DB    21,53     ;Note 3
                DB    21,53     ;Note 4
                DB    23,53     ;Note 5
                DB    25,53     ;Note 6
                DB    25,26     ;Note 7
                DB    23,26     ;Note 8
                DB    25,26     ;Note 9
                DB    23,26     ;Note 10
                DB    21,105    ;Note 11

                ;Tune number 150 *****************************************
  TUNE_150:     DB    13        ;Length of this tune(notes).
                DB    4,53      ;Note 1 val(1-48),dur(10ms inc's).
                DB    9,26      ;Note 2
                DB    9,26      ;Note 3
                DB    9,26      ;Note 4
                DB    8,26      ;Note 5
                DB    9,26      ;Note 6
                DB    11,79     ;Note 7
                DB    9,26      ;Note 8
                DB    9,26      ;Note 9
                DB    8,26      ;Note 10
                DB    8,26      ;Note 11
                DB    7,26      ;Note 12
                DB    8,105     ;Note 13

                ;Tune number 151 *****************************************
  TUNE_151:     DB    9         ;Length of this tune(notes).
                DB    20,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,25     ;Note 2
                DB    23,25     ;Note 3
                DB    21,75     ;Note 4
                DB    18,25     ;Note 5
                DB    23,25     ;Note 6
                DB    21,75     ;Note 7
                DB    18,25     ;Note 8
                DB    16,100    ;Note 9

                ;Tune number 152 *****************************************
  TUNE_152:     DB    8         ;Length of this tune(notes).
                DB    16,50     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,38     ;Note 2
                DB    21,13     ;Note 3
                DB    21,100    ;Note 4
                DB    16,50     ;Note 5
                DB    23,38     ;Note 6
                DB    16,13     ;Note 7
                DB    21,100    ;Note 8

                ;Tune number 153 *****************************************
  TUNE_153:     DB    14        ;Length of this tune(notes).
                DB    18,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,50     ;Note 2
                DB    18,50     ;Note 3
                DB    21,50     ;Note 4
                DB    18,50     ;Note 5
                DB    16,50     ;Note 6
                DB    18,25     ;Note 7
                DB    21,75     ;Note 8
                DB    21,25     ;Note 9
                DB    23,50     ;Note 10
                DB    25,50     ;Note 11
                DB    23,75     ;Note 12
                DB    25,25     ;Note 13
                DB    23,100    ;Note 14

                ;Tune number 154 *****************************************
  TUNE_154:     DB    14        ;Length of this tune(notes).
                DB    19,47     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,47     ;Note 2
                DB    23,70     ;Note 3
                DB    23,23     ;Note 4
                DB    21,23     ;Note 5
                DB    19,47     ;Note 6
                DB    23,70     ;Note 7
                DB    19,47     ;Note 8
                DB    21,47     ;Note 9
                DB    23,70     ;Note 10
                DB    23,23     ;Note 11
                DB    21,23     ;Note 12
                DB    19,70     ;Note 13
                DB    16,93     ;Note 14

                ;Tune number 155 *****************************************
  TUNE_155:     DB    9         ;Length of this tune(notes).
                DB    28,48     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,48     ;Note 2
                DB    20,95     ;Note 3
                DB    28,24     ;Note 4
                DB    30,71     ;Note 5
                DB    28,24     ;Note 6
                DB    27,48     ;Note 7
                DB    24,48     ;Note 8
                DB    21,95     ;Note 9

                ;Tune number 156 *****************************************
  TUNE_156:     DB    14        ;Length of this tune(notes).
                DB    16,50     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,50     ;Note 2
                DB    23,50     ;Note 3
                DB    25,50     ;Note 4
                DB    25,25     ;Note 5
                DB    23,25     ;Note 6
                DB    25,50     ;Note 7
                DB    25,25     ;Note 8
                DB    23,25     ;Note 9
                DB    25,25     ;Note 10
                DB    23,25     ;Note 11
                DB    21,25     ;Note 12
                DB    23,25     ;Note 13
                DB    25,50     ;Note 14

                ;Tune number 157 *****************************************
  TUNE_157:     DB    16        ;Length of this tune(notes).
                DB    30,100    ;Note 1 val(1-48),dur(10ms inc's).
                DB    30,25     ;Note 2
                DB    25,25     ;Note 3
                DB    22,25     ;Note 4
                DB    18,25     ;Note 5
                DB    17,50     ;Note 6
                DB    27,25     ;Note 7
                DB    25,75     ;Note 8
                DB    29,100    ;Note 9
                DB    29,25     ;Note 10
                DB    25,25     ;Note 11
                DB    23,25     ;Note 12
                DB    20,25     ;Note 13
                DB    18,50     ;Note 14
                DB    30,25     ;Note 15
                DB    30,75     ;Note 16

                ;Tune number 158 *****************************************
  TUNE_158:     DB    12        ;Length of this tune(notes).
                DB    23,46     ;Note 1 val(1-48),dur(10ms inc's).
                DB    28,46     ;Note 2
                DB    49,46     ;Note 3
                DB    20,46     ;Note 4
                DB    23,46     ;Note 5
                DB    49,46     ;Note 6
                DB    18,23     ;Note 7
                DB    20,23     ;Note 8
                DB    21,92     ;Note 9
                DB    21,46     ;Note 10
                DB    20,46     ;Note 11
                DB    18,92     ;Note 12

                ;Tune number 159 *****************************************
  TUNE_159:     DB    11        ;Length of this tune(notes).
                DB    18,38     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,13     ;Note 2
                DB    30,100    ;Note 3
                DB    30,50     ;Note 4
                DB    27,100    ;Note 5
                DB    18,50     ;Note 6
                DB    20,100    ;Note 7
                DB    18,50     ;Note 8
                DB    21,100    ;Note 9
                DB    18,50     ;Note 10
                DB    22,150    ;Note 11

                ;Tune number 160 *****************************************
  TUNE_160:     DB    16        ;Length of this tune(notes).
                DB    21,44     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,44     ;Note 2
                DB    26,44     ;Note 3
                DB    49,44     ;Note 4
                DB    30,66     ;Note 5
                DB    29,22     ;Note 6
                DB    30,44     ;Note 7
                DB    26,44     ;Note 8
                DB    49,44     ;Note 9
                DB    30,44     ;Note 10
                DB    30,44     ;Note 11
                DB    49,44     ;Note 12
                DB    28,66     ;Note 13
                DB    27,22     ;Note 14
                DB    28,44     ;Note 15
                DB    25,44     ;Note 16

                ;Tune number 161 *****************************************
  TUNE_161:     DB    17        ;Length of this tune(notes).
                DB    23,50     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,50     ;Note 2
                DB    20,25     ;Note 3
                DB    18,25     ;Note 4
                DB    23,25     ;Note 5
                DB    23,25     ;Note 6
                DB    23,25     ;Note 7
                DB    18,75     ;Note 8
                DB    49,50     ;Note 9
                DB    27,25     ;Note 10
                DB    25,25     ;Note 11
                DB    23,25     ;Note 12
                DB    20,25     ;Note 13
                DB    23,25     ;Note 14
                DB    23,25     ;Note 15
                DB    23,25     ;Note 16
                DB    27,75     ;Note 17

                ;Tune number 162 *****************************************
  TUNE_162:     DB    17        ;Length of this tune(notes).
                DB    9,23      ;Note 1 val(1-48),dur(10ms inc's).
                DB    11,68     ;Note 2
                DB    14,68     ;Note 3
                DB    14,23     ;Note 4
                DB    16,23     ;Note 5
                DB    14,68     ;Note 6
                DB    10,90     ;Note 7
                DB    9,23      ;Note 8
                DB    11,68     ;Note 9
                DB    14,68     ;Note 10
                DB    18,23     ;Note 11
                DB    21,23     ;Note 12
                DB    21,23     ;Note 13
                DB    20,23     ;Note 14
                DB    20,23     ;Note 15
                DB    19,23     ;Note 16
                DB    18,45     ;Note 17

                ;Tune number 163 *****************************************
  TUNE_163:     DB    12        ;Length of this tune(notes).
                DB    25,31     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,31     ;Note 2
                DB    25,23     ;Note 3
                DB    49,8      ;Note 4
                DB    25,31     ;Note 5
                DB    25,31     ;Note 6
                DB    25,31     ;Note 7
                DB    24,31     ;Note 8
                DB    25,31     ;Note 9
                DB    26,62     ;Note 10
                DB    28,31     ;Note 11
                DB    23,62     ;Note 12

                ;Tune number 164 *****************************************
  TUNE_164:     DB    11        ;Length of this tune(notes).
                DB    9,45      ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,45     ;Note 2
                DB    20,23     ;Note 3
                DB    21,45     ;Note 4
                DB    18,68     ;Note 5
                DB    19,45     ;Note 6
                DB    18,23     ;Note 7
                DB    19,45     ;Note 8
                DB    13,68     ;Note 9
                DB    16,45     ;Note 10
                DB    14,45     ;Note 11

                ;Tune number 165 *****************************************
  TUNE_165:     DB    25        ;Length of this tune(notes).
                DB    25,119    ;Note 1 val(1-48),dur(10ms inc's).
                DB    26,24     ;Note 2
                DB    25,24     ;Note 3
                DB    20,24     ;Note 4
                DB    23,24     ;Note 5
                DB    25,24     ;Note 6
                DB    29,24     ;Note 7
                DB    20,24     ;Note 8
                DB    23,119    ;Note 9
                DB    25,24     ;Note 10
                DB    26,24     ;Note 11
                DB    25,24     ;Note 12
                DB    26,24     ;Note 13
                DB    25,24     ;Note 14
                DB    24,24     ;Note 15
                DB    17,24     ;Note 16
                DB    23,119    ;Note 17
                DB    25,24     ;Note 18
                DB    24,24     ;Note 19
                DB    23,24     ;Note 20
                DB    22,24     ;Note 21
                DB    21,24     ;Note 22
                DB    19,24     ;Note 23
                DB    17,24     ;Note 24
                DB    18,143    ;Note 25

                ;Tune number 166 *****************************************
  TUNE_166:     DB    12        ;Length of this tune(notes).
                DB    16,38     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,19     ;Note 2
                DB    23,19     ;Note 3
                DB    23,38     ;Note 4
                DB    16,38     ;Note 5
                DB    23,19     ;Note 6
                DB    23,19     ;Note 7
                DB    23,38     ;Note 8
                DB    23,38     ;Note 9
                DB    24,57     ;Note 10
                DB    23,19     ;Note 11
                DB    21,57     ;Note 12

                ;Tune number 167 *****************************************
  TUNE_167:     DB    9         ;Length of this tune(notes).
                DB    19,80     ;Note 1 val(1-48),dur(10ms inc's).
                DB    24,27     ;Note 2
                DB    23,27     ;Note 3
                DB    21,27     ;Note 4
                DB    19,60     ;Note 5
                DB    21,20     ;Note 6
                DB    16,60     ;Note 7
                DB    19,20     ;Note 8
                DB    12,80     ;Note 9

                ;Tune number 168 *****************************************
  TUNE_168:     DB    19        ;Length of this tune(notes).
                DB    11,10     ;Note 1 val(1-48),dur(10ms inc's).
                DB    13,38     ;Note 2
                DB    16,38     ;Note 3
                DB    49,38     ;Note 4
                DB    23,38     ;Note 5
                DB    49,38     ;Note 6
                DB    21,10     ;Note 7
                DB    20,38     ;Note 8
                DB    16,38     ;Note 9
                DB    23,38     ;Note 10
                DB    49,38     ;Note 11
                DB    21,10     ;Note 12
                DB    20,38     ;Note 13
                DB    16,38     ;Note 14
                DB    13,38     ;Note 15
                DB    16,38     ;Note 16
                DB    20,38     ;Note 17
                DB    16,19     ;Note 18
                DB    18,19     ;Note 19

                ;Tune number 169 *****************************************
  TUNE_169:     DB    15        ;Length of this tune(notes).
                DB    16,40     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,80     ;Note 2
                DB    21,20     ;Note 3
                DB    20,20     ;Note 4
                DB    18,40     ;Note 5
                DB    18,80     ;Note 6
                DB    23,20     ;Note 7
                DB    21,20     ;Note 8
                DB    20,40     ;Note 9
                DB    20,80     ;Note 10
                DB    25,20     ;Note 11
                DB    23,20     ;Note 12
                DB    21,40     ;Note 13
                DB    25,40     ;Note 14
                DB    16,40     ;Note 15

                ;Tune number 170 *****************************************
  TUNE_170:     DB    8         ;Length of this tune(notes).
                DB    18,75     ;Note 1 val(1-48),dur(10ms inc's).
                DB    15,19     ;Note 2
                DB    16,19     ;Note 3
                DB    18,19     ;Note 4
                DB    18,19     ;Note 5
                DB    27,38     ;Note 6
                DB    27,38     ;Note 7
                DB    23,75     ;Note 8

                ;Tune number 171 *****************************************
  TUNE_171:     DB    11        ;Length of this tune(notes).
                DB    21,67     ;Note 1 val(1-48),dur(10ms inc's).
                DB    22,67     ;Note 2
                DB    24,22     ;Note 3
                DB    24,22     ;Note 4
                DB    24,22     ;Note 5
                DB    23,22     ;Note 6
                DB    23,22     ;Note 7
                DB    24,22     ;Note 8
                DB    26,50     ;Note 9
                DB    24,17     ;Note 10
                DB    22,67     ;Note 11

                ;Tune number 172 *****************************************
  TUNE_172:     DB    12        ;Length of this tune(notes).
                DB    24,60     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,60     ;Note 2
                DB    24,60     ;Note 3
                DB    21,30     ;Note 4
                DB    23,30     ;Note 5
                DB    24,30     ;Note 6
                DB    21,30     ;Note 7
                DB    26,90     ;Note 8
                DB    28,30     ;Note 9
                DB    29,90     ;Note 10
                DB    25,30     ;Note 11
                DB    26,60     ;Note 12

                ;Tune number 173 *****************************************
  TUNE_173:     DB    14        ;Length of this tune(notes).
                DB    18,42     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,14     ;Note 2
                DB    18,28     ;Note 3
                DB    16,28     ;Note 4
                DB    14,28     ;Note 5
                DB    11,83     ;Note 6
                DB    19,56     ;Note 7
                DB    49,56     ;Note 8
                DB    9,28      ;Note 9
                DB    13,28     ;Note 10
                DB    16,28     ;Note 11
                DB    19,28     ;Note 12
                DB    23,28     ;Note 13
                DB    21,28     ;Note 14

                ;Tune number 174 *****************************************
  TUNE_174:     DB    13        ;Length of this tune(notes).
                DB    16,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,50     ;Note 2
                DB    16,25     ;Note 3
                DB    18,50     ;Note 4
                DB    21,50     ;Note 5
                DB    25,50     ;Note 6
                DB    25,100    ;Note 7
                DB    16,25     ;Note 8
                DB    16,50     ;Note 9
                DB    16,25     ;Note 10
                DB    18,50     ;Note 11
                DB    21,50     ;Note 12
                DB    23,50     ;Note 13

                ;Tune number 175 *****************************************
  TUNE_175:     DB    15        ;Length of this tune(notes).
                DB    6,57      ;Note 1 val(1-48),dur(10ms inc's).
                DB    11,29     ;Note 2
                DB    11,29     ;Note 3
                DB    15,29     ;Note 4
                DB    11,29     ;Note 5
                DB    11,29     ;Note 6
                DB    6,29      ;Note 7
                DB    11,29     ;Note 8
                DB    11,29     ;Note 9
                DB    15,29     ;Note 10
                DB    11,86     ;Note 11
                DB    6,29      ;Note 12
                DB    8,57      ;Note 13
                DB    10,57     ;Note 14
                DB    11,57     ;Note 15

                ;Tune number 176 *****************************************
  TUNE_176:     DB    15        ;Length of this tune(notes).
                DB    25,56     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,56     ;Note 2
                DB    21,28     ;Note 3
                DB    20,28     ;Note 4
                DB    21,28     ;Note 5
                DB    23,28     ;Note 6
                DB    21,28     ;Note 7
                DB    16,28     ;Note 8
                DB    13,28     ;Note 9
                DB    14,28     ;Note 10
                DB    16,28     ;Note 11
                DB    18,28     ;Note 12
                DB    16,28     ;Note 13
                DB    15,28     ;Note 14
                DB    16,56     ;Note 15

                ;Tune number 177 *****************************************
  TUNE_177:     DB    11        ;Length of this tune(notes).
                DB    18,29     ;Note 1 val(1-48),dur(10ms inc's).
                DB    17,29     ;Note 2
                DB    16,29     ;Note 3
                DB    15,59     ;Note 4
                DB    18,59     ;Note 5
                DB    23,118    ;Note 6
                DB    22,29     ;Note 7
                DB    21,29     ;Note 8
                DB    22,29     ;Note 9
                DB    18,29     ;Note 10
                DB    13,59     ;Note 11

                ;Tune number 178 *****************************************
  TUNE_178:     DB    16        ;Length of this tune(notes).
                DB    6,26      ;Note 1 val(1-48),dur(10ms inc's).
                DB    6,26      ;Note 2
                DB    9,26      ;Note 3
                DB    13,26     ;Note 4
                DB    11,26     ;Note 5
                DB    9,26      ;Note 6
                DB    9,39      ;Note 7
                DB    6,13      ;Note 8
                DB    9,26      ;Note 9
                DB    6,26      ;Note 10
                DB    49,52     ;Note 11
                DB    13,26     ;Note 12
                DB    16,78     ;Note 13
                DB    20,78     ;Note 14
                DB    16,26     ;Note 15
                DB    18,78     ;Note 16

                ;Tune number 179 *****************************************
  TUNE_179:     DB    11        ;Length of this tune(notes).
                DB    25,36     ;Note 1 val(1-48),dur(10ms inc's).
                DB    22,36     ;Note 2
                DB    20,36     ;Note 3
                DB    22,36     ;Note 4
                DB    17,36     ;Note 5
                DB    17,107    ;Note 6
                DB    25,36     ;Note 7
                DB    22,36     ;Note 8
                DB    20,36     ;Note 9
                DB    22,36     ;Note 10
                DB    17,71     ;Note 11

                ;Tune number 180 *****************************************
  TUNE_180:     DB    14        ;Length of this tune(notes).
                DB    18,29     ;Note 1 val(1-48),dur(10ms inc's).
                DB    11,29     ;Note 2
                DB    13,117    ;Note 3
                DB    15,29     ;Note 4
                DB    13,29     ;Note 5
                DB    18,29     ;Note 6
                DB    22,88     ;Note 7
                DB    20,29     ;Note 8
                DB    18,29     ;Note 9
                DB    18,29     ;Note 10
                DB    15,88     ;Note 11
                DB    15,29     ;Note 12
                DB    18,29     ;Note 13
                DB    13,117    ;Note 14

                ;Tune number 181 *****************************************
  TUNE_181:     DB    9         ;Length of this tune(notes).
                DB    13,50     ;Note 1 val(1-48),dur(10ms inc's).
                DB    13,17     ;Note 2
                DB    13,17     ;Note 3
                DB    13,17     ;Note 4
                DB    15,50     ;Note 5
                DB    15,50     ;Note 6
                DB    17,50     ;Note 7
                DB    13,50     ;Note 8
                DB    15,100    ;Note 9

                ;Tune number 182 *****************************************
  TUNE_182:     DB    13        ;Length of this tune(notes).
                DB    23,57     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,43     ;Note 2
                DB    25,14     ;Note 3
                DB    23,57     ;Note 4
                DB    49,29     ;Note 5
                DB    20,29     ;Note 6
                DB    21,19     ;Note 7
                DB    23,19     ;Note 8
                DB    21,19     ;Note 9
                DB    21,19     ;Note 10
                DB    20,19     ;Note 11
                DB    18,19     ;Note 12
                DB    20,57     ;Note 13

                ;Tune number 183 *****************************************
  TUNE_183:     DB    14        ;Length of this tune(notes).
                DB    18,43     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,43     ;Note 2
                DB    23,86     ;Note 3
                DB    23,86     ;Note 4
                DB    27,43     ;Note 5
                DB    25,43     ;Note 6
                DB    23,86     ;Note 7
                DB    27,86     ;Note 8
                DB    27,43     ;Note 9
                DB    25,43     ;Note 10
                DB    23,86     ;Note 11
                DB    20,43     ;Note 12
                DB    23,43     ;Note 13
                DB    18,129    ;Note 14

                ;Tune number 184 *****************************************
  TUNE_184:     DB    11        ;Length of this tune(notes).
                DB    17,36     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,36     ;Note 2
                DB    22,71     ;Note 3
                DB    17,36     ;Note 4
                DB    20,18     ;Note 5
                DB    20,18     ;Note 6
                DB    22,71     ;Note 7
                DB    20,36     ;Note 8
                DB    17,36     ;Note 9
                DB    20,36     ;Note 10
                DB    15,107    ;Note 11

                ;Tune number 185 *****************************************
  TUNE_185:     DB    16        ;Length of this tune(notes).
                DB    18,64     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,21     ;Note 2
                DB    18,43     ;Note 3
                DB    15,43     ;Note 4
                DB    11,43     ;Note 5
                DB    15,43     ;Note 6
                DB    18,64     ;Note 7
                DB    20,21     ;Note 8
                DB    18,43     ;Note 9
                DB    15,129    ;Note 10
                DB    18,86     ;Note 11
                DB    20,86     ;Note 12
                DB    16,64     ;Note 13
                DB    15,21     ;Note 14
                DB    16,43     ;Note 15
                DB    13,86     ;Note 16

                ;Tune number 186 *****************************************
  TUNE_186:     DB    14        ;Length of this tune(notes).
                DB    21,40     ;Note 1 val(1-48),dur(10ms inc's).
                DB    24,20     ;Note 2
                DB    24,40     ;Note 3
                DB    24,20     ;Note 4
                DB    22,40     ;Note 5
                DB    21,40     ;Note 6
                DB    24,120    ;Note 7
                DB    24,40     ;Note 8
                DB    26,20     ;Note 9
                DB    22,40     ;Note 10
                DB    26,20     ;Note 11
                DB    29,40     ;Note 12
                DB    26,40     ;Note 13
                DB    24,80     ;Note 14

                ;Tune number 187 *****************************************
  TUNE_187:     DB    10        ;Length of this tune(notes).
                DB    10,56     ;Note 1 val(1-48),dur(10ms inc's).
                DB    13,56     ;Note 2
                DB    17,56     ;Note 3
                DB    22,56     ;Note 4
                DB    18,111    ;Note 5
                DB    17,28     ;Note 6
                DB    15,28     ;Note 7
                DB    14,28     ;Note 8
                DB    12,28     ;Note 9
                DB    10,56     ;Note 10

                ;Tune number 188 *****************************************
  TUNE_188:     DB    10        ;Length of this tune(notes).
                DB    16,75     ;Note 1 val(1-48),dur(10ms inc's).
                DB    19,25     ;Note 2
                DB    19,25     ;Note 3
                DB    19,25     ;Note 4
                DB    16,150    ;Note 5
                DB    20,38     ;Note 6
                DB    19,38     ;Note 7
                DB    16,75     ;Note 8
                DB    12,75     ;Note 9
                DB    16,75     ;Note 10

                ;Tune number 189 *****************************************
  TUNE_189:     DB    11        ;Length of this tune(notes).
                DB    13,54     ;Note 1 val(1-48),dur(10ms inc's).
                DB    4,108     ;Note 2
                DB    5,54      ;Note 3
                DB    6,54      ;Note 4
                DB    9,54      ;Note 5
                DB    13,54     ;Note 6
                DB    18,54     ;Note 7
                DB    16,54     ;Note 8
                DB    13,54     ;Note 9
                DB    14,81     ;Note 10
                DB    18,81     ;Note 11

                ;Tune number 190 *****************************************
  TUNE_190:     DB    14        ;Length of this tune(notes).
                DB    20,31     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,31     ;Note 2
                DB    16,31     ;Note 3
                DB    13,31     ;Note 4
                DB    16,31     ;Note 5
                DB    16,31     ;Note 6
                DB    20,63     ;Note 7
                DB    20,31     ;Note 8
                DB    16,31     ;Note 9
                DB    16,31     ;Note 10
                DB    13,31     ;Note 11
                DB    16,31     ;Note 12
                DB    18,31     ;Note 13
                DB    16,63     ;Note 14

                ;Tune number 191 *****************************************
  TUNE_191:     DB    14        ;Length of this tune(notes).
                DB    11,33     ;Note 1 val(1-48),dur(10ms inc's).
                DB    13,33     ;Note 2
                DB    11,33     ;Note 3
                DB    20,33     ;Note 4
                DB    20,133    ;Note 5
                DB    11,33     ;Note 6
                DB    13,33     ;Note 7
                DB    11,33     ;Note 8
                DB    21,133    ;Note 9
                DB    11,33     ;Note 10
                DB    13,33     ;Note 11
                DB    11,33     ;Note 12
                DB    15,33     ;Note 13
                DB    11,67     ;Note 14

                ;Tune number 192 *****************************************
  TUNE_192:     DB    14        ;Length of this tune(notes).
                DB    23,28     ;Note 1 val(1-48),dur(10ms inc's).
                DB    24,28     ;Note 2
                DB    27,168    ;Note 3
                DB    23,28     ;Note 4
                DB    24,28     ;Note 5
                DB    27,28     ;Note 6
                DB    24,28     ;Note 7
                DB    27,28     ;Note 8
                DB    26,28     ;Note 9
                DB    25,56     ;Note 10
                DB    23,28     ;Note 11
                DB    24,28     ;Note 12
                DB    27,28     ;Note 13
                DB    15,168    ;Note 14

                ;Tune number 193 *****************************************
  TUNE_193:     DB    14        ;Length of this tune(notes).
                DB    23,27     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,27     ;Note 2
                DB    15,164    ;Note 3
                DB    11,27     ;Note 4
                DB    13,27     ;Note 5
                DB    14,27     ;Note 6
                DB    25,27     ;Note 7
                DB    25,27     ;Note 8
                DB    25,27     ;Note 9
                DB    25,27     ;Note 10
                DB    23,27     ;Note 11
                DB    20,27     ;Note 12
                DB    16,27     ;Note 13
                DB    13,109    ;Note 14

                ;Tune number 194 *****************************************
  TUNE_194:     DB    14        ;Length of this tune(notes).
                DB    16,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,25     ;Note 2
                DB    18,25     ;Note 3
                DB    20,25     ;Note 4
                DB    16,25     ;Note 5
                DB    20,25     ;Note 6
                DB    20,50     ;Note 7
                DB    16,25     ;Note 8
                DB    20,25     ;Note 9
                DB    18,25     ;Note 10
                DB    20,25     ;Note 11
                DB    21,25     ;Note 12
                DB    21,25     ;Note 13
                DB    21,50     ;Note 14

                ;Tune number 195 *****************************************
  TUNE_195:     DB    15        ;Length of this tune(notes).
                DB    16,52     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,26     ;Note 2
                DB    20,26     ;Note 3
                DB    18,78     ;Note 4
                DB    20,26     ;Note 5
                DB    21,26     ;Note 6
                DB    20,26     ;Note 7
                DB    18,26     ;Note 8
                DB    16,78     ;Note 9
                DB    16,26     ;Note 10
                DB    20,26     ;Note 11
                DB    18,26     ;Note 12
                DB    16,78     ;Note 13
                DB    20,26     ;Note 14
                DB    18,52     ;Note 15

                ;Tune number 196 *****************************************
  TUNE_196:     DB    18        ;Length of this tune(notes).
                DB    23,52     ;Note 1 val(1-48),dur(10ms inc's).
                DB    22,26     ;Note 2
                DB    22,26     ;Note 3
                DB    20,26     ;Note 4
                DB    19,26     ;Note 5
                DB    20,26     ;Note 6
                DB    23,26     ;Note 7
                DB    49,26     ;Note 8
                DB    22,26     ;Note 9
                DB    22,52     ;Note 10
                DB    49,52     ;Note 11
                DB    22,52     ;Note 12
                DB    18,26     ;Note 13
                DB    22,26     ;Note 14
                DB    18,26     ;Note 15
                DB    22,26     ;Note 16
                DB    25,26     ;Note 17
                DB    23,52     ;Note 18
                               
                ;Tune number 197 *****************************************
  TUNE_197:     DB    12        ;Length of this tune(notes).
                DB    27,23     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,23     ;Note 2
                DB    23,68     ;Note 3
                DB    25,23     ;Note 4
                DB    23,45     ;Note 5
                DB    23,45     ;Note 6
                DB    27,45     ;Note 7
                DB    30,45     ;Note 8
                DB    28,45     ;Note 9
                DB    32,45     ;Note 10
                DB    35,45     ;Note 11
                DB    35,45     ;Note 12

                ;Tune number 198 *****************************************
  TUNE_198:     DB    11        ;Length of this tune(notes).
                DB    16,46     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,46     ;Note 2
                DB    21,92     ;Note 3
                DB    25,46     ;Note 4
                DB    20,92     ;Note 5
                DB    18,46     ;Note 6
                DB    18,46     ;Note 7
                DB    23,46     ;Note 8
                DB    20,69     ;Note 9
                DB    18,23     ;Note 10
                DB    16,46     ;Note 11

                ;Tune number 199 *****************************************
  TUNE_199:     DB    11        ;Length of this tune(notes).
                DB    25,44     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,44     ;Note 2
                DB    26,44     ;Note 3
                DB    28,44     ;Note 4
                DB    28,44     ;Note 5
                DB    30,44     ;Note 6
                DB    32,44     ;Note 7
                DB    30,15     ;Note 8
                DB    32,15     ;Note 9
                DB    30,15     ;Note 10
                DB    28,44     ;Note 11

                ;Tune number 200 *****************************************
  TUNE_200:     DB    15        ;Length of this tune(notes).
                DB    23,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,25     ;Note 2
                DB    19,50     ;Note 3
                DB    21,25     ;Note 4
                DB    19,25     ;Note 5
                DB    18,25     ;Note 6
                DB    18,25     ;Note 7
                DB    19,25     ;Note 8
                DB    21,25     ;Note 9
                DB    19,50     ;Note 10
                DB    49,75     ;Note 11
                DB    19,25     ;Note 12
                DB    21,50     ;Note 13
                DB    19,50     ;Note 14
                DB    16,100    ;Note 15

                ;Tune number 201 *****************************************
  TUNE_201:     DB    10        ;Length of this tune(notes).
                DB    14,33     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,33     ;Note 2
                DB    19,67     ;Note 3
                DB    23,33     ;Note 4
                DB    21,33     ;Note 5
                DB    19,33     ;Note 6
                DB    16,33     ;Note 7
                DB    14,33     ;Note 8
                DB    16,33     ;Note 9
                DB    19,67     ;Note 10

                ;Tune number 202 *****************************************
  TUNE_202:     DB    9         ;Length of this tune(notes).
                DB    23,64     ;Note 1 val(1-48),dur(10ms inc's).
                DB    24,64     ;Note 2
                DB    28,64     ;Note 3
                DB    26,191    ;Note 4
                DB    28,64     ;Note 5
                DB    30,64     ;Note 6
                DB    31,64     ;Note 7
                DB    26,48     ;Note 8
                DB    23,80     ;Note 9

                ;Tune number 203 *****************************************
  TUNE_203:     DB    16        ;Length of this tune(notes).
                DB    21,75     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,38     ;Note 2
                DB    49,19     ;Note 3
                DB    21,19     ;Note 4
                DB    21,38     ;Note 5
                DB    25,38     ;Note 6
                DB    28,38     ;Note 7
                DB    28,19     ;Note 8
                DB    28,19     ;Note 9
                DB    30,38     ;Note 10
                DB    26,38     ;Note 11
                DB    23,38     ;Note 12
                DB    21,38     ;Note 13
                DB    20,38     ;Note 14
                DB    23,38     ;Note 15
                DB    16,75     ;Note 16

                ;Tune number 204 *****************************************
  TUNE_204:     DB    11        ;Length of this tune(notes).
                DB    16,19     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,19     ;Note 2
                DB    20,19     ;Note 3
                DB    21,57     ;Note 4
                DB    16,57     ;Note 5
                DB    13,57     ;Note 6
                DB    11,43     ;Note 7
                DB    9,14      ;Note 8
                DB    8,43      ;Note 9
                DB    18,14     ;Note 10
                DB    16,57     ;Note 11

                ;Tune number 205 *****************************************
  TUNE_205:     DB    10        ;Length of this tune(notes).
                DB    16,43     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,43     ;Note 2
                DB    16,32     ;Note 3
                DB    18,11     ;Note 4
                DB    20,43     ;Note 5
                DB    20,32     ;Note 6
                DB    18,11     ;Note 7
                DB    20,32     ;Note 8
                DB    21,11     ;Note 9
                DB    23,43     ;Note 10

                ;Tune number 206 *****************************************
  TUNE_206:     DB    11        ;Length of this tune(notes).
                DB    16,43     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,43     ;Note 2
                DB    20,43     ;Note 3
                DB    20,129    ;Note 4
                DB    16,86     ;Note 5
                DB    20,43     ;Note 6
                DB    20,43     ;Note 7
                DB    18,43     ;Note 8
                DB    16,43     ;Note 9
                DB    20,21     ;Note 10
                DB    18,64     ;Note 11

                ;Tune number 207 *****************************************
  TUNE_207:     DB    15        ;Length of this tune(notes).
                DB    13,53     ;Note 1 val(1-48),dur(10ms inc's).
                DB    17,53     ;Note 2
                DB    20,79     ;Note 3
                DB    20,26     ;Note 4
                DB    20,105    ;Note 5
                DB    22,53     ;Note 6
                DB    22,53     ;Note 7
                DB    20,105    ;Note 8
                DB    49,53     ;Note 9
                DB    18,105    ;Note 10
                DB    17,53     ;Note 11
                DB    13,53     ;Note 12
                DB    15,53     ;Note 13
                DB    15,53     ;Note 14
                DB    13,105    ;Note 15

                ;Tune number 208 *****************************************
  TUNE_208:     DB    11        ;Length of this tune(notes).
                DB    23,31     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,31     ;Note 2
                DB    23,31     ;Note 3
                DB    23,31     ;Note 4
                DB    25,63     ;Note 5
                DB    25,31     ;Note 6
                DB    25,31     ;Note 7
                DB    23,31     ;Note 8
                DB    20,63     ;Note 9
                DB    23,63     ;Note 10
                DB    20,94     ;Note 11

                ;Tune number 209 *****************************************
  TUNE_209:     DB    11        ;Length of this tune(notes).
                DB    22,68     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,23     ;Note 2
                DB    24,23     ;Note 3
                DB    25,23     ;Note 4
                DB    34,45     ;Note 5
                DB    30,45     ;Note 6
                DB    27,45     ;Note 7
                DB    23,45     ;Note 8
                DB    20,91     ;Note 9
                DB    23,45     ;Note 10
                DB    27,45     ;Note 11

                ;Tune number 210 *****************************************
  TUNE_210:     DB    14        ;Length of this tune(notes).
                DB    16,21     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,21     ;Note 2
                DB    18,42     ;Note 3
                DB    21,42     ;Note 4
                DB    20,42     ;Note 5
                DB    18,42     ;Note 6
                DB    18,31     ;Note 7
                DB    16,10     ;Note 8
                DB    16,83     ;Note 9
                DB    18,31     ;Note 10
                DB    16,10     ;Note 11
                DB    16,21     ;Note 12
                DB    14,21     ;Note 13
                DB    14,83     ;Note 14

                ;Tune number 211 *****************************************
  TUNE_211:     DB    10        ;Length of this tune(notes).
                DB    26,40     ;Note 1 val(1-48),dur(10ms inc's).
                DB    26,70     ;Note 2
                DB    23,10     ;Note 3
                DB    21,40     ;Note 4
                DB    18,40     ;Note 5
                DB    14,40     ;Note 6
                DB    16,40     ;Note 7
                DB    18,40     ;Note 8
                DB    16,40     ;Note 9
                DB    14,40     ;Note 10

                ;Tune number 212 *****************************************
  TUNE_212:     DB    16        ;Length of this tune(notes).
                DB    28,80     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,40     ;Note 2
                DB    21,40     ;Note 3
                DB    20,40     ;Note 4
                DB    21,40     ;Note 5
                DB    23,40     ;Note 6
                DB    23,40     ;Note 7
                DB    20,40     ;Note 8
                DB    16,80     ;Note 9
                DB    28,40     ;Note 10
                DB    30,80     ;Note 11
                DB    28,40     ;Note 12
                DB    26,40     ;Note 13
                DB    25,40     ;Note 14
                DB    23,40     ;Note 15
                DB    28,80     ;Note 16

                ;Tune number 213 *****************************************
  TUNE_213:     DB    17        ;Length of this tune(notes).
                DB    6,56      ;Note 1 val(1-48),dur(10ms inc's).
                DB    13,19     ;Note 2
                DB    15,56     ;Note 3
                DB    13,19     ;Note 4
                DB    7,56      ;Note 5
                DB    13,19     ;Note 6
                DB    15,56     ;Note 7
                DB    13,19     ;Note 8
                DB    8,56      ;Note 9
                DB    13,19     ;Note 10
                DB    15,38     ;Note 11
                DB    13,38     ;Note 12
                DB    49,38     ;Note 13
                DB    13,38     ;Note 14
                DB    15,25     ;Note 15
                DB    14,25     ;Note 16
                DB    13,25     ;Note 17

                ;Tune number 214 *****************************************
  TUNE_214:     DB    14        ;Length of this tune(notes).
                DB    18,66     ;Note 1 val(1-48),dur(10ms inc's).
                DB    21,22     ;Note 2
                DB    25,88     ;Note 3
                DB    23,66     ;Note 4
                DB    21,22     ;Note 5
                DB    18,88     ;Note 6
                DB    18,44     ;Note 7
                DB    21,44     ;Note 8
                DB    25,29     ;Note 9
                DB    26,29     ;Note 10
                DB    25,29     ;Note 11
                DB    23,66     ;Note 12
                DB    21,22     ;Note 13
                DB    18,88     ;Note 14

                ;Tune number 215 *****************************************
  TUNE_215:     DB    18        ;Length of this tune(notes).
                DB    21,36     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,71     ;Note 2
                DB    30,107    ;Note 3
                DB    29,36     ;Note 4
                DB    27,24     ;Note 5
                DB    25,24     ;Note 6
                DB    24,24     ;Note 7
                DB    23,71     ;Note 8
                DB    23,107    ;Note 9
                DB    21,36     ;Note 10
                DB    20,71     ;Note 11
                DB    32,107    ;Note 12
                DB    30,36     ;Note 13
                DB    29,24     ;Note 14
                DB    27,24     ;Note 15
                DB    25,24     ;Note 16
                DB    24,71     ;Note 17
                DB    24,107    ;Note 18

                ;Tune number 216 *****************************************
  TUNE_216:     DB    16        ;Length of this tune(notes).
                DB    23,54     ;Note 1 val(1-48),dur(10ms inc's).
                DB    22,54     ;Note 2
                DB    20,54     ;Note 3
                DB    25,81     ;Note 4
                DB    25,27     ;Note 5
                DB    25,81     ;Note 6
                DB    20,27     ;Note 7
                DB    23,27     ;Note 8
                DB    23,27     ;Note 9
                DB    23,27     ;Note 10
                DB    23,27     ;Note 11
                DB    22,27     ;Note 12
                DB    20,27     ;Note 13
                DB    18,27     ;Note 14
                DB    17,27     ;Note 15
                DB    18,108    ;Note 16

                ;Tune number 217 *****************************************
  TUNE_217:     DB    10        ;Length of this tune(notes).
                DB    11,8      ;Note 1 val(1-48),dur(10ms inc's).
                DB    13,8      ;Note 2
                DB    15,8      ;Note 3
                DB    16,8      ;Note 4
                DB    18,33     ;Note 5
                DB    18,33     ;Note 6
                DB    23,67     ;Note 7
                DB    28,33     ;Note 8
                DB    28,33     ;Note 9
                DB    27,67     ;Note 10

                ;Tune number 218 *****************************************
  TUNE_218:     DB    11        ;Length of this tune(notes).
                DB    23,19     ;Note 1 val(1-48),dur(10ms inc's).
                DB    35,19     ;Note 2
                DB    23,38     ;Note 3
                DB    23,19     ;Note 4
                DB    35,19     ;Note 5
                DB    23,38     ;Note 6
                DB    23,19     ;Note 7
                DB    35,19     ;Note 8
                DB    23,38     ;Note 9
                DB    32,38     ;Note 10
                DB    28,38     ;Note 11

                ;Tune number 219 *****************************************
  TUNE_219:     DB    13        ;Length of this tune(notes).
                DB    23,59     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,44     ;Note 2
                DB    21,15     ;Note 3
                DB    23,29     ;Note 4
                DB    28,88     ;Note 5
                DB    27,15     ;Note 6
                DB    28,15     ;Note 7
                DB    30,15     ;Note 8
                DB    30,15     ;Note 9
                DB    28,29     ;Note 10
                DB    27,29     ;Note 11
                DB    25,29     ;Note 12
                DB    23,118    ;Note 13

                ;Tune number 220 *****************************************
  TUNE_220:     DB    12        ;Length of this tune(notes).
                DB    23,50     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,17     ;Note 2
                DB    22,50     ;Note 3
                DB    22,17     ;Note 4
                DB    22,50     ;Note 5
                DB    18,17     ;Note 6
                DB    20,50     ;Note 7
                DB    20,17     ;Note 8
                DB    20,22     ;Note 9
                DB    18,22     ;Note 10
                DB    17,22     ;Note 11
                DB    18,67     ;Note 12

                ;Tune number 221 *****************************************
  TUNE_221:     DB    15        ;Length of this tune(notes).
                DB    30,44     ;Note 1 val(1-48),dur(10ms inc's).
                DB    29,44     ;Note 2
                DB    30,44     ;Note 3
                DB    33,22     ;Note 4
                DB    49,11     ;Note 5
                DB    26,11     ;Note 6
                DB    26,44     ;Note 7
                DB    49,22     ;Note 8
                DB    26,22     ;Note 9
                DB    25,22     ;Note 10
                DB    26,22     ;Note 11
                DB    30,22     ;Note 12
                DB    49,11     ;Note 13
                DB    21,11     ;Note 14
                DB    21,44     ;Note 15

                ;Tune number 222 *****************************************
  TUNE_222:     DB    8         ;Length of this tune(notes).
                DB    16,164    ;Note 1 val(1-48),dur(10ms inc's).
                DB    30,55     ;Note 2
                DB    28,109    ;Note 3
                DB    27,82     ;Note 4
                DB    25,27     ;Note 5
                DB    23,27     ;Note 6
                DB    21,27     ;Note 7
                DB    23,109    ;Note 8

                ;Tune number 223 *****************************************
  TUNE_223:     DB    18        ;Length of this tune(notes).
                DB    11,27     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,27     ;Note 2
                DB    19,27     ;Note 3
                DB    21,27     ;Note 4
                DB    22,27     ;Note 5
                DB    23,27     ;Note 6
                DB    22,27     ;Note 7
                DB    21,27     ;Note 8
                DB    19,53     ;Note 9
                DB    11,53     ;Note 10
                DB    14,53     ;Note 11
                DB    16,160    ;Note 12
                DB    18,9      ;Note 13
                DB    19,9      ;Note 14
                DB    18,9      ;Note 15
                DB    16,27     ;Note 16
                DB    14,53     ;Note 17
                DB    16,160    ;Note 18

                ;Tune number 224 *****************************************
  TUNE_224:     DB    12        ;Length of this tune(notes).
                DB    18,44     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,15     ;Note 2
                DB    21,44     ;Note 3
                DB    21,15     ;Note 4
                DB    18,44     ;Note 5
                DB    18,15     ;Note 6
                DB    15,44     ;Note 7
                DB    13,15     ;Note 8
                DB    18,117    ;Note 9
                DB    18,117    ;Note 10
                DB    6,117     ;Note 11
                DB    6,117     ;Note 12

                ;Tune number 225 *****************************************
  TUNE_225:     DB    16        ;Length of this tune(notes).
                DB    27,41     ;Note 1 val(1-48),dur(10ms inc's).
                DB    27,41     ;Note 2
                DB    25,41     ;Note 3
                DB    27,41     ;Note 4
                DB    28,21     ;Note 5
                DB    30,62     ;Note 6
                DB    30,41     ;Note 7
                DB    30,41     ;Note 8
                DB    30,21     ;Note 9
                DB    32,62     ;Note 10
                DB    30,21     ;Note 11
                DB    28,62     ;Note 12
                DB    27,41     ;Note 13
                DB    28,62     ;Note 14
                DB    25,21     ;Note 15
                DB    23,82     ;Note 16

                ;Tune number 226 *****************************************
  TUNE_226:     DB    12        ;Length of this tune(notes).
                DB    6,24      ;Note 1 val(1-48),dur(10ms inc's).
                DB    8,24      ;Note 2
                DB    10,24     ;Note 3
                DB    11,47     ;Note 4
                DB    6,24      ;Note 5
                DB    11,24     ;Note 6
                DB    15,47     ;Note 7
                DB    11,24     ;Note 8
                DB    15,24     ;Note 9
                DB    20,47     ;Note 10
                DB    18,47     ;Note 11
                DB    15,47     ;Note 12

                ;Tune number 227 *****************************************
  TUNE_227:     DB    8         ;Length of this tune(notes).
                DB    16,44     ;Note 1 val(1-48),dur(10ms inc's).
                DB    24,44     ;Note 2
                DB    21,44     ;Note 3
                DB    23,44     ;Note 4
                DB    21,44     ;Note 5
                DB    17,44     ;Note 6
                DB    19,44     ;Note 7
                DB    16,89     ;Note 8

                ;Tune number 228 *****************************************
  TUNE_228:     DB    16        ;Length of this tune(notes).
                DB    13,6      ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,6      ;Note 2
                DB    13,6      ;Note 3
                DB    25,6      ;Note 4
                DB    13,6      ;Note 5
                DB    25,6      ;Note 6
                DB    13,6      ;Note 7
                DB    25,6      ;Note 8
                DB    13,6      ;Note 9
                DB    25,6      ;Note 10
                DB    13,6      ;Note 11
                DB    25,6      ;Note 12
                DB    13,6      ;Note 13
                DB    25,6      ;Note 14
                DB    13,6      ;Note 15
                DB    25,6      ;Note 16

                ;Tune number 229 *****************************************
  TUNE_229:     DB    10        ;Length of this tune(notes).
                DB    28,75     ;Note 1 val(1-48),dur(10ms inc's).
                DB    27,25     ;Note 2
                DB    26,75     ;Note 3
                DB    25,25     ;Note 4
                DB    23,25     ;Note 5
                DB    21,25     ;Note 6
                DB    20,25     ;Note 7
                DB    18,25     ;Note 8
                DB    16,50     ;Note 9
                DB    28,50     ;Note 10

                ;Tune number 230 *****************************************
  TUNE_230:     DB    15        ;Length of this tune(notes).
                DB    17,40     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,40     ;Note 2
                DB    49,40     ;Note 3
                DB    22,60     ;Note 4
                DB    20,20     ;Note 5
                DB    22,40     ;Note 6
                DB    20,40     ;Note 7
                DB    49,40     ;Note 8
                DB    17,40     ;Note 9
                DB    15,40     ;Note 10
                DB    49,40     ;Note 11
                DB    13,40     ;Note 12
                DB    15,40     ;Note 13
                DB    17,40     ;Note 14
                DB    13,40     ;Note 15

                ;Tune number 231 *****************************************
  TUNE_231:     DB    18        ;Length of this tune(notes).
                DB    30,48     ;Note 1 val(1-48),dur(10ms inc's).
                DB    30,24     ;Note 2
                DB    32,24     ;Note 3
                DB    29,48     ;Note 4
                DB    49,48     ;Note 5
                DB    28,48     ;Note 6
                DB    28,24     ;Note 7
                DB    30,24     ;Note 8
                DB    27,48     ;Note 9
                DB    49,48     ;Note 10
                DB    30,24     ;Note 11
                DB    32,24     ;Note 12
                DB    29,48     ;Note 13
                DB    49,72     ;Note 14
                DB    28,24     ;Note 15
                DB    28,24     ;Note 16
                DB    30,24     ;Note 17
                DB    27,72     ;Note 18

                ;Tune number 232 *****************************************
  TUNE_232:     DB    13        ;Length of this tune(notes).
                DB    23,31     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,31     ;Note 2
                DB    20,31     ;Note 3
                DB    25,31     ;Note 4
                DB    23,63     ;Note 5
                DB    20,31     ;Note 6
                DB    21,31     ;Note 7
                DB    23,31     ;Note 8
                DB    23,31     ;Note 9
                DB    20,31     ;Note 10
                DB    25,31     ;Note 11
                DB    23,63     ;Note 12
                DB    20,63     ;Note 13

                ;Tune number 233 *****************************************
  TUNE_233:     DB    16        ;Length of this tune(notes).
                DB    27,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,25     ;Note 2
                DB    23,50     ;Note 3
                DB    23,25     ;Note 4
                DB    25,25     ;Note 5
                DB    23,50     ;Note 6
                DB    49,50     ;Note 7
                DB    23,25     ;Note 8
                DB    25,25     ;Note 9
                DB    23,25     ;Note 10
                DB    25,25     ;Note 11
                DB    27,50     ;Note 12
                DB    49,50     ;Note 13
                DB    27,50     ;Note 14
                DB    30,38     ;Note 15
                DB    27,63     ;Note 16

                ;Tune number 234 *****************************************
  TUNE_234:     DB    17        ;Length of this tune(notes).
                DB    15,35     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,12     ;Note 2
                DB    18,93     ;Note 3
                DB    16,16     ;Note 4
                DB    15,16     ;Note 5
                DB    13,16     ;Note 6
                DB    15,47     ;Note 7
                DB    16,47     ;Note 8
                DB    17,47     ;Note 9
                DB    18,47     ;Note 10
                DB    20,35     ;Note 11
                DB    23,12     ;Note 12
                DB    23,93     ;Note 13
                DB    25,16     ;Note 14
                DB    23,16     ;Note 15
                DB    20,16     ;Note 16
                DB    18,140    ;Note 17

                ;Tune number 235 *****************************************
  TUNE_235:     DB    11        ;Length of this tune(notes).
                DB    11,100    ;Note 1 val(1-48),dur(10ms inc's).
                DB    11,75     ;Note 2
                DB    11,25     ;Note 3
                DB    11,100    ;Note 4
                DB    14,50     ;Note 5
                DB    13,50     ;Note 6
                DB    13,75     ;Note 7
                DB    11,25     ;Note 8
                DB    11,50     ;Note 9
                DB    10,50     ;Note 10
                DB    11,100    ;Note 11

                ;Tune number 236 *****************************************
  TUNE_236:     DB    13        ;Length of this tune(notes).
                DB    16,32     ;Note 1 val(1-48),dur(10ms inc's).
                DB    13,32     ;Note 2
                DB    18,64     ;Note 3
                DB    13,64     ;Note 4
                DB    15,64     ;Note 5
                DB    16,64     ;Note 6
                DB    21,64     ;Note 7
                DB    20,64     ;Note 8
                DB    18,64     ;Note 9
                DB    16,32     ;Note 10
                DB    13,32     ;Note 11
                DB    18,64     ;Note 12
                DB    13,64     ;Note 13

                ;Tune number 237 *****************************************
  TUNE_237:     DB    12        ;Length of this tune(notes).
                DB    32,19     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,19     ;Note 2
                DB    32,19     ;Note 3
                DB    25,19     ;Note 4
                DB    34,38     ;Note 5
                DB    32,38     ;Note 6
                DB    49,19     ;Note 7
                DB    25,19     ;Note 8
                DB    32,19     ;Note 9
                DB    25,19     ;Note 10
                DB    34,38     ;Note 11
                DB    32,38     ;Note 12

                ;Tune number 238 *****************************************
  TUNE_238:     DB    11        ;Length of this tune(notes).
                DB    16,28     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,28     ;Note 2
                DB    20,28     ;Note 3
                DB    21,56     ;Note 4
                DB    21,56     ;Note 5
                DB    21,56     ;Note 6
                DB    21,56     ;Note 7
                DB    21,28     ;Note 8
                DB    25,28     ;Note 9
                DB    23,28     ;Note 10
                DB    21,111    ;Note 11

                ;Tune number 239 *****************************************
  TUNE_239:     DB    10        ;Length of this tune(notes).
                DB    25,109    ;Note 1 val(1-48),dur(10ms inc's).
                DB    24,55     ;Note 2
                DB    25,27     ;Note 3
                DB    24,27     ;Note 4
                DB    22,109    ;Note 5
                DB    25,109    ;Note 6
                DB    27,55     ;Note 7
                DB    25,27     ;Note 8
                DB    22,27     ;Note 9
                DB    20,55     ;Note 10

                ;Tune number 240 *****************************************
  TUNE_240:     DB    10        ;Length of this tune(notes).
                DB    22,35     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,35     ;Note 2
                DB    22,35     ;Note 3
                DB    25,35     ;Note 4
                DB    22,35     ;Note 5
                DB    25,71     ;Note 6
                DB    27,141    ;Note 7
                DB    25,71     ;Note 8
                DB    27,35     ;Note 9
                DB    25,106    ;Note 10

                ;Tune number 241 *****************************************
  TUNE_241:     DB    18        ;Length of this tune(notes).
                DB    25,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    22,25     ;Note 2
                DB    49,25     ;Note 3
                DB    22,25     ;Note 4
                DB    23,25     ;Note 5
                DB    25,25     ;Note 6
                DB    34,50     ;Note 7
                DB    34,50     ;Note 8
                DB    30,100    ;Note 9
                DB    25,25     ;Note 10
                DB    22,25     ;Note 11
                DB    49,25     ;Note 12
                DB    22,25     ;Note 13
                DB    23,25     ;Note 14
                DB    22,25     ;Note 15
                DB    25,50     ;Note 16
                DB    25,50     ;Note 17
                DB    23,100    ;Note 18

                ;Tune number 242 *****************************************
  TUNE_242:     DB    14        ;Length of this tune(notes).
                DB    11,31     ;Note 1 val(1-48),dur(10ms inc's).
                DB    15,31     ;Note 2
                DB    18,47     ;Note 3
                DB    18,16     ;Note 4
                DB    18,47     ;Note 5
                DB    15,16     ;Note 6
                DB    18,63     ;Note 7
                DB    11,31     ;Note 8
                DB    15,31     ;Note 9
                DB    13,47     ;Note 10
                DB    13,16     ;Note 11
                DB    18,47     ;Note 12
                DB    18,16     ;Note 13
                DB    11,63     ;Note 14

                ;Tune number 243 *****************************************
  TUNE_243:     DB    15        ;Length of this tune(notes).
                DB    21,96     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,32     ;Note 2
                DB    22,32     ;Note 3
                DB    13,32     ;Note 4
                DB    18,32     ;Note 5
                DB    23,32     ;Note 6
                DB    22,64     ;Note 7
                DB    18,128    ;Note 8
                DB    20,96     ;Note 9
                DB    22,32     ;Note 10
                DB    20,32     ;Note 11
                DB    13,32     ;Note 12
                DB    17,32     ;Note 13
                DB    23,32     ;Note 14
                DB    22,96     ;Note 15

                ;Tune number 244 *****************************************
  TUNE_244:     DB    14        ;Length of this tune(notes).
                DB    20,67     ;Note 1 val(1-48),dur(10ms inc's).
                DB    16,67     ;Note 2
                DB    13,67     ;Note 3
                DB    11,67     ;Note 4
                DB    13,33     ;Note 5
                DB    14,33     ;Note 6
                DB    49,33     ;Note 7
                DB    20,100    ;Note 8
                DB    16,33     ;Note 9
                DB    18,33     ;Note 10
                DB    16,33     ;Note 11
                DB    9,44      ;Note 12
                DB    9,44      ;Note 13
                DB    9,44      ;Note 14

                ;Tune number 245 *****************************************
  TUNE_245:     DB    15        ;Length of this tune(notes).
                DB    20,33     ;Note 1 val(1-48),dur(10ms inc's).
                DB    17,33     ;Note 2
                DB    20,33     ;Note 3
                DB    20,33     ;Note 4
                DB    20,33     ;Note 5
                DB    17,33     ;Note 6
                DB    20,33     ;Note 7
                DB    25,33     ;Note 8
                DB    25,33     ;Note 9
                DB    22,133    ;Note 10
                DB    20,133    ;Note 11
                DB    20,33     ;Note 12
                DB    20,33     ;Note 13
                DB    20,33     ;Note 14
                DB    17,133    ;Note 15

                ;Tune number 246 *****************************************
  TUNE_246:     DB    16        ;Length of this tune(notes).
                DB    25,23     ;Note 1 val(1-48),dur(10ms inc's).
                DB    25,23     ;Note 2
                DB    25,23     ;Note 3
                DB    22,35     ;Note 4
                DB    25,104    ;Note 5
                DB    25,23     ;Note 6
                DB    25,23     ;Note 7
                DB    25,23     ;Note 8
                DB    22,35     ;Note 9
                DB    25,104    ;Note 10
                DB    27,35     ;Note 11
                DB    27,70     ;Note 12
                DB    27,70     ;Note 13
                DB    25,35     ;Note 14
                DB    23,35     ;Note 15
                DB    22,139    ;Note 16

                ;Tune number 247 *****************************************
  TUNE_247:     DB    11        ;Length of this tune(notes).
                DB    20,58     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,19     ;Note 2
                DB    20,77     ;Note 3
                DB    20,38     ;Note 4
                DB    20,38     ;Note 5
                DB    17,38     ;Note 6
                DB    22,38     ;Note 7
                DB    22,38     ;Note 8
                DB    22,38     ;Note 9
                DB    18,38     ;Note 10
                DB    20,77     ;Note 11

                ;Tune number 248 *****************************************
  TUNE_248:     DB    18        ;Length of this tune(notes).
                DB    8,25      ;Note 1 val(1-48),dur(10ms inc's).
                DB    8,75      ;Note 2
                DB    8,25      ;Note 3
                DB    10,25     ;Note 4
                DB    11,75     ;Note 5
                DB    11,25     ;Note 6
                DB    11,25     ;Note 7
                DB    10,25     ;Note 8
                DB    49,50     ;Note 9
                DB    23,50     ;Note 10
                DB    18,50     ;Note 11
                DB    18,50     ;Note 12
                DB    17,50     ;Note 13
                DB    49,50     ;Note 14
                DB    17,50     ;Note 15
                DB    20,50     ;Note 16
                DB    23,50     ;Note 17
                DB    22,50     ;Note 18

                ;Tune number 249 *****************************************
  TUNE_249:     DB    11        ;Length of this tune(notes).
                DB    22,24     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,47     ;Note 2
                DB    23,24     ;Note 3
                DB    22,24     ;Note 4
                DB    20,71     ;Note 5
                DB    20,24     ;Note 6
                DB    22,24     ;Note 7
                DB    22,24     ;Note 8
                DB    22,24     ;Note 9
                DB    20,24     ;Note 10
                DB    18,94     ;Note 11

                ;Tune number 250 *****************************************
  TUNE_250:     DB    10        ;Length of this tune(notes).
                DB    23,43     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,14     ;Note 2
                DB    23,29     ;Note 3
                DB    23,29     ;Note 4
                DB    22,57     ;Note 5
                DB    18,57     ;Note 6
                DB    20,57     ;Note 7
                DB    23,29     ;Note 8
                DB    23,29     ;Note 9
                DB    18,57     ;Note 10

                ;Tune number 251 *****************************************
  TUNE_251:     DB    15        ;Length of this tune(notes).
                DB    22,81     ;Note 1 val(1-48),dur(10ms inc's).
                DB    20,27     ;Note 2
                DB    20,27     ;Note 3
                DB    18,27     ;Note 4
                DB    17,27     ;Note 5
                DB    18,27     ;Note 6
                DB    20,108    ;Note 7
                DB    15,108    ;Note 8
                DB    20,81     ;Note 9
                DB    22,27     ;Note 10
                DB    20,27     ;Note 11
                DB    17,27     ;Note 12
                DB    15,27     ;Note 13
                DB    13,27     ;Note 14
                DB    18,54     ;Note 15

                ;Tune number 252 *****************************************
  TUNE_252:     DB    13        ;Length of this tune(notes).
                DB    18,63     ;Note 1 val(1-48),dur(10ms inc's).
                DB    17,63     ;Note 2
                DB    15,47     ;Note 3
                DB    11,16     ;Note 4
                DB    15,63     ;Note 5
                DB    49,31     ;Note 6
                DB    17,31     ;Note 7
                DB    20,21     ;Note 8
                DB    18,21     ;Note 9
                DB    15,21     ;Note 10
                DB    13,47     ;Note 11
                DB    10,16     ;Note 12
                DB    13,63     ;Note 13

                ;Tune number 253 *****************************************
  TUNE_253:     DB    13        ;Length of this tune(notes).
                DB    13,33     ;Note 1 val(1-48),dur(10ms inc's).
                DB    18,50     ;Note 2
                DB    18,17     ;Note 3
                DB    22,50     ;Note 4
                DB    13,17     ;Note 5
                DB    18,100    ;Note 6
                DB    13,33     ;Note 7
                DB    18,22     ;Note 8
                DB    22,22     ;Note 9
                DB    25,22     ;Note 10
                DB    22,50     ;Note 11
                DB    13,17     ;Note 12
                DB    18,67     ;Note 13

                ;Tune number 254 *****************************************
  TUNE_254:     DB    10        ;Length of this tune(notes).
                DB    18,57     ;Note 1 val(1-48),dur(10ms inc's).
                DB    23,57     ;Note 2
                DB    23,57     ;Note 3
                DB    18,57     ;Note 4
                DB    20,19     ;Note 5
                DB    19,19     ;Note 6
                DB    20,19     ;Note 7
                DB    23,43     ;Note 8
                DB    20,14     ;Note 9
                DB    18,57     ;Note 10

                ;Tune number 255 *****************************************
  TUNE_255:     DB    17        ;Length of this tune(notes).
                DB    20,25     ;Note 1 val(1-48),dur(10ms inc's).
                DB    19,8      ;Note 2
                DB    20,33     ;Note 3
                DB    25,33     ;Note 4
                DB    20,33     ;Note 5
                DB    20,33     ;Note 6
                DB    18,33     ;Note 7
                DB    24,33     ;Note 8
                DB    49,33     ;Note 9
                DB    17,25     ;Note 10
                DB    16,8      ;Note 11
                DB    17,33     ;Note 12
                DB    22,33     ;Note 13
                DB    17,33     ;Note 14
                DB    17,33     ;Note 15
                DB    15,33     ;Note 16
                DB    20,33     ;Note 17

;******************************************************************************
;                           CHARACTER FONT CONSTANTS
;******************************************************************************
;The following are the constants for the 7-segment digit font.  Each line of
;one byte defines one digit.
;
;Each digit is made up of 7 segments plus a decimal point.  These 8 elements
;correspond to the 8-bits of the bytes below. The segments follow a standard
;7-segment labeling convention, as follows:
;
;       A
;     *****
;    *     *
;  F *     * B
;    *     *
;    *  G  *
;     *****
;    *     *
;  E *     * C
;    *     *
;    *     *
;     *****
;       D    DP
;
;and the mapping of the bit positions within the byte to segments is:
;
;  Bit 0 (LSB) -- segment G
;  Bit 1 -------- segment F
;  Bit 2 -------- segment E
;  Bit 3 -------- segment D
;  Bit 4 -------- segment C
;  Bit 5 -------- segment B
;  Bit 6 -------- segment A
;  Bit 7 (MSB) -- decimal point
;
;The following font defines the standard ASCII digits corresponding to the hex
;codes 30h through 39h, plus some special characters.
;
CHAR_FONT:      MOVC       A, @A+PC
                RET
                DB  00h                            ;20h   (space)
                DB  00h                            ;21h !
                DB  00h                            ;22h "
                DB  00h                            ;23h #
                DB  00h                            ;24h $
                DB  00h                            ;25h %
                DB  00h                            ;26h &
                DB  00h                            ;27h '
                DB  00h                            ;28h (
                DB  00h                            ;29h )
                DB  00h                            ;2Ah * (the NOP character)
                DB  00h                            ;2Bh +
                DB  00h                            ;2Ch ,
                DB  01h                            ;2Dh -
                DB  00h                            ;2Eh .
                DB  00h                            ;2Fh /
                DB  7Eh                            ;30h 0
                DB  30h                            ;31h 1
                DB  6Dh                            ;32h 2
                DB  79h                            ;33h 3
                DB  33h                            ;34h 4
                DB  5Bh                            ;35h 5
                DB  5Fh                            ;36h 6
                DB  70h                            ;37h 7
                DB  7Fh                            ;38h 8
                DB  7Bh                            ;39h 9
                DB  00h                            ;3Ah :
                DB  00h                            ;3Bh ;
                DB  00h                            ;3Ch <
                DB  00h                            ;3Dh =
                DB  00h                            ;3Eh >
                DB  00h                            ;3Fh ?
                DB  00h                            ;40h @
                DB 0FEh ;use as 0 w/ RH decimal pt ;41h A (upper case)
                DB 0B0h ;use as 1 w/ RH decimal pt ;42h B
                DB 0EDh ;use as 2 w/ RH decimal pt ;43h C
                DB 0F9h ;use as 3 w/ RH decimal pt ;44h D
                DB 0B3h ;use as 4 w/ RH decimal pt ;45h E
                DB 0DBh ;use as 5 w/ RH decimal pt ;46h F
                DB 0DFh ;use as 6 w/ RH decimal pt ;47h G
                DB 0F0h ;use as 7 w/ RH decimal pt ;48h H
                DB 0FFh ;use as 8 w/ RH decimal pt ;49h I
                DB 0FBh ;use as 9 w/ RH decimal pt ;4Ah J
                DB  00h                            ;4Bh K
                DB  00h                            ;4Ch L
                DB  00h                            ;4Dh M
                DB  00h                            ;4Eh N
                DB  00h                            ;4Fh O
                DB  00h                            ;50h P
                DB  00h                            ;51h Q
                DB  00h                            ;52h R
                DB  00h                            ;53h S
                DB  00h                            ;54h T
                DB  00h                            ;55h U
                DB  1Ch                            ;56h V
                DB  00h                            ;57h W
                DB  00h                            ;58h X
                DB  00h                            ;59h Y
                DB  00h                            ;5Ah Z

;******************************************************************************
;                   Look Up Table: TUNE_REF_LUT0
;******************************************************************************
;
TUNE_REF_LUT0:  ;This is a look-up-table (LUT) of pointers that point
                ;to labels which indicate the starting points of
                ;tunes 1 - 127.

                DW         TUNE_001    ;Pointer to label of Tune 1
                DW         TUNE_002    ;Pointer to label of Tune 2
                DW         TUNE_003    ;Pointer to label of Tune 3
                DW         TUNE_004    ;Pointer to label of Tune 4
                DW         TUNE_005    ;Pointer to label of Tune 5
                DW         TUNE_006    ;Pointer to label of Tune 6
                DW         TUNE_007    ;Pointer to label of Tune 7
                DW         TUNE_008    ;Pointer to label of Tune 8
                DW         TUNE_009    ;Pointer to label of Tune 9
                DW         TUNE_010    ;Pointer to label of Tune 10
                DW         TUNE_011    ;Pointer to label of Tune 11
                DW         TUNE_012    ;Pointer to label of Tune 12
                DW         TUNE_013    ;Pointer to label of Tune 13
                DW         TUNE_014    ;Pointer to label of Tune 14
                DW         TUNE_015    ;Pointer to label of Tune 15
                DW         TUNE_016    ;Pointer to label of Tune 16
                DW         TUNE_017    ;Pointer to label of Tune 17
                DW         TUNE_018    ;Pointer to label of Tune 18
                DW         TUNE_019    ;Pointer to label of Tune 19
                DW         TUNE_020    ;Pointer to label of Tune 20
                DW         TUNE_021    ;Pointer to label of Tune 21
                DW         TUNE_022    ;Pointer to label of Tune 22
                DW         TUNE_023    ;Pointer to label of Tune 23
                DW         TUNE_024    ;Pointer to label of Tune 24
                DW         TUNE_025    ;Pointer to label of Tune 25
                DW         TUNE_026    ;Pointer to label of Tune 26
                DW         TUNE_027    ;Pointer to label of Tune 27
                DW         TUNE_028    ;Pointer to label of Tune 28
                DW         TUNE_029    ;Pointer to label of Tune 29
                DW         TUNE_030    ;Pointer to label of Tune 30
                DW         TUNE_031    ;Pointer to label of Tune 31
                DW         TUNE_032    ;Pointer to label of Tune 32
                DW         TUNE_033    ;Pointer to label of Tune 33
                DW         TUNE_034    ;Pointer to label of Tune 34
                DW         TUNE_035    ;Pointer to label of Tune 35
                DW         TUNE_036    ;Pointer to label of Tune 36
                DW         TUNE_037    ;Pointer to label of Tune 37
                DW         TUNE_038    ;Pointer to label of Tune 38
                DW         TUNE_039    ;Pointer to label of Tune 39
                DW         TUNE_040    ;Pointer to label of Tune 40
                DW         TUNE_041    ;Pointer to label of Tune 41
                DW         TUNE_042    ;Pointer to label of Tune 42
                DW         TUNE_043    ;Pointer to label of Tune 43
                DW         TUNE_044    ;Pointer to label of Tune 44
                DW         TUNE_045    ;Pointer to label of Tune 45
                DW         TUNE_046    ;Pointer to label of Tune 46
                DW         TUNE_047    ;Pointer to label of Tune 47
                DW         TUNE_048    ;Pointer to label of Tune 48
                DW         TUNE_049    ;Pointer to label of Tune 49
                DW         TUNE_050    ;Pointer to label of Tune 50
                DW         TUNE_051    ;Pointer to label of Tune 51
                DW         TUNE_052    ;Pointer to label of Tune 52
                DW         TUNE_053    ;Pointer to label of Tune 53
                DW         TUNE_054    ;Pointer to label of Tune 54
                DW         TUNE_055    ;Pointer to label of Tune 55
                DW         TUNE_056    ;Pointer to label of Tune 56
                DW         TUNE_057    ;Pointer to label of Tune 57
                DW         TUNE_058    ;Pointer to label of Tune 58
                DW         TUNE_059    ;Pointer to label of Tune 59
                DW         TUNE_060    ;Pointer to label of Tune 60
                DW         TUNE_061    ;Pointer to label of Tune 61
                DW         TUNE_062    ;Pointer to label of Tune 62
                DW         TUNE_063    ;Pointer to label of Tune 63
                DW         TUNE_064    ;Pointer to label of Tune 64
                DW         TUNE_065    ;Pointer to label of Tune 65
                DW         TUNE_066    ;Pointer to label of Tune 66
                DW         TUNE_067    ;Pointer to label of Tune 67
                DW         TUNE_068    ;Pointer to label of Tune 68
                DW         TUNE_069    ;Pointer to label of Tune 69
                DW         TUNE_070    ;Pointer to label of Tune 70
                DW         TUNE_071    ;Pointer to label of Tune 71
                DW         TUNE_072    ;Pointer to label of Tune 72
                DW         TUNE_073    ;Pointer to label of Tune 73
                DW         TUNE_074    ;Pointer to label of Tune 74
                DW         TUNE_075    ;Pointer to label of Tune 75
                DW         TUNE_076    ;Pointer to label of Tune 76
                DW         TUNE_077    ;Pointer to label of Tune 77
                DW         TUNE_078    ;Pointer to label of Tune 78
                DW         TUNE_079    ;Pointer to label of Tune 79
                DW         TUNE_080    ;Pointer to label of Tune 80
                DW         TUNE_081    ;Pointer to label of Tune 81
                DW         TUNE_082    ;Pointer to label of Tune 82
                DW         TUNE_083    ;Pointer to label of Tune 83
                DW         TUNE_084    ;Pointer to label of Tune 84
                DW         TUNE_085    ;Pointer to label of Tune 85
                DW         TUNE_086    ;Pointer to label of Tune 86
                DW         TUNE_087    ;Pointer to label of Tune 87
                DW         TUNE_088    ;Pointer to label of Tune 88
                DW         TUNE_089    ;Pointer to label of Tune 89
                DW         TUNE_090    ;Pointer to label of Tune 90
                DW         TUNE_091    ;Pointer to label of Tune 91
                DW         TUNE_092    ;Pointer to label of Tune 92
                DW         TUNE_093    ;Pointer to label of Tune 93
                DW         TUNE_094    ;Pointer to label of Tune 94
                DW         TUNE_095    ;Pointer to label of Tune 95
                DW         TUNE_096    ;Pointer to label of Tune 96
                DW         TUNE_097    ;Pointer to label of Tune 97
                DW         TUNE_098    ;Pointer to label of Tune 98
                DW         TUNE_099    ;Pointer to label of Tune 99
                DW         TUNE_100    ;Pointer to label of Tune 100
                DW         TUNE_101    ;Pointer to label of Tune 101
                DW         TUNE_102    ;Pointer to label of Tune 102
                DW         TUNE_103    ;Pointer to label of Tune 103
                DW         TUNE_104    ;Pointer to label of Tune 104
                DW         TUNE_105    ;Pointer to label of Tune 105
                DW         TUNE_106    ;Pointer to label of Tune 106
                DW         TUNE_107    ;Pointer to label of Tune 107
                DW         TUNE_108    ;Pointer to label of Tune 108
                DW         TUNE_109    ;Pointer to label of Tune 109
                DW         TUNE_110    ;Pointer to label of Tune 110
                DW         TUNE_111    ;Pointer to label of Tune 111
                DW         TUNE_112    ;Pointer to label of Tune 112
                DW         TUNE_113    ;Pointer to label of Tune 113
                DW         TUNE_114    ;Pointer to label of Tune 114
                DW         TUNE_115    ;Pointer to label of Tune 115
                DW         TUNE_116    ;Pointer to label of Tune 116
                DW         TUNE_117    ;Pointer to label of Tune 117
                DW         TUNE_118    ;Pointer to label of Tune 118
                DW         TUNE_119    ;Pointer to label of Tune 119
                DW         TUNE_120    ;Pointer to label of Tune 120
                DW         TUNE_121    ;Pointer to label of Tune 121
                DW         TUNE_122    ;Pointer to label of Tune 122
                DW         TUNE_123    ;Pointer to label of Tune 123
                DW         TUNE_124    ;Pointer to label of Tune 124
                DW         TUNE_125    ;Pointer to label of Tune 125
                DW         TUNE_126    ;Pointer to label of Tune 126
                DW         TUNE_127    ;Pointer to label of Tune 127

;******************************************************************************
;                   Look Up Table: TUNE_REF_LUT1
;******************************************************************************
;
TUNE_REF_LUT1:  ;This is a look-up-table (LUT) of pointers that point
                ;to labels which indicate the starting points of
                ;tunes 128 - 255.

                DW         TUNE_128    ;Pointer to label of Tune 128
                DW         TUNE_129    ;Pointer to label of Tune 129
                DW         TUNE_130    ;Pointer to label of Tune 130
                DW         TUNE_131    ;Pointer to label of Tune 131
                DW         TUNE_132    ;Pointer to label of Tune 132
                DW         TUNE_133    ;Pointer to label of Tune 133
                DW         TUNE_134    ;Pointer to label of Tune 134
                DW         TUNE_135    ;Pointer to label of Tune 135
                DW         TUNE_136    ;Pointer to label of Tune 136
                DW         TUNE_137    ;Pointer to label of Tune 137
                DW         TUNE_138    ;Pointer to label of Tune 138
                DW         TUNE_139    ;Pointer to label of Tune 139
                DW         TUNE_140    ;Pointer to label of Tune 140
                DW         TUNE_141    ;Pointer to label of Tune 141
                DW         TUNE_142    ;Pointer to label of Tune 142
                DW         TUNE_143    ;Pointer to label of Tune 143
                DW         TUNE_144    ;Pointer to label of Tune 144
                DW         TUNE_145    ;Pointer to label of Tune 145
                DW         TUNE_146    ;Pointer to label of Tune 146
                DW         TUNE_147    ;Pointer to label of Tune 147
                DW         TUNE_148    ;Pointer to label of Tune 148
                DW         TUNE_149    ;Pointer to label of Tune 149
                DW         TUNE_150    ;Pointer to label of Tune 150
                DW         TUNE_151    ;Pointer to label of Tune 151
                DW         TUNE_152    ;Pointer to label of Tune 152
                DW         TUNE_153    ;Pointer to label of Tune 153
                DW         TUNE_154    ;Pointer to label of Tune 154
                DW         TUNE_155    ;Pointer to label of Tune 155
                DW         TUNE_156    ;Pointer to label of Tune 156
                DW         TUNE_157    ;Pointer to label of Tune 157
                DW         TUNE_158    ;Pointer to label of Tune 158
                DW         TUNE_159    ;Pointer to label of Tune 159
                DW         TUNE_160    ;Pointer to label of Tune 160
                DW         TUNE_161    ;Pointer to label of Tune 161
                DW         TUNE_162    ;Pointer to label of Tune 162
                DW         TUNE_163    ;Pointer to label of Tune 163
                DW         TUNE_164    ;Pointer to label of Tune 164
                DW         TUNE_165    ;Pointer to label of Tune 165
                DW         TUNE_166    ;Pointer to label of Tune 166
                DW         TUNE_167    ;Pointer to label of Tune 167
                DW         TUNE_168    ;Pointer to label of Tune 168
                DW         TUNE_169    ;Pointer to label of Tune 169
                DW         TUNE_170    ;Pointer to label of Tune 170
                DW         TUNE_171    ;Pointer to label of Tune 171
                DW         TUNE_172    ;Pointer to label of Tune 172
                DW         TUNE_173    ;Pointer to label of Tune 173
                DW         TUNE_174    ;Pointer to label of Tune 174
                DW         TUNE_175    ;Pointer to label of Tune 175
                DW         TUNE_176    ;Pointer to label of Tune 176
                DW         TUNE_177    ;Pointer to label of Tune 177
                DW         TUNE_178    ;Pointer to label of Tune 178
                DW         TUNE_179    ;Pointer to label of Tune 179
                DW         TUNE_180    ;Pointer to label of Tune 180
                DW         TUNE_181    ;Pointer to label of Tune 181
                DW         TUNE_182    ;Pointer to label of Tune 182
                DW         TUNE_183    ;Pointer to label of Tune 183
                DW         TUNE_184    ;Pointer to label of Tune 184
                DW         TUNE_185    ;Pointer to label of Tune 185
                DW         TUNE_186    ;Pointer to label of Tune 186
                DW         TUNE_187    ;Pointer to label of Tune 187
                DW         TUNE_188    ;Pointer to label of Tune 188
                DW         TUNE_189    ;Pointer to label of Tune 189
                DW         TUNE_190    ;Pointer to label of Tune 190
                DW         TUNE_191    ;Pointer to label of Tune 191
                DW         TUNE_192    ;Pointer to label of Tune 192
                DW         TUNE_193    ;Pointer to label of Tune 193
                DW         TUNE_194    ;Pointer to label of Tune 194
                DW         TUNE_195    ;Pointer to label of Tune 195
                DW         TUNE_196    ;Pointer to label of Tune 196
                DW         TUNE_197    ;Pointer to label of Tune 197
                DW         TUNE_198    ;Pointer to label of Tune 198
                DW         TUNE_199    ;Pointer to label of Tune 199
                DW         TUNE_200    ;Pointer to label of Tune 200
                DW         TUNE_201    ;Pointer to label of Tune 201
                DW         TUNE_202    ;Pointer to label of Tune 202
                DW         TUNE_203    ;Pointer to label of Tune 203
                DW         TUNE_204    ;Pointer to label of Tune 204
                DW         TUNE_205    ;Pointer to label of Tune 205
                DW         TUNE_206    ;Pointer to label of Tune 206
                DW         TUNE_207    ;Pointer to label of Tune 207
                DW         TUNE_208    ;Pointer to label of Tune 208
                DW         TUNE_209    ;Pointer to label of Tune 209
                DW         TUNE_210    ;Pointer to label of Tune 210
                DW         TUNE_211    ;Pointer to label of Tune 211
                DW         TUNE_212    ;Pointer to label of Tune 212
                DW         TUNE_213    ;Pointer to label of Tune 213
                DW         TUNE_214    ;Pointer to label of Tune 214
                DW         TUNE_215    ;Pointer to label of Tune 215
                DW         TUNE_216    ;Pointer to label of Tune 216
                DW         TUNE_217    ;Pointer to label of Tune 217
                DW         TUNE_218    ;Pointer to label of Tune 218
                DW         TUNE_219    ;Pointer to label of Tune 219
                DW         TUNE_220    ;Pointer to label of Tune 220
                DW         TUNE_221    ;Pointer to label of Tune 221
                DW         TUNE_222    ;Pointer to label of Tune 222
                DW         TUNE_223    ;Pointer to label of Tune 223
                DW         TUNE_224    ;Pointer to label of Tune 224
                DW         TUNE_225    ;Pointer to label of Tune 225
                DW         TUNE_226    ;Pointer to label of Tune 226
                DW         TUNE_227    ;Pointer to label of Tune 227
                DW         TUNE_228    ;Pointer to label of Tune 228
                DW         TUNE_229    ;Pointer to label of Tune 229
                DW         TUNE_230    ;Pointer to label of Tune 230
                DW         TUNE_231    ;Pointer to label of Tune 231
                DW         TUNE_232    ;Pointer to label of Tune 232
                DW         TUNE_233    ;Pointer to label of Tune 233
                DW         TUNE_234    ;Pointer to label of Tune 234
                DW         TUNE_235    ;Pointer to label of Tune 235
                DW         TUNE_236    ;Pointer to label of Tune 236
                DW         TUNE_237    ;Pointer to label of Tune 237
                DW         TUNE_238    ;Pointer to label of Tune 238
                DW         TUNE_239    ;Pointer to label of Tune 239
                DW         TUNE_240    ;Pointer to label of Tune 240
                DW         TUNE_241    ;Pointer to label of Tune 241
                DW         TUNE_242    ;Pointer to label of Tune 242
                DW         TUNE_243    ;Pointer to label of Tune 243
                DW         TUNE_244    ;Pointer to label of Tune 244
                DW         TUNE_245    ;Pointer to label of Tune 245
                DW         TUNE_246    ;Pointer to label of Tune 246
                DW         TUNE_247    ;Pointer to label of Tune 247
                DW         TUNE_248    ;Pointer to label of Tune 248
                DW         TUNE_249    ;Pointer to label of Tune 249
                DW         TUNE_250    ;Pointer to label of Tune 250
                DW         TUNE_251    ;Pointer to label of Tune 251
                DW         TUNE_252    ;Pointer to label of Tune 252
                DW         TUNE_253    ;Pointer to label of Tune 253
                DW         TUNE_254    ;Pointer to label of Tune 254
                DW         TUNE_255    ;Pointer to label of Tune 255

;******************************************************************************
;                   Look Up Table: NOTE_STOR_LUT
;******************************************************************************
;
NOTE_STOR_LUT:  MOVC       A, @A+PC
                RET

                ;48 notes total, 12 notes per octave * 4 octaves(0-3).

                ;Octave 0 ****************************************
                DW         (0FFFFh-4189) ;A,   110.000 Hz
                DW         (0FFFFh-3954) ;A#,  116.541 Hz
                DW         (0FFFFh-3732) ;B,   123.471 Hz
                DW         (0FFFFh-3523) ;C,   130.813 Hz
                DW         (0FFFFh-3325) ;C#,  138.591 Hz
                DW         (0FFFFh-3138) ;D,   146.832 Hz
                DW         (0FFFFh-2962) ;D#,  155.563 Hz
                DW         (0FFFFh-2796) ;E,   164.814 Hz
                DW         (0FFFFh-2639) ;F,   174.614 Hz
                DW         (0FFFFh-2491) ;F#,  184.997 Hz
                DW         (0FFFFh-2351) ;G,   195.998 Hz
                DW         (0FFFFh-2219) ;G#,  207.652 Hz

                ;Octave 1 ****************************************
                DW         (0FFFFh-2095) ;A,   220.000 Hz
                DW         (0FFFFh-1977) ;A#,  233.082 Hz
                DW         (0FFFFh-1866) ;B,   246.942 Hz
                DW         (0FFFFh-1761) ;C,   261.626 Hz
                DW         (0FFFFh-1662) ;C#,  277.183 Hz
                DW         (0FFFFh-1569) ;D,   293.665 Hz
                DW         (0FFFFh-1481) ;D#,  311.127 Hz
                DW         (0FFFFh-1398) ;E,   329.628 Hz
                DW         (0FFFFh-1319) ;F,   349.228 Hz
                DW         (0FFFFh-1245) ;F#,  369.994 Hz
                DW         (0FFFFh-1176) ;G,   391.995 Hz
                DW         (0FFFFh-1110) ;G#,  415.305 Hz

                ;Octave 2 ****************************************
                DW         (0FFFFh-1047) ;A,   440.000 Hz
                DW         (0FFFFh-988)  ;A#,  466.164 Hz
                DW         (0FFFFh-933)  ;B,   493.883 Hz
                DW         (0FFFFh-881)  ;C,   523.251 Hz
                DW         (0FFFFh-831)  ;C#,  554.365 Hz
                DW         (0FFFFh-785)  ;D,   587.330 Hz
                DW         (0FFFFh-741)  ;D#,  622.254 Hz
                DW         (0FFFFh-699)  ;E,   659.255 Hz
                DW         (0FFFFh-660)  ;F,   698.456 Hz
                DW         (0FFFFh-623)  ;F#,  739.989 Hz
                DW         (0FFFFh-588)  ;G,   783.989 Hz
                DW         (0FFFFh-555)  ;G#,  830.609 Hz

                ;Octave 3 ****************************************
                DW         (0FFFFh-524)  ;A,   880.000 Hz
                DW         (0FFFFh-494)  ;A#,  932.328 Hz
                DW         (0FFFFh-467)  ;B,   987.767 Hz
                DW         (0FFFFh-440)  ;C,  1046.502 Hz
                DW         (0FFFFh-416)  ;C#, 1108.731 Hz
                DW         (0FFFFh-392)  ;D,  1174.659 Hz
                DW         (0FFFFh-370)  ;D#, 1244.508 Hz
                DW         (0FFFFh-349)  ;E,  1318.510 Hz
                DW         (0FFFFh-330)  ;F,  1396.913 Hz
                DW         (0FFFFh-311)  ;F#, 1479.978 Hz
                DW         (0FFFFh-294)  ;G,  1567.982 Hz
                DW         (0FFFFh-277)  ;G#, 1661.219 Hz

                ;Rest Frequency ***************************************
                DW         (0FFFFh-1047) ;Rest. This value is not
                                         ; actually used as a
                                         ; timer 2 value.

                END
