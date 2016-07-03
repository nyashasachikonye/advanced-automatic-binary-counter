@SCHKUZ002, MLKTSH012
    .syntax unified
    .global _start

    .equ RAM_END, 0x20002000
    .equ RAM_START, 0x20000000 						@ This is the address of the beginning of RAM

vectors:
    .word RAM_END                   @ 0x00: defines the reset value of the stack pointer
    .word _start + 1                @ 0x04: defines the reset vector, thereby specifying the first instruction.
    .word Default_Handler + 1       @ 0x08: NMI vector

    .word HardFault_Handler + 1     @ 0x0C: HardFault vector

    .word Default_Handler + 1       @ 0x10: reserved
    .word Default_Handler + 1       @ 0x14: reserved
    .word Default_Handler + 1       @ 0x18: reserved
    .word Default_Handler + 1       @ 0x1C: reserved
    .word Default_Handler + 1       @ 0x20: reserved
    .word Default_Handler + 1       @ 0x24: reserved
    .word Default_Handler + 1       @ 0x28: reserved
    .word Default_Handler + 1       @ 0x2C: SVCall vector
    .word Default_Handler + 1       @ 0x30: reserved
    .word Default_Handler + 1       @ 0x34: reserved
    .word Default_Handler + 1       @ 0x38: PendSV vector
    .word Default_Handler + 1       @ 0x3C: SysTick vector
    .word Default_Handler + 1       @ 0x40: WWDG vector
    .word Default_Handler + 1       @ 0x44: PVD_VDDIO2 vector
    .word Default_Handler + 1       @ 0x48: RTC vector
    .word Default_Handler + 1       @ 0x4C: FLASH vector
    .word Default_Handler + 1       @ 0x50: RCC_CRS vector
    .word Default_Handler + 1       @ 0x54: EXTI0_1 vector
    .word Default_Handler + 1       @ 0x58: EXTI2_3 vector
    .word Default_Handler + 1       @ 0x5C: EXTI4_15 vector
    .word Default_Handler + 1       @ 0x60: TSC vector
    .word Default_Handler + 1       @ 0x64: DMA_CH1 v
    .word Default_Handler + 1       @ 0x68: DMA_CH2_3 v
    .word Default_Handler + 1       @ 0x6C: DMA_CH[4:7] v
    .word Default_Handler + 1       @ 0x70: ADC_COMP v
    .word Default_Handler + 1       @ 0x74: TIM1_BRK_UP_TRG_COM v
    .word Default_Handler + 1       @ 0x78: TIM1_CC
    .word Default_Handler + 1       @ 0x7C: TIM2 vector
    .word Default_Handler + 1       @ 0x80: TIM3 vector

    .word TIM6_Handler + 1          @ 0x84: TIM6 vector

    .word Default_Handler + 1       @ 0x88: TIM7 vector (not implemented)
    .word Default_Handler + 1       @ 0x8C: TIM14 vector
    .word Default_Handler + 1       @ 0x90: TIM15 vector
    .word Default_Handler + 1       @ 0x94: TIM16 vector
    .word Default_Handler + 1       @ 0x98: TIM17 vector
    .word Default_Handler + 1       @ 0x9C: I2C1 vector
    .word Default_Handler + 1       @ 0xA0: I2C2 vector
    .word Default_Handler + 1       @ 0xA4: SPI1 vector
    .word Default_Handler + 1       @ 0xA8: SPI2 vector
    .word Default_Handler + 1       @ 0xAC: USART1 vector
    .word Default_Handler + 1       @ 0xB0: USART2 vector
    .word Default_Handler + 1       @ 0xB4: USART3_4 vector
    .word Default_Handler + 1       @ 0xB8: CEC_CAN vector
    .word Default_Handler + 1       @ 0xBC: USB vector
    .word Default_Handler + 1       @ 0x98: USART2 vector

Default_Handler:
    @ The following block contains the routine that the program will run for any unexpected errors : INFINTE LOOP
    NOP
    B Default_Handler

HardFault_Handler:
    @ The following block contains the routine that the program will run for any hARD_FAULTS : INFINTE LOOP
    NOP
    B HardFault_Handler

_start:
    @ Pick some fixed memory location (first word of RAM seems good!) to hold
    @ the offset of the pattern being displayed. Initialise this offset to 0.
    
    @ DO NOT MODIFY THE MAIN LOOP. Your program must sit in an infinite main loop doing nothing.
    @ failure to comply with this will result in you receiving 0 marks.




	@ The following block clocks the ADC on the RCC_APB2ENR
	LDR R0, RCC_BASE								@ R0 = RCC Base Address
	LDR R1, ADC_CLOCK								@ R1 = Pattern to clock ADC
	STR R1, [R0,#0x18]								@ Write ADC Clock Pattern to RCC_APB2ENR

	@ The following block clocks the GPIOB (LED Lights) and GPIOA (Push Buttons & POTs) on the RCC_AHBENR
	LDR R0, RCC_BASE                                @ R0 = RCC base address
	LDR R1, [R0,#0x14]								@ R1 = RCC_AHBENR			
	LDR R2, GPIOA_GPIOB_CLOCK						@ R2 = Pattern to clock Port A & Port B
	ORRS R1, R1, R2 								@ Force BITS 17:18 (IOAEN, IOBEN) CLOCK
	STR R1, [R0,#0x14]								@ Write back to RCC_AHBENR

    @ The following block clocks the TIM6 on the RCC_APB1ENR
    LDR R0, RCC_BASE                                @ R0 = RCC base address
	LDR R1, [R0, #0x1C]                             @ RCC_APB1ENR
    LDR R2, TIM6_ENABLE                             @ R2 = Pattern to CLOCK TIM6
    ORRS R1, R1, R2                                 @ Force BIT 4 (TIM6EN)to ENABLE 1
    STR R1, [R0, #0x1C]                             @ write back to RCC_APB1ENR


	@ The following block sets the MODE of the GPIOB (LED Lights) to OUTPUT on the GPIOB_MODER
	LDR R0, GPIOB_BASE								@ R0 = GPIOB base address
	LDR R1, [R0, #0x00]								@ R1 = GPIOB_MODER
	LDR R2, LED_OUTPUT_MODE							@ R2 = Pattern to set LEDs to OUTPUT Mode
	ORRS R1, R1, R2									@ Force BITS 0:15 (MODER0:7)to OUTPUT (01)
	STR R1, [R0, #0x00]								@ Write back to GPIOB_MODER

	@ The following block sets the MODE of the GPIOA (POTs & SWITCHES) to INPUT | PA6,PA5 to ANALOG on the GPIOA_MODER
	LDR R0, GPIOA_BASE								@ R0 = GPIOA base address
	LDR R1, [R0, #0x00]								@ R1 = GPIOA_MODER
	LDR R2, PA5_PA6_ANALOG_MODE				 		@ Pattern to set all bits to be 00 (input) & PA5 (POT0) PA6 (POT1) to Analog (11)
	ORRS R1, R1, R2									@ Force BITS 10:13 (MODER5,MODER6) to OUTPUT (11)
	STR R1, [R0, #0x00]								@ Write back to GPIOA_MODER

	LDR R0, GPIOA_BASE								@ R0 = GPIOA base address
	LDR R1, [R0, #0x0C]								@ R1 = GPIOA_PUPDR
	LDR R2, SW0_SW1_PULLUP							@ Pattern to set SWITCH0:1 to INPUT
	ORRS R1, R1, R2									@ Force BITS 0:3 (PUPDR0:1) to PULLUP (01) 
	STR R1, [R0, #0x0C]								@ Write back to GPIOA_PUPDR

	@ The following block sets the RESOLUTION of the ADC to to 8 bits on the ADC_CFGR1
	LDR R0, ADC_BASE 								@ R0 = ADC Base Address
	LDR R1, [R0,#0x0C] 								@ R1 = ADC_CFGR1
	LDR R2, ADC_RES 								@ R2 = Pattern to set ADC RESOLUTION to 8 bits
	ORRS R1, R1, R2 								@ Force BITS 3:4 (RES) to 8BIT (10) 
	STR R1, [R0,#0x0C] 								@ Write back to ADC_CFGR1

	@ The following block internally enables the ADC. First clearing the configuration, then enabling the ADC
	LDR R0, ADC_BASE								@ R0 = ADC Base Address
	MOVS R1, 0x00 									@ R1 = 0 = Pattern to CLEAR the ADC Configuration before enabling
	STR R1, [R0,#0x08]								@ Write back to ADC_CR
	LDR R1, [R0,#0x08] 								@ R1 = ADC_CR
	LDR R2, ADC_ENABLE 								@ R2 = Pattern to ENABLE the ADC
	ORRS R1, R1, R2 								@ Force BIT 1 (ADEN) to ENABLE (1)
	STR R1, [R0,#0x08] 								@ Write back to ADC_CR

ADC_ENABLE_LOOP:
	@ The following block checks if the ADC has been enabled
	LDR R0, ADC_BASE 								@ R0 = ADC Base Address
	LDR R1, [R0,#0x00] 								@ R1 = ADC_ISR
	LDR R2, ADC_READY_CHECK 						@ R2 = Pattern to check if ADC is ready
	ANDS R2, R2, R1									@ BIT MASK: R2 = ADRDY
	CMP R2, #1 										@ Check ADRDY. HIGH = ADC ENABLED
	BNE ADC_ENABLE_LOOP 							@ Loop if ADRDY = LOW = ADC DISABLED

    @ The following block initialises the TIMER = TIM6
    @TIM6 INITIALISATION: SET PRESCALAR = 780
    LDR R0, TIM6_BASE
    LDR R1, PRESCALAR_VALUE
    STR R1, [R0, #0x28]                             @ Write back to TIM6_PSC

    @ TIM6 INITIALISATION: SET ARR = 10238
    LDR R0, TIM6_BASE
    LDR R1, SINGLE_DELAY_TIME
    STR R1, [R0, #0x2C]                             @ Write back to TIM6_ARR

    @ TIM6 INITIALISATION: ENABLE INTERRUPT REQUEST
    LDR R0, TIM6_BASE
    LDR R1, [R0, #0x0C]                             @ TIM6_DIER
    LDR R2, INTERRUPT_REQ_ENABLE                    @ R2 = Pattern to ENABLE INTERRUPT REQUESTS
    ORRS R1, R1, R2                                 @ Force BIT 0 (UIE) to ENABLE (1)
    STR R1, [R0, #0x0C]                             @ Write back to the TIM6_DIER

    @ TIM6 INITIALISATION: ENABLE COUNT
    LDR R0, TIM6_BASE
    LDR R1, [R0, #0x00]                             @ R1 = TIM6_CR1
    LDR R2, COUNT_ENABLE                            @ R2 = Pattern to ENABLE the TIM6_COUNTER
    ORRS R1, R1, R2                                 @ Force BIT 0 (CEN) to ENABLE (1)
    STR R1, [R0, #0]                                @ Write back to TIM6_CR1

    @ TIM6 INITIALISATION: ENABLE INTTERUPT PROCEDURE NVIC
    LDR R0, NVIC_ISER
    LDR R1, [R0, #0]                                @ R1 = NVIC_ISER
    LDR R2, NVIC_TIM6_INTRPT_ENABLE                 @ R2 = Pattern to ENABLE TIM6 INTERRUPTS
    ORRS R1, R1, R2                                 @ Force BIT 17 (CEN) to ENABLE (1)
    STR R1, [R0, #0]                                @ Write back to NVIC_ISER

    @ The following block sets the counter to zero in RAM
    LDR R0, =RAM_START
    MOVS R2, #0x00
    STR R2, [R0,#0x00]


@ END: INITIALISATION	
@+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ INITIALISATION BLOCK +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



main_loop:
    NOP
    B main_loop

TIM6_Handler:
    @ The following block ACKNOWLEDGES the INTERRUPT
    PUSH {LR}
    LDR R0, TIM6_BASE
    LDR R1, INTERRUPT_ACKNOWLEDGE                     @ R1 = Pattern to ACKNOWLEDGE INTTERUPT
    STR R1, [R0, #0x10]                               @ Write back to TIM6_SR

    BL RESET_TIMER

    BL CHECK_SWO
    BL CHECK_SW1

    @ If SW0 held:
    @ - set the ARR such that the IRQ will be triggered every 2 seconds
    @ Else if SW0 not held:
    @ - set the ARR such that the IRQ will be triggered every 1 second

    @ If SW1 held:
    @ - fetch offset from some location in memory
    @ - increment offset, wrapping to 0 if necessary
    @ - write updated offset back to memory
    @ - fetch pattern from address of PATTERNS + offset
    @ - write pattern to LEDs
    @ Else, if SW0 not held:
    @ - fetch current value on LEDs
    @ - increment by 1
    @ - write back

    @ The following block returns from the ISR

TIM6_RETURN:
    POP {PC}                              @ return from ISR


@================================================= SWITCH SW0 =================================================
CHECK_SWO:
    @ The following block runs the routine when SWITCH 0 is pressed
    PUSH {LR}
	LDR R0, GPIOA_BASE								@ R0 = GPIOA base address
	LDR R1, [R0, #0x10]								@ R7 = GPIOA_IDR
	LDR R2, =0b0000000000000001						@ R5 = Switch press pattern (SW1 pressed)
	ANDS R2, R2, R1									@ Set button press indicator (R7 = 2 = no_press, R7 = 0 = press)
	CMP R2, #0 										@ Compare R7 to press value (0)
	BEQ SW0_press 									@ Branch to SW1_press routine
	CMP R2, #1										@ Compare R7 to no press value (2)
	BEQ	SW0_nopress									@ Branch to SW1_nopress routine

SW0_nopress:
    B SINGLE_DELAY

SW0_press:
    B DOUBLE_DELAY

SINGLE_DELAY:
    B SWO_RETURN1

DOUBLE_DELAY:
    LDR R0, TIM6_BASE
    LDR R1, DOUBLE_DELAY_TIME
    STR R1, [R0, #0x2C]                             @ Write back to TIM6_ARR
    B SWO_RETURN1

SWO_RETURN1:
    POP {PC}

@================================================= SWITCH SW0 =================================================


@================================================= SWITCH SW1 =================================================
CHECK_SW1:
    @ The following block runs the routine when SWITCH 0 is pressed
    PUSH {LR}
	LDR R0, GPIOA_BASE								@ R0 = GPIOA base address
	LDR R1, [R0, #0x10]								@ R7 = GPIOA_IDR
	LDR R2, =0b0000000000000010						@ R5 = Switch press pattern (SW1 pressed)
	ANDS R2, R2, R1									@ Set button press indicator (R7 = 2 = no_press, R7 = 0 = press)
	CMP R2, #0 										@ Compare R7 to press value (0)
	BEQ SW1_press 									@ Branch to SW1_press routine
	CMP R2, #2										@ Compare R7 to no press value (2)
	BEQ	SW1_nopress									@ Branch to SW1_nopress routine

SW1_nopress:
    @ Read GPIOB_ODR (remember, we're only interested in the LSB!)
    LDR R0, GPIOB_BASE
    LDRB R1, [R0, #0x14]                @ R1 = GPIOB_ODR
    ADDS R1, R1, #1                     @ increment ODR by 1
    B SW1_RETURN1

SW1_press:
    LDR R0, GPIOB_BASE
    LDRB R1, [R0, #0x14]                @ R1 = GPIOB_ODR
    BL PATTERN_LOAD
    B SW1_RETURN1
    @B CHECK_SW1

PATTERN_LOAD:
    PUSH {LR}
    LDR R3, DATA_START
    LDR R4, =RAM_START
    LDR R2, [R4,#0x00]
    CMP R2, #0x08
    BEQ COUNT_WRAP
    LDRB R1, [R3,R2]
    ADDS R2, R2, #0x01
    STR R2, [R4,#0x00]
    POP {PC}

COUNT_WRAP:
    LDR R2, =0x00
    LDRB R1, [R3,R2]
    ADDS R2, R2, #0x01
    STR R2, [R4,#0x00]
    POP {PC}


SW1_RETURN1:
    STRB R1, [R0, #0x14]
    POP {PC}

@================================================= SWITCH SW 1=================================================

RESET_TIMER:
    @ The following block resets the timer for TIM6
    PUSH {LR}
    LDR R0, TIM6_BASE
    LDR R1, SINGLE_DELAY_TIME
    STR R1, [R0, #0x2C]                             @ Write back to TIM6_ARR
    POP {PC}

    .align
@ The following block defines the base addressses (ADDRESS) required for use in the program
RCC_BASE: .word 0x40021000
GPIOA_BASE: .word 0x48000000
GPIOB_BASE: .word 0x48000400
ADC_BASE: .word 0x40012400
TIM6_BASE: .word 0x40001000
NVIC_ISER: .word 0xE000E100

@ The following defines the configurations (CONFIGS) required for use in the program
ADC_CLOCK: .word 0x200								@ Pattern to clock the ADC (BIT 9 = ADCEN = 1)
GPIOA_GPIOB_CLOCK: .word 0x60000					@ Pattern to clock GPIOA/GPIOB (BIT 17:18 = 1)
TIM6_ENABLE: .word 0x10                             @ Pattern to clock the TIM6
INTERRUPT_REQ_ENABLE: .word 0x01                    @ Pattern to enable the TIM6 INTERRUPTS
COUNT_ENABLE: .word 0x01                            @ Pattern to enable the TIM6 counter
NVIC_TIM6_INTRPT_ENABLE: .word 0x20000              @ Pattern to enable TIM6 INTERRUPTS
INTERRUPT_ACKNOWLEDGE: .word 0x00                   @ Pattern to ACKNOWLEDGE INTERRUPT
LED_OUTPUT_MODE: .word 0x5555						@ Pattern to set LEDs to OUTPUT MODE (BIT 0:15 = 01)
PA5_PA6_ANALOG_MODE: .word 0x3C00 					@ Pattern to set PA6/PA5, (POT0 & POT1) to ANALOG MODE (BIT 10:13 = 11)
ADC_RES: .word 0x10
ADC_ENABLE: .word 0x1
ADC_READY_CHECK: .word 0x1
CH5_SELECT: .word 0x20
CH6_SELECT: .word 0x40
CH16_SELECT: .word 0x10000
SW0_SW1_PULLUP: .word 0x5
LED_CLEAR: .word 0x0
MAXIMUM: .word 0xFF
SW0_PRESS_VALUE: .word 0x1
SW1_PRESS_VALUE: .word 0x2
ADC_EOC_CHECK: .word 0x4
ADC_START_CONV: .word 0x4
TIM_OFFSET: .word 0x4E200
TIM_GRAD: .word 0x139B
TSENS_ENABLE: .word 0x800000
DELAY_TIME: .word 0x24C4A6
SINGLE_DELAY_TIME: .word 0x27FE
DOUBLE_DELAY_TIME: .word 0x4FFC
PRESCALAR_VALUE: .word 0x30C
DATA_START: .word PATTERNS


@ This is the sequence of patterns (each byte is a pattern) which the LEDs must cycle through. 
@ The values of the following words will be modified at compile time by the marker, but the number 
@ of values will not change. Hence, you can hard-code your bounds checking condition if you wish.
PATTERNS:
    .word 0xBBAA5500
    .word 0xFFEEDDCC
