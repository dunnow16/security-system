/*****************************************************
 * Names: David Bizzocchi & Owen Dunn
 * Project: Security System
 * Class: EGR 326
 * Professor: Dr. Bossemeyer
 * Aspects: SMCLK @ 12 MHz
 * 			MCLK @ 48 MHz
 *
 * 	-see solenoid lecture (high current actuators slide)
 * 	change period to effect frequency only
 * 	no need for PWM
 ****************************************************/

#include "driverlib.h"
#include "msp.h"
#include "ST7735.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BLUE_PIN BIT3; // blue motor control line
#define PINK_PIN BIT0;
#define YELLOW_PIN BIT7;
#define ORANGE_PIN BIT7;
#define RTC_Address 0x68 // 0b01101000
static volatile uint8_t seconds, minutes, hours, day, date, month, year, offset,
		tempMSB, tempLSB, state[3], InterruptIterations, BuzzerIterations;
static volatile double temp;
static volatile char refChar, outputChar, attemptCode[4], password[4] = { '1',
		'2', '3', '4' };
static volatile bool AlarmArmed, AlarmTriggered, isWindowOpen, isDoorOpen,
		isDoorLocked, MotionDetected, BuzzerShortDelay;

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig = { EUSCI_B_I2C_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
		12000000, // SMCLK = 12MHz
		EUSCI_B_I2C_SET_DATA_RATE_400KBPS, // Desired I2C Clock of 400khz
		0, // No byte counter threshold
		EUSCI_B_I2C_NO_AUTO_STOP // No Autostop
		};

/* UART Configuration Parameter */
const eUSCI_UART_Config uartConfig = { EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
		312,                                     // BRDIV
		0,                                     	 // UCxBRF
		0,                                     	 // UCxBRS
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Mode
		};

/* Initializes rows as inputs */
void KeypadRowInit() {
	P4DIR &= BIT0;
	P4DIR &= BIT1;
	P4DIR &= BIT2;
	P4DIR &= BIT3;
}

/* Initializes an external pull-up push button (using P5.4) */
void MenuButtonInit() {
	P5SEL0 &= ~BIT4;
	P5SEL1 &= ~BIT4; // 1) configure P5.4 GPIO
	P5DIR &= ~BIT4; // 2) make P5.4 input
	P5REN |= BIT4; // 3) enable pull resistors on P5.4
	P5OUT |= BIT4; // P5.4 is pull-up
}

/* Sets all used bits on pin layout to 0 */
void ResetPins() {
	P2OUT &= ~BLUE_PIN
	;
	P3OUT &= ~PINK_PIN
	;
	P1OUT &= ~YELLOW_PIN
	;
	P2OUT &= ~ORANGE_PIN
	;
}

/* Configure motor GPIO pins */
void SetDriverPins(void) {
	P2DIR |= BLUE_PIN
	; /* Configure P2.3 as output */
	P3DIR |= PINK_PIN
	; /* Configure P2.5 as output */
	P1DIR |= YELLOW_PIN
	; /* Configure P2.6 as output */
	P2DIR |= ORANGE_PIN
	; /* Configure P2.7 as output */
}

/* Initialize LED locked and unlocked indicators */
void StatusLEDInit() {
	P2DIR |= BIT0; /* Configure P2.0 as output */
	P2DIR |= BIT1; /* Configure P2.1 as output */

	/* Turn LEDs off initially */
	P2OUT &= BIT0;
	P2OUT &= BIT1;
}

/* Initialize Alarm LED */
void AlarmLEDInit() {
	P1DIR |= BIT0; /* Configure P1.0 as output */

	P1OUT & BIT0; /* Turn LED off initially */
}

/* Initialize Piezo Alarm Buzzer */
void BuzzerInit() {
	P5DIR |= BIT1; /* Configure P1.0 as output */
	P5OUT &= BIT1; /* Turn Buzzer off initially */
}

/* Initializes MSP432 SysTick timer */
void SysTick_Init(void) {
	SysTick->CTRL = 0;
	SysTick->LOAD = 0x00FFFFFF;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x00000005;
}

/* Initializes Hall effect sensor pins */
void SensorInit() {
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN6); //set P3.6 as output
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN7); //set P3.7 as output
}

/* sets the clock module to use external 48 MHz crystal */
void Clock_Init48MHz(void) {
	/* Configuring pins for peripheral/crystal usage */
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
	GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
	CS_setExternalClockSourceFrequency(32000, 48000000); // enables getMCLK,
//getSMCLK to know externally set frequencies
	/* Starting HFXT in non-bypass mode without a timeout. Before we start
	 * we have to change VCORE to 1 to support the 48MHz frequency */
	MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
	MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
	MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
	CS_startHFXT(false); // false means that there are no timeouts set,
//will return when stable
	/* Initializing MCLK to HFXT (effectively 48MHz) */
	MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

/* Initialize i2c data transmission */
void I2C_Init() {
// Select Port 6 for I2C
// Set Pin 4, 5 to input Primary Module Function,
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
	GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Initializing I2C Master */
	MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);
	/* Specify slave address */
	MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, RTC_Address);
	/* Set Master in transmit mode */
	MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
	/* Enable I2C Module to start operations */
	MAP_I2C_enableModule(EUSCI_B1_BASE);
}

/* Initialize update and alarm interrupts */
void InterruptInit() {
	/* Configuring Timer32 to 240000000 (1s) of MCLK in periodic mode */
	MAP_Timer32_initModule(TIMER32_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
	TIMER32_PERIODIC_MODE);
	MAP_Timer32_setCount(TIMER32_BASE, 240000000);

	/* Enabling interrupts */
	//MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
	MAP_Interrupt_enableInterrupt(INT_PORT1);
	MAP_Interrupt_enableInterrupt(INT_T32_INT1);

	/* Starting the Timer */
	MAP_Timer32_enableInterrupt(TIMER32_BASE);
	MAP_Timer32_startTimer(TIMER32_BASE, true);
}

/* Initialize Watch Dog Timer */
void WDTInit() {
	// Halting the Watchdog (while we set it up)
	MAP_WDT_A_holdTimer();

	// set ACLK to REF0
	MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

	// Initialize WDT for ~3.5 sec
	WDT_A_initWatchdogTimer(WDT_A_CLOCKSOURCE_ACLK, WDT_A_CLOCKITERATIONS_512K);

	// Set reset on watchdog timeout
	WDT_A_setTimeoutReset(WDT_A_SOFT_RESET);

	// Start WDT
	WDT_A_startTimer();
}

/* Toggles Alarm LED */
void ToggleAlarmLED() {
	P1DIR ^= BIT0;
}

/* Toggles Piezo Buzzer */
void ToggleBuzzer() {
	P5DIR ^= BIT1;
}

/* Creates delay using SysTick timer */
// -adjusted for 48MHz clock
void SysTick_Delay(uint16_t delay) {
	uint32_t temp = ((delay * 48000) - 1);
	SysTick->LOAD = temp;
	SysTick->VAL = 0;

	// Wait for flag to be SET (Timeout happened)
	while ((SysTick->CTRL & 0x00010000) == 0)
		;
}

/* Reads input for each row */
uint8_t DebouncedReadInputRows() {
	__delay_cycles(10); 	//delay 3.3 us to allow column to settle
	if (((P4IN & BIT0 ) >> 0) == 0) {	//shift bit to read appropriately
		return 1;
	}
	__delay_cycles(10); 	//delay 3.3 us to allow column to settle
	if (((P4IN & BIT1 ) >> 1) == 0) {
		return 2;
	}
	__delay_cycles(10); 	//delay 3.3 us to allow column to settle
	if (((P4IN & BIT2 ) >> 2) == 0) {
		return 3;
	}
	__delay_cycles(10); 	//delay 3.3 us to allow column to settle
	if (((P4IN & BIT3 ) >> 3) == 0) {
		return 4;
	}
	return 0;
}

/* Cycles through Full Step motor rotations */
void FullStep(int step) {
	switch (step) {
	case 0:
		ResetPins();
		P2OUT |= BLUE_PIN
		;
		P2OUT |= ORANGE_PIN
		;
		break;
	case 1:
		ResetPins();
		P2OUT |= BLUE_PIN
		;
		P3OUT |= PINK_PIN
		;
		break;
	case 2:
		ResetPins();
		P3OUT |= PINK_PIN
		;
		P1OUT |= YELLOW_PIN
		;
		break;
	case 3:
		ResetPins();
		P1OUT |= YELLOW_PIN
		;
		P2OUT |= ORANGE_PIN
		;
		break;
	}
}

/* Sets all row values to zero */
ResetKeypadStates() {
	int index;
	for (index = 0; index <= 3; index++) {
		state[index] = 0;
	}
}

/* Reads in keys, scans columns individually */
void scanKeys() {
	ResetKeypadStates();
	int index = 0;	// index for state array

	/* Scan column C0 */
	P4DIR |= BIT4;		// set column C0 to output
	P4OUT &= ~BIT4; 	// set column C0 to LOW
	__delay_cycles(10); 	//delay 3.3 us to allow column to settle
	state[index] = DebouncedReadInputRows();	// read all row pins
	P4DIR &= ~BIT4;		// set column C0 back to input
	index++;

	/* Scan column C1 */
	P4DIR |= BIT5;		// set column C1 to output
	P4OUT &= ~BIT5; 	// set column C1 to LOW
	__delay_cycles(10); 	//delay 3.3 us to allow column to settle
	state[index] = DebouncedReadInputRows(); // read all row pins
	P4DIR &= ~BIT5;		// set column C1 back to input
	index++;

	/* Scan column C2 */
	P4DIR |= BIT6;		// set column C2 to output
	P4OUT &= ~BIT6; 	// set column C2 to LOW
	__delay_cycles(10); 	//delay 3.3 us to allow column to settle
	state[index] = DebouncedReadInputRows();	// read all row pins
	P4DIR &= ~BIT6;		// set column C2 back to input
	index++;

	if (++index >= 3) {
		index = 0;		//reset buffer
	}
}

/* determines button pressed based on input array */
void DetermineChar() {
	if (outputChar != ' ') {
		refChar = outputChar;	// change reference if non empty input detected
	}
	if ((state[0] == 0) && (state[1] == 0) && (state[2] == 0)) {
		refChar = ' ';			// reset ref character if no imput found
	}

	if (state[0] != 0) {	// If column 1 pressed
		switch (state[0]) {
		case 1:
			outputChar = '1';
			break;
		case 2:
			outputChar = '4';
			break;
		case 3:
			outputChar = '7';
			break;
		case 4:
			outputChar = '*';
			break;
		}
	} else if (state[1] != 0) {	// If column 2 pressed
		switch (state[1]) {
		case 1:
			outputChar = '2';
			break;
		case 2:
			outputChar = '5';
			break;
		case 3:
			outputChar = '8';
			break;
		case 4:
			outputChar = '0';
			break;
		}
	} else if (state[2] != 0) {	// If column 3 pressed
		switch (state[2]) {
		case 1:
			outputChar = '3';
			break;
		case 2:
			outputChar = '6';
			break;
		case 3:
			outputChar = '9';
			break;
		case 4:
			outputChar = '#';
			break;
		}
	} else {	// if no input detected reset input char
		outputChar = ' ';
	}

	if (outputChar == refChar) {	// Determine if there is no change in input
		outputChar = ' ';			// Reset output if no change in input
	}
}

/* Determines if external switch has been pressed for a set amount of time */
bool MenuButtonPressed() {
	static uint16_t state = 0;
	state = (state << 1) | (P5IN & BIT4 ) >> 1 | 0xf800;
	if (state == 0xfc00) {
		return true;
	}
	return false;
}

void DisarmAlarm() {
	AlarmTriggered = false;
	P2OUT |= BIT0; 	// Turn on red LED
	P2OUT &= ~BIT1; 	// Turn off green LED
	P1OUT &= ~BIT0; 	// Turn off alarm LED
	P5OUT &= ~BIT1;		// Turn off Buzzer
	BuzzerIterations = 0;	//reset buzzer iterations
}

void ArmAlarm() {
	P2OUT |= BIT1; 	// Turn on green LED
	P2OUT &= ~BIT0; 	// Turn off red LED
}

void ToggleAlarmState() {
	if (AlarmArmed == true) {
		AlarmArmed = false;
		DisarmAlarm();
	} else {
		AlarmArmed = true;
		ArmAlarm();
	}
}

void LCDShowDetected() {
	int x = 35, y = 115;

	ST7735_FillRect(27, 110, 106, 20, ST7735_WHITE);
	ST7735_FillRect(30, 112, 100, 15, ST7735_RED);
	ST7735_DrawCharS(x, y, 'M', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'O', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'T', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'I', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'O', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'N', ST7735_WHITE, ST7735_RED, 1);
	x = x + 12;
	ST7735_DrawCharS(x, y, 'D', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'E', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'T', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'E', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'C', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'T', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'E', ST7735_WHITE, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'D', ST7735_WHITE, ST7735_RED, 1);
}

void ReadPIRSensors() {
	int test = GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN5);

	if (test == 1 && !MotionDetected) {
		MotionDetected = true;
		LCDShowDetected();
	} else if (test == 0 && MotionDetected) {
		MotionDetected = false;
		ST7735_FillRect(27, 110, 106, 20, ST7735_BLACK); //Clears motion detected warning
	}
}

/* Writes date time year information to RTC */
void WriteRTC() {
	uint8_t bcdSec = 0x00;
	uint8_t bcdMin = 0x00;
	uint8_t bcdHour = 0x12;
	uint8_t bcdDay = 0x01;
	uint8_t bcdDate = 0x31;
	uint8_t bcdMonth = 0x10;
	uint8_t bcdYear = 0x16;

	//Set Master in transmit mode
	MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
	//Wait for bus release, ready to write
	while (MAP_I2C_isBusBusy(EUSCI_B1_BASE))
		;
	//set pointer to beginning of RTC registers
	MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, 0x00);
	//write to seconds register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdSec);
	//write to minutes register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdMin);
	//write to hours register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdHour);
	//write to day register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdDay);
	//write to day register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdDate);
	//write to months register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdMonth);
	//write to year register and send stop
	MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, bcdYear);
}

/* Converts input temperature to readable format */
ConvertTemp() {
	if ((tempMSB & 0x80) != 0) { 	// check for negative flag (MSB = 1)
		/* Convert Result */
		tempMSB ^= 0xff;
		tempMSB += 0x1;
		temp = tempMSB + ((tempLSB >> 6) * 0.25);
		temp = temp * -1;
	} else {
		temp = tempMSB + ((tempLSB >> 6) * 0.25);
	}
}

/* Reads in DTY information from the RTC */
void ReadRTC() {
// Select Port 6 for I2C
// Set Pin 4, 5 to input Primary Module Function,
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
	GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Initializing I2C Master */
	MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);
	/* Specify slave address */
	MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, RTC_Address);
	/* Set Master in transmit mode */
	MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
	/* Enable I2C Module to start operations */
	MAP_I2C_enableModule(EUSCI_B1_BASE);
// Set Master in transmit mode
	//MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
// Wait for bus release, ready to write
	while (MAP_I2C_isBusBusy(EUSCI_B1_BASE))
		;
// set pointer to beginning of RTC registers
	MAP_I2C_masterSendSingleByte(EUSCI_B1_BASE, 0);
// Wait for bus release
	while (MAP_I2C_isBusBusy(EUSCI_B1_BASE))
		;
// Set Master in receive mode
	MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE);
// Wait for bus release, ready to receive
	while (MAP_I2C_isBusBusy(EUSCI_B1_BASE))
		;
// read from RTC registers (pointer auto increments after each read)
	seconds = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	minutes = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	hours = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	day = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	date = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	month = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	year = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	offset = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	tempMSB = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
	tempLSB = (int) MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);

	ConvertTemp();
}

/* Obtains and returns combination of 2 key presses */
int8_t ObtainTwoDigitKeypad(int16_t y) {
	uint8_t digitA, digitB;
	while (1) {
		WDT_A_clearTimer();	// clear watch dog timer
		if (MenuButtonPressed()) {
			return -1;
		}
		scanKeys();		//read in keys
		DetermineChar();	// determines character based on key scanned in
		if (outputChar != ' ') {
			digitA = outputChar - '0';
			ST7735_DrawCharS(120, y, outputChar, ST7735_WHITE, ST7735_BLACK, 1);
			break;
		}
	}
	//outputChar = ' ';
	while (1) {
		WDT_A_clearTimer();	// clear watch dog timer
		if (MenuButtonPressed()) {
			return -1;
		}
		scanKeys();		//read in keys
		DetermineChar();	// determines character based on key scanned in
		if (outputChar != ' ') {
			digitB = outputChar - '0';
			ST7735_DrawCharS(126, y, outputChar, ST7735_WHITE, ST7735_BLACK, 1);
			break;
		}
	}
	return ((digitA * 10) + digitB);
}

/* Gathers input information to write to RTC */
void setDateTimeInfo() {
	ST7735_FillRect(0, 16, 160, 112, ST7735_BLACK);	//clear area below date time info

	// declare variable for info input
	int8_t currMin, currHour, currDate, currMonth, currYear;
	int16_t y = 30; 	// start point for pixel on vertical axis

	ST7735_DrawString(2, 3, "Enter Month (MM): ", ST7735_WHITE);
	currMonth = ObtainTwoDigitKeypad(y);
	if (currMonth == -1)
		return;
	currMonth = (currMonth / 10) * 16 + (currMonth % 10);	// conversion to bcd

	y = y + 10;
	ST7735_DrawString(2, 4, "Enter Date (DD): ", ST7735_WHITE);
	currDate = ObtainTwoDigitKeypad(y);
	if (currDate == -1)
		return;
	currDate = (currDate / 10) * 16 + (currDate % 10);

	y = y + 10;
	ST7735_DrawString(2, 5, "Enter Year (20YY): ", ST7735_WHITE);
	currYear = ObtainTwoDigitKeypad(y);
	if (currYear == -1)
		return;
	currYear = (currYear / 10) * 16 + (currYear % 10);

	y = y + 10;
	ST7735_DrawString(2, 6, "Enter Hour (HH): ", ST7735_WHITE);
	currHour = ObtainTwoDigitKeypad(y);
	if (currHour == -1)
		return;
	currHour = (currHour / 10) * 16 + (currHour % 10);

	y = y + 10;
	ST7735_DrawString(2, 7, "Enter Minute (MM): ", ST7735_WHITE);
	currMin = ObtainTwoDigitKeypad(y);
	if (currMin == -1)
		return;
	currMin = (currMin / 10) * 16 + (currMin % 10);

	//Set Master in transmit mode
	MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
	//Wait for bus release, ready to write
	while (MAP_I2C_isBusBusy(EUSCI_B1_BASE))
		;
	//set pointer to beginning of RTC registers
	MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, 0x00);
	//write to seconds register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, 0x00);
	//write to minutes register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, currMin);
	//write to hours register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, currHour);
	//write to day register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, 0x00);
	//write to day register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, currDate);
	//write to months register
	MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, currMonth);
	//write to year register and send stop
	MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, currYear);
}

void DisplayMenu() {
	ST7735_FillRect(0, 16, 160, 94, ST7735_BLACK);//clear area below date time info

	ST7735_DrawString(2, 4, "1: Set Date/Time Info", ST7735_WHITE);
	ST7735_DrawString(2, 5, "2: Arm/Disarm Alarm", ST7735_WHITE);
	ST7735_DrawString(2, 6, "3: View Sensor Status", ST7735_WHITE);
	ST7735_DrawString(2, 7, "4: Lock/Unlock Door", ST7735_WHITE);
	ST7735_DrawString(2, 8, "5: Change Password", ST7735_WHITE);
}

int BcdToDecimal(uint8_t hex) {
	int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
	return dec;
}

void UpdateDT() {
	int x = 3;

	ReadRTC();

	month = BcdToDecimal(month);
	ST7735_DrawCharS(x, 2, ((month / 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, ((month % 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, '/', ST7735_Color565(255, 255, 255), 0, 1);
	x = x + 6;
	date = BcdToDecimal(date);
	ST7735_DrawCharS(x, 2, ((date / 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, ((date % 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, '/', ST7735_Color565(255, 255, 255), 0, 1);
	x = x + 6;
	year = BcdToDecimal(year);
	ST7735_DrawCharS(x, 2, ((year / 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, ((year % 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);

	x = x + 30;
	hours = BcdToDecimal(hours);
	ST7735_DrawCharS(x, 2, ((hours / 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, ((hours % 10) + '0'), ST7735_Color565(255, 255, 255),
			0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, ':', ST7735_Color565(255, 255, 255), 0, 1);
	x = x + 6;
	minutes = BcdToDecimal(minutes);
	ST7735_DrawCharS(x, 2, ((minutes / 10) + '0'),
			ST7735_Color565(255, 255, 255), 0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, ((minutes % 10) + '0'),
			ST7735_Color565(255, 255, 255), 0, 1);

	temp = temp * 1.8 + 32;	// convert to fahrenheit
	x = x + 30;
	if (temp >= 110) {
		ArmAlarm();
		AlarmTriggered = true;
	}
	if (temp >= 100) {
		ST7735_DrawCharS(x, 2, ((temp / 100) + '0'),
				ST7735_Color565(255, 255, 255), 0, 1);
		x = x + 6;
		ST7735_DrawCharS(x, 2, (((temp / 10) - 10) + '0'),
				ST7735_Color565(255, 255, 255), 0, 1);
		x = x + 6;
	} else {
		ST7735_DrawCharS(x, 2, ((temp / 10) + '0'),
				ST7735_Color565(255, 255, 255), 0, 1);
		x = x + 6;
	}
	ST7735_DrawCharS(x, 2, (((int) temp % 10) + '0'),
			ST7735_Color565(255, 255, 255), 0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 0, 'o', ST7735_Color565(255, 255, 255), 0, 1);
	x = x + 6;
	ST7735_DrawCharS(x, 2, 'F', ST7735_Color565(255, 255, 255), 0, 1);
}

/* Checks each element in password arrays, returns true of matching */
bool VerifyPassword() {
	int i, matches = 0;
	for (i = 0; i < 4; i++) {
		if (attemptCode[i] == password[i]) {
			matches++;
		}
	}
	if (matches >= 4) {
		return true;
	}
	return false;
}

/* Shifts password one digit back */
ShiftPassCode(char input) {
	int i = 0;
	for (i = 1; i < 4; i++) {
		attemptCode[i - 1] = attemptCode[i];
	}
	attemptCode[3] = input;
}

/* Stores character in attempt password if number */
int StoreAttemptChar(char input, int passWordIndex) {
	if (input != '*') {
		attemptCode[passWordIndex] = input;
		passWordIndex++;
	}
	if (passWordIndex >= 5) {
		ShiftPassCode(input);
		return 4;
	}
	return passWordIndex;
}

/* Checks if password is correct, returns true if match */
bool PasswordCheck(char message[22]) {
	int passWordIndex = 0, x = 48;
	ST7735_FillRect(0, 16, 160, 94, ST7735_BLACK);//clear area below date time info
	ST7735_DrawString(3, 3, message, ST7735_WHITE);
	while (1) {
		WDT_A_clearTimer();	// clear watch dog timer
		if (MenuButtonPressed()) {
			return false;
		}
		scanKeys();			// read in and store keys
		DetermineChar();	// determines character based on key scanned in
		if (outputChar != ' ') {		// if input is determined
			passWordIndex = StoreAttemptChar(outputChar, passWordIndex);
			ST7735_DrawCharS(x, 70, outputChar, ST7735_WHITE, ST7735_BLACK, 2);	// print key input
			x += 15;
		}
		if (passWordIndex == 4) {
			break;
		}
	}
	ST7735_FillRect(0, 16, 160, 112, ST7735_BLACK);	//clear area below date time info
	return VerifyPassword();
}

/* Resets value for upward counting */
int CheckIndexForward(int index) {
	if (index > 3 || index < 0) {
		return 0;
	}
	return index;
}

/* Resets value for downward counting */
int CheckIndexBackward(int index) {
	if (index > 3 || index < 0) {
		return 3;
	}
	return index;
}

void LockDoor() {
	int step = 0, i = 0;
	/* Iterates forward through (half cycle x gear ratio) */
	for (i = 0; i < (32 * 64); i++) {
		step = CheckIndexForward(step);	//sets index to appropriate value
		FullStep(step);	//shift through steps
		++step;
		SysTick_Delay(2);
	}
	isDoorLocked = true;
}

void UnlockDoor() {
	int step = 0, i = 0;
	/* Iterates forward through (half cycle x gear ratio) */
	for (i = 0; i < (32 * 32); i++) {
		step = CheckIndexBackward(step);	//sets index to appropriate value
		FullStep(step);	//shift through steps
		--step;
		SysTick_Delay(2);
	}
	isDoorLocked = false;
}

void WriteClosed(int16_t x, int16_t y) {
	ST7735_DrawCharS(x, y, 'C', ST7735_BLACK, ST7735_GREEN, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'L', ST7735_BLACK, ST7735_GREEN, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'O', ST7735_BLACK, ST7735_GREEN, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'S', ST7735_BLACK, ST7735_GREEN, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'E', ST7735_BLACK, ST7735_GREEN, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'D', ST7735_BLACK, ST7735_GREEN, 1);
}

void WriteOpen(int16_t x, int16_t y) {
	ST7735_DrawCharS(x, y, 'O', ST7735_BLACK, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'P', ST7735_BLACK, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'E', ST7735_BLACK, ST7735_RED, 1);
	x = x + 6;
	ST7735_DrawCharS(x, y, 'N', ST7735_BLACK, ST7735_RED, 1);
}

/* Updates Sensor variables */
void CheckSensorStatus() {
	if ((GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN6)) == 1) {
		isDoorOpen = true;
	} else {
		isDoorOpen = false;
	}
	if ((GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7)) == 1) {
		isWindowOpen = true;
	} else {
		isWindowOpen = false;
	}
}

/* Updates Window and Door Variables */
void DisplaySensorStatus() {
	bool tempDoor, tempWindow;

	ST7735_FillRect(0, 16, 160, 112, ST7735_BLACK);	//clear area below date time info
	ST7735_DrawString(5, 2, "Door", ST7735_WHITE);
	ST7735_DrawFastVLine(79, 16, 112, ST7735_WHITE);//draw division between sensors
	ST7735_DrawFastVLine(80, 16, 112, ST7735_WHITE);//draw division between sensors
	ST7735_DrawString(18, 2, "Window", ST7735_WHITE);

	if (isDoorOpen == true) {
		ST7735_FillRect(0, 30, 79, 112, ST7735_RED);
		WriteOpen(30, 75);
	} else {
		ST7735_FillRect(0, 30, 79, 112, ST7735_GREEN);
		WriteClosed(24, 75);
	}
	if (isWindowOpen == true) {
		ST7735_FillRect(81, 30, 79, 112, ST7735_RED);
		WriteOpen(110, 75);
	} else {
		ST7735_FillRect(81, 30, 79, 112, ST7735_GREEN);
		WriteClosed(104, 75);
	}

	while (1) {
		WDT_A_clearTimer();	// clear watch dog timer
		if (MenuButtonPressed()) {
			return;
		}
		tempDoor = isDoorOpen;
		tempWindow = isWindowOpen;
		CheckSensorStatus();
		if (tempDoor != isDoorOpen) {
			if (isDoorOpen) {
				if (MotionDetected) {
					ST7735_FillRect(0, 30, 79, 80, ST7735_RED);
					ST7735_FillRect(0, 110, 27, 18, ST7735_RED);
				} else {
					ST7735_FillRect(0, 30, 79, 112, ST7735_RED);
				}
				WriteOpen(30, 75);
			} else {
				if (MotionDetected) {
					ST7735_FillRect(0, 30, 79, 80, ST7735_GREEN);
					ST7735_FillRect(0, 110, 27, 18, ST7735_GREEN);
				} else {
					ST7735_FillRect(0, 30, 79, 112, ST7735_GREEN);
				}
				WriteClosed(24, 75);
			}
		}
		if (tempWindow != isWindowOpen) {
			if (isWindowOpen) {
				if (MotionDetected) {
					ST7735_FillRect(81, 30, 79, 80, ST7735_RED);
					ST7735_FillRect(133, 110, 27, 18, ST7735_RED);
				} else {
					ST7735_FillRect(81, 30, 79, 112, ST7735_RED);
				}
				WriteOpen(110, 75);
			} else {
				if (MotionDetected) {
					ST7735_FillRect(81, 30, 79, 80, ST7735_GREEN);
					ST7735_FillRect(133, 110, 27, 18, ST7735_GREEN);
				} else {
					ST7735_FillRect(81, 30, 79, 112, ST7735_GREEN);
				}
				WriteClosed(104, 75);
			}
		}
	}
}

/* Changes current password in memory */
void ChangePassword() {
	int i;
	PasswordCheck("  Enter New Password");//get new attempt code, ignore result
	for (i = 0; i < 4; i++) {
		password[i] = attemptCode[i];		//set attempt code to password
	}
	ST7735_FillRect(0, 16, 160, 112, ST7735_BLACK);	//clear area below date time info
}

/* Timer32 ISR */
void T32_INT1_IRQHandler(void) {
	MAP_Timer32_clearInterruptFlag(TIMER32_BASE);
	if (AlarmArmed) {
		InterruptIterations++;
		if (isDoorOpen || isWindowOpen) {
			AlarmTriggered = true;
		}
		if (AlarmTriggered) {
			if (BuzzerIterations > 1000) {
				BuzzerShortDelay ?
						(BuzzerShortDelay = false) : (BuzzerShortDelay = true);
				BuzzerIterations = 0;
				BuzzerIterations++;
			}
			if (BuzzerShortDelay) {
				ToggleBuzzer();
			} else if (BuzzerIterations % 2 == 0) {
				ToggleBuzzer();
			}
		}
		MAP_Timer32_setCount(TIMER32_BASE, 48000);	//set 1ms interrupt
		if (InterruptIterations <= 5000) {
			return;
		}

	} else {
		MAP_Timer32_setCount(TIMER32_BASE, 240000000);	//set 5s interrupt
	}
	InterruptIterations = 0;
	UpdateDT();
	ReadPIRSensors();
}

int main(void) {
	AlarmArmed = false;
	isDoorLocked = false;
	WDT_A_holdTimer();

	/* Initialize delay timer */
	SysTick_Init();

	/* Initialize LCD */
	Clock_Init48MHz();
	MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);// set SMCLK to 12 MHz
	ST7735_InitR(INITR_REDTAB);
	ST7735_FillScreen(0x0000);            // set screen to black
	ST7735_SetRotation(3);				  // rotate screen 270 deg

	/* Initialize Keypad */
	KeypadRowInit();

	/* Initialize Interrupts */
	InterruptInit();
	InterruptIterations = 0;

	/* Initialize RTC */
	I2C_Init();
	WriteRTC();

	/* Initialize Alarm */
	StatusLEDInit();
	AlarmLEDInit();
	BuzzerInit();
	DisarmAlarm();

	/* Initialize Sensors */
	SensorInit();

	/* Initialize Menu Button */
	MenuButtonInit();

	/* Initialize Door Lock Motor */
	SetDriverPins();
	ResetPins();

	/* Initialize PIR Sensor */
	GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN5); //set P3.5 as input

	/* Initialize Watch Dog Timer */
	WDTInit();

	/* Display Home Screen */
	ST7735_DrawFastHLine(0, 14, 160, ST7735_WHITE);
	DisplayMenu();
	UpdateDT();

	while (1) {
		WDT_A_clearTimer();	// Clear watch dog timer
		scanKeys();			// read in and store keys
		DetermineChar();	// determines character based on key scanned in
		if (outputChar != ' ') {		// if input is determined
			switch (outputChar) {

			/* Set Date & Time Info */
			case '1':
				setDateTimeInfo();
				DisplayMenu();
				UpdateDT();
				break;

				/* Arm or Disarm Alarm */
			case '2':
				if (PasswordCheck("Enter 4 Digit Password")) {
					ToggleAlarmState();
				}
				DisplayMenu();
				break;

				/* Display Sensor Status' */
			case '3':
				DisplaySensorStatus();
				ST7735_FillScreen(0x0000);            // set screen to black
				DisplayMenu();
				break;

				/* Lock or Unlock Door */
			case '4':
				if (PasswordCheck("Enter 4 Digit Password")) {
					if (isDoorLocked) {
						UnlockDoor();
					} else {
						LockDoor();
					}
				}
				DisplayMenu();
				break;

				/* Change Password */
			case '5':
				if (PasswordCheck("Enter Current Password")) {
					ChangePassword();
				}
				DisplayMenu();
				break;
			}
		}
	}
}
