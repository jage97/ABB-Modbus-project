/*
 ===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
 ===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "word.h"
#include "I2C.h"
#include "crc16.h"
#include "ModbusMaster.h"
#include "LpcUart.h"
#include "ModbusRegister.h"
#include "board.h"
#include "LiquidCrystal.h"
#endif
#endif
// TODO: insert other include files here
#include "DigitalIoPin.h"
#include "Wrapper.h"
#define I2C_CLK_DIVIDER (40)
#define I2C_BITRATE (100000)
#define I2C_MODE    (0)
#define ADC_DR_RESULT(n) ((((n) >> 4) & 0xFFF))
#define TICKRATE_HZ1 (1000)
#define I2C_ADDR_7BIT  (0x40)
#define READ_VALUE (0xF1)
// TODO: insert other include files here
#include <atomic>
static I2CM_XFER_T i2cmXferRec;
static volatile std::atomic_int counter;
static volatile uint32_t systicks;
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Handle interrupt from SysTick timer
 * @return Nothing
 */
void SysTick_Handler(void) {
	systicks++;
	if (counter > 0)
		counter--;
}
#ifdef __cplusplus
}
#endif
void Sleep(int ms) {
	counter = ms;
	while (counter > 0) {
		__WFI();
	}
}

static void setupI2CMaster() {
	Chip_I2C_Init(LPC_I2C0);
	Chip_I2C_SetClockDiv(LPC_I2C0, I2C_CLK_DIVIDER);
	Chip_I2CM_SetBusSpeed(LPC_I2C0, I2C_BITRATE);
	Chip_I2CM_Enable(LPC_I2C0);
}
static void SetupXferRecAndExecute(uint8_t devAddr, uint8_t *txBuffPtr,
		uint16_t txSize, uint8_t *rxBuffPtr, uint16_t rxSize) {
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = devAddr;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = txSize;
	i2cmXferRec.rxSz = rxSize;
	i2cmXferRec.txBuff = txBuffPtr;
	i2cmXferRec.rxBuff = rxBuffPtr;
	Chip_I2CM_XferBlocking(LPC_I2C0, &i2cmXferRec);
}
float getPIDValue(float current, float target) {//Calculate the difference between the current and target pressures
	float error = target - current;				//Returns throttle value (pwn)
	float pwm = error / 120;
	if (pwm > 1)
		pwm = 1;
	else if (pwm < -1)
		pwm = -1;
	return pwm;
}
static int ReadDataI2CM() {		//Read data from the pressure sensor
	uint8_t data[3];
	uint8_t lm75TempRegisterAddress = READ_VALUE;
	Wrapper p;

	SetupXferRecAndExecute(I2C_ADDR_7BIT, &lm75TempRegisterAddress, 1, data, 3);
	if (i2cmXferRec.status == I2CM_STATUS_OK) {

		int16_t finaldata = ((int16_t) data[0] << 8) | (int16_t) data[1];

		finaldata = finaldata / 240;
		p.print2(finaldata);
		return finaldata;				//return the final data in pascals
	}
}

uint32_t millis() {
	return systicks;
}
bool setFrequency(ModbusMaster& node, uint16_t freq) {
	int result;
	int ctr;
	bool atSetpoint;
	const int delay = 500;

	ModbusRegister Frequency(&node, 1); // reference 1
	ModbusRegister StatusWord(&node, 3);

	Frequency = freq; // set motor frequency

	// wait until we reach set point or timeout occurs
	ctr = 0;
	atSetpoint = false;
	do {
		Sleep(delay);
		// read status word
		result = StatusWord;
		// check if we are at setpoint
		if (result >= 0 && (result & 0x0100))
			atSetpoint = true;
		ctr++;
	} while (ctr < 20 && !atSetpoint);

	return atSetpoint;
}
int main(void) {
	uint32_t sysTickRate;

#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
	NVIC_DisableIRQ(I2C0_IRQn);
#endif
#endif
	DigitalIoPin sw1(1, 3, true, true, true);
	DigitalIoPin sw3(0, 9, true, true, true);
	DigitalIoPin sw2(0, 10, true, true, true);
	Chip_Clock_SetSysTickClockDiv(1);
	DigitalIoPin d4(1, 8, false, false, false);
	DigitalIoPin d5(0, 5, false, false, false);
	DigitalIoPin d6(0, 6, false, false, false);
	DigitalIoPin d7(0, 7, false, false, false);
	DigitalIoPin en(1, 6, false, false, false);
	DigitalIoPin rs(0, 8, false, false, false);

	ModbusMaster node(2); // Create modbus object that connects to slave id 2
	node.begin(9600); // set transmission rate
	Chip_RIT_Init(LPC_RITIMER);
	ModbusRegister ControlWord(&node, 0);
	ModbusRegister StatusWord(&node, 3);
	ModbusRegister OutputFrequency(&node, 102);
	ModbusRegister Current(&node, 103);
	Chip_Clock_SetSysTickClockDiv(1);
	sysTickRate = Chip_Clock_GetSysTickClockRate();
	SysTick_Config(sysTickRate / TICKRATE_HZ1);
	Chip_I2CM_Enable(LPC_I2C0);
	setupI2CMaster();

	ControlWord = 0x0406;
	Sleep(1000);

	ControlWord = 0x047F;

	Sleep(1000);

	volatile static int i = 0;
	int a = 0;
	LiquidCrystal lcd(&rs, &en, &d4, &d5, &d6, &d7);
	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
	bool isSet = false;
	bool pressed = false;
	bool pressed2 = false;
	bool pressed3 = false;
	bool clearpressed = false;
	bool pressureSet = false;
	bool automaticMode = false;
	bool manualMode = false;
	float percent = 0;
	float targetValue = 0.0;
	float PIDValue = 0;
	Wrapper p;
	int timerPID = 0;

	while (1) {

		lcd.clear();
		lcd.setCursor(0, 1);
		lcd.print("2 MANUALMODE");
		lcd.setCursor(0, 0);
		lcd.print("1 AUTOMATICMODE");
		if (sw1.read()) {
			while (sw1.read()) {
			}
			automaticMode = true;
		}
		if (sw2.read()) {
			while (sw2.read()) {
			}
			manualMode = true;
		}

		while (automaticMode) {
			while (!pressureSet) {
				if (!clearpressed) {
					lcd.clear();
					lcd.setCursor(0, 0);
					lcd.print("Target pressure:");
					lcd.setCursor(0, 1);
					lcd.print("(0 - 120) ");
					char prr[4];
					itoa(targetValue, prr, 10);
					lcd.print(prr);
					clearpressed = true;
				}
				if (sw1.read()) {	//raise target pressure
					pressed = true;
				} else if (pressed) {
					clearpressed = false;
					pressed = false;
					if (targetValue >= 120)
						targetValue = 120;
					else {
						targetValue += 10.0;
					}
				}
				if (sw2.read()) {
					pressed2 = true;
				} else if (pressed2) {	//lower target pressure
					clearpressed = false;
					pressed2 = false;
					if (targetValue <= 0)
						targetValue = 0;
					else {
						targetValue -= 10.0;
					}
				}
				if (sw3.read()) {
					pressureSet = true;
				}
			}
			while (getPIDValue(ReadDataI2CM(), targetValue) != 0) {	//Read pressure and PID value
				a += 10000 * getPIDValue(ReadDataI2CM(), targetValue);//Update frequency with current PID value
				if (a > 20000)
					a = 20000;				//Max frequency for the fan is 20000
				if ((ReadDataI2CM() > targetValue
						&& ReadDataI2CM() - targetValue <= 5)
						|| (ReadDataI2CM() < targetValue
								&& targetValue - ReadDataI2CM() <= 5)) {
					timerPID = 0;
				}
				if (timerPID < 10) {//If the target pressure is not within 5 units in time print to LCD and go back to menu
					timerPID += 1;
					Sleep(10);
				} else {
					lcd.clear();
					lcd.setCursor(0, 0);
					lcd.print("Pressure not");
					lcd.setCursor(0, 1);
					lcd.print("reached in time");
					while (!sw1.read());
					Sleep(50);
					while (sw1.read());
					automaticMode = false;
					timerPID = 0;
					break;
				}
				char prr[10];
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Tvalue  Cvalue");
				itoa(targetValue, prr, 10);
				lcd.setCursor(0, 1);
				lcd.print(prr);
				lcd.setCursor(8, 1);
				itoa(ReadDataI2CM(), prr, 10);
				lcd.print(prr);
				setFrequency(node, a);
				Sleep(3000);
				if (sw3.read()) {		//exit automatic mode
					automaticMode = false;
					pressureSet = false;
					break;
				}
			}
		}

		while (manualMode) {
			node.readHoldingRegisters(102, 2);	//Heartbeat signal
			lcd.clear();
			percent = (a * 100 / 20000);
			lcd.setCursor(0, 1);
			lcd.print("Fan: ");
			char prr[10];
			itoa(percent, prr, 10);
			lcd.print(prr);
			lcd.print("%");
			lcd.setCursor(0, 0);
			lcd.print("Pre: ");
			itoa(ReadDataI2CM(), prr, 10);
			lcd.print(prr);
			if (sw1.read()) {
				pressed = true;

			} else if (pressed) {	//raise fan speed
				a += 1000;
				isSet = false;
				pressed = false;
			}
			if (sw2.read()) {
				pressed2 = true;

			} else if (pressed2) {	//lower fan speed
				a -= 1000;
				isSet = false;
				pressed2 = false;
			}
			if (sw3.read()) {
				pressed3 = true;
			} else if (pressed3) {	//exit manual mode
				pressed3 = false;
				manualMode = false;
				break;
			}
			if (a >= 20000)
				a = 20000;
			if (a <= 0)
				a = 0;
			if (isSet == false) {
				setFrequency(node, a);
				isSet = true;
				Sleep(2500);
			}
			Sleep(200);
		}

		Sleep(50);
		i++;
	}
	return 0;
}
