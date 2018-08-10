/*
* Mega Meter.c
*
* Created: 9/12/2017 5:17:58 PM
* Author : william
* Uses ATMega2560
*/

#include "config.h"

#include "HardwareSerial.h"
#include "PCD8544.h"

#define PACKET_SYNC 0xFF
#define PACKET_VER  4

PCD8544 lcd = PCD8544();

void randomSeed(unsigned long seed)
{
	if (seed != 0) srandom(seed);
}

long random(long howbig)
{
	if (howbig == 0) return 0;
	return random() % howbig;
}

long random(long howsmall, long howbig)
{
	if (howsmall >= howbig) return howsmall;
	long diff = howbig - howsmall;
	return random(diff) + howsmall;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Pin Mapping
// Port - Arduino	Description		Usage
// PF1 - A1			Output			Left Indicator
// PF2 - A2			Output			Right Indicator
// PF3 - A3			Output			High Beam
// PF4 - A4			Output			Lights On
// PF5 - A5			Output			Dash Backlights
// PF6 - A6			Output			Fog Lights
// PF7 - A7			Output			Smart Lock
// PK0 - A8			Output			Park Break
// PK1 - A9			Output			Break Fail
// PK2 - A10		Output			Rear Demist
// PK3 - A11		Output			Oil Light
// PK4 - A12		Output			Battery
// PK5 - A13		Output			Radiator
// PK6 - A14		Output			Air Bag
// PK7 - A15		Output			Check ABS
// PJ1 - D14		Output			Econ
// PJ0 - D15		Output			Cruise
// PH1 - D16		Output			Seat Belt
// PH0 - D17		Output			Fuel Low
// PD3 - D18		Output			High Temp
// PD2 - D19		Output			Alternator
// PA0 - D22		Output			Oil Pressure Low
// PA1 - D23		Output			Left Spare Light
// PA2 - D24		Output			Right Spare Light
// PA3 - D25		Output			Tick-Tock Relay
// PF0 - A0			ADC0			Testing Pot
// PB5 - D11		OCR1A			Speed Sin +
// PB6 - D12		OCR1B			Speed Sin -
// PB7 - D13		OCR1C			Fuel Gauge
// PE3 - D5			OCR3A			Speed Cos +
// PE4 - D2			OCR3B			Speed Cos -
// PE5 - D3			OCR3C			Oil Pressure
// PH3 - D6			OCR4A			Tacho Sin +
// PH4 - D7			OCR4B			Tacho Sin -
// PH5 - D8			OCR4C			Volt Gauge
// PL3 - D46		OCR5A			Tacho Cos +
// PL4 - D45		OCR5B			Tacho Cos -
// PL5 - D44		OCR5C			Temp Gauge
// PL1 - D48		Output			LCD Reset
// PL0 - D49		Output			LCD Data/Command
// PB3 - D50		MOSI			LCD SI
// PB1 - D52		SCK				LCD SCK

void loop();
void setTacho(uint16_t value);
void setSpeed(uint16_t value);

int main(void)
{
	// Enable Global Interrupts
	sei();
	
	// Outputs
	DDRA |= _BV(PA0) | _BV(PA1) | _BV(PA2) | _BV(PA3);
	DDRB |= _BV(PB5) | _BV(PB6) | _BV(PB7);
	DDRD |= _BV(PD2) | _BV(PD3);
	DDRE |= _BV(PE3) | _BV(PE4) | _BV(PE5) | _BV(PE6) | _BV(PE7);
	DDRF |= _BV(PF1) | _BV(PF2) | _BV(PF3) | _BV(PF4) | _BV(PF5) | _BV(PF6) | _BV(PF7);
	DDRH |= _BV(PH0) | _BV(PH1) | _BV(PH3) | _BV(PH4) | _BV(PH5);
	DDRJ |= _BV(PJ0) | _BV(PJ1);
	DDRK |= _BV(PK1) | _BV(PK2) | _BV(PK3) | _BV(PK4) | _BV(PK5) | _BV(PK6) | _BV(PK7);
	DDRL |= _BV(PL3) | _BV(PL4) | _BV(PL5);

	// Timers
	// Timer 0 F_CPU/64 interrupt
	TCCR0A |= _BV(WGM01) | _BV(WGM00);
	TCCR0B |= _BV(CS01) | _BV(CS00);
	TIMSK0 |= _BV(TOIE0);

	// 10-Bit Fast PWM mode
	// Clear on Compare Match
	// F_CPU/64
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1C1) | _BV(WGM11) | _BV(WGM10);
	TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31) | _BV(WGM30);
	TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41) | _BV(WGM40);
	TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51) | _BV(WGM50);
	TCCR1B = _BV(CS11) | _BV(WGM12);
	TCCR3B = _BV(CS31) | _BV(WGM32);
	TCCR4B = _BV(CS41) | _BV(WGM42);
	TCCR5B = _BV(CS51) | _BV(WGM52);
	TCCR1C=0;
	TCCR3C=0;
	TCCR4C=0;
	TCCR5C=0;

	// ADC
	ADMUX = _BV(REFS0);
	ADCSRB = 0;
	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

	// Serial
	UCSR0B=0;
	Serial.begin(250000);

	// LCD
	dashBacklights(1);
	lcd.begin();
	lcd.setContrast(60); // Good values range from 40-60
	lcd.updateDisplay();
	lcd.clearDisplay();   // clears the screen and buffer

	/* setLine Example */
	int x0 = LCD_WIDTH/2;
	int y0 = LCD_HEIGHT/2;
	for (float i=0; i<2*M_PI; i+=M_PI/8)
	{
		// Time to whip out some maths:
		const int lineLength = 24;
		int x1 = x0 + lineLength * sin(i);
		int y1 = y0 + lineLength * cos(i);
		
		// setLine(x0, y0, x1, y1, bw) takes five variables. The
		// first four are coordinates for the start and end of the
		// line. The last variable is the color (1=black, 0=white).
		lcd.setLine(x0, y0, x1, y1, BLACK);
		lcd.updateDisplay();
	}
	
	lcd.printf(20, 20, WHITE, "TESTING");

	// Test Cycle
	for(uint16_t i=0;i<0x3FF;i++)
	{
		setSpeed(i);
		setTacho(i);
		setFuel(i);
		setTemp(i);
		setOilP(i)
		setVolt(i);
	}
	leftIndicator(1);
	rightIndicator(1);
	highBeam(1);
	lightsOn(1);
	dashBacklights(1);
	fogLights(1);
	smartLock(1);
	parkBreak(1);
	breakFail(1);
	rearDemist(1);
	oilLight(1);
	battery(1);
	radiator(1);
	airBag(1);
	checkABS(1);
	econ(1);
	seatBelt(1);
	cruise(1);
	fuelLow(1);
	highTemp(1);
	alternator(1);
	oilPressure(1);
	spareLeft(1);
	spareRight(1);
	tickTock(1);
	_delay_ms(500);

	for(uint16_t i=0x3FF;i>0;i--)
	{
		setSpeed(i);
		setTacho(i);
		setFuel(i);
		setTemp(i);
		setOilP(i)
		setVolt(i);
	}
	leftIndicator(0);
	rightIndicator(0);
	highBeam(0);
	lightsOn(0);
	dashBacklights(0);
	fogLights(0);
	smartLock(0);
	parkBreak(0);
	breakFail(0);
	rearDemist(0);
	oilLight(0);
	battery(0);
	radiator(0);
	airBag(0);
	checkABS(0);
	econ(0);
	seatBelt(0);
	cruise(0);
	fuelLow(0);
	highTemp(0);
	alternator(0);
	oilPressure(0);
	spareLeft(0);
	spareRight(0);
	tickTock(0);

	lcd.clearDisplay();

	lcd.printfBig(0, 0, BLACK, "%06d", (int)0);
	lcd.printf(70, 4, BLACK, "KM");

	dashBacklights(1);
	while (1)
	{
		loop();
	}
}

void loop()
{
	//int adcValue=ADC;
	uint8_t byte1, byte2;
	int8_t i8;
	int i16;

	if (Serial.available() < 22) return;
	
	byte1 = Serial.read();
	if (byte1 != PACKET_SYNC) return;
	
	byte1 = Serial.read();
	if (byte1 != PACKET_VER) return;

	// Speed
	byte1 = Serial.read();
	byte1 = (byte1 < 0) ? 0 : ((byte1 > 0xFF) ? 0xFF : byte1);
	setSpeed(map(byte1, 0, 0xFF, 0, 0x03FF));
	
	// RPM
	byte1 = Serial.read();
	byte1 = (byte1 < 0) ? 0 : ((byte1 > 0xFF) ? 0xFF : byte1);
	setTacho(map(byte1, 0, 0xFF, 0, 0x03FF));
	
	// Brake air pressure
	byte1 = Serial.read();
	byte1 = (byte1 < 0) ? 0 : ((byte1 > 0xFF) ? 0xFF : byte1);
	setVolt(map(byte1, 0, 0xFF, 0, 0x03FF));

	// Brake temperature
	byte1 = Serial.read();
	lcd.printf(42, 40, BLACK, "Bk %3dC", byte1);
	
	// Fuel ratio
	byte1 = Serial.read();
	byte1 = (byte1 < 0) ? 0 : ((byte1 > 0xFF) ? 0xFF : byte1);
	setFuel(map(byte1, 0, 0xFF, 0, 0x03FF));
	
	// Ad-Blue Level
	byte1 = Serial.read();
	byte2 = Serial.read();
	i16=byte1;
	i16 |= (byte2<<8);
	lcd.printf(0, 24, BLACK, "AdBlue %3d lts", i16);

	// Oil pressure
	byte1 = Serial.read();
	byte1 = (byte1 < 0) ? 0 : ((byte1 > 0xFF) ? 0xFF : byte1);
	setOilP(map(byte1, 0, 0xFF, 0, 0x03FF));

	// Oil temperature
	byte1 = Serial.read();
	lcd.printf(0, 40, BLACK, "O %3dC", byte1);

	// Water temperature
	byte1 = Serial.read();
	byte1 = (byte1 < 0) ? 0 : ((byte1 > 0xFF) ? 0xFF : byte1);
	setTemp(map(byte1, 0, 0xFF, 0, 0x03FF));
	
	// Battery voltage
	byte1=Serial.read();
	lcd.printf(42, 32, BLACK, "%02d VDC", byte1);
	
	// Odometer
	byte1 = Serial.read();
	byte2 = Serial.read();
	i16=byte1;
	i16 |= (byte2<<8);
	lcd.printfBig(0, 0, BLACK, "%06d", i16);
	lcd.printf(70, 4, BLACK, "KM");

	// Fuel Range
	byte1 = Serial.read();
	byte2 = Serial.read();
	i16=byte1;
	i16 |= (byte2<<8);
	lcd.printf(0, 16, BLACK, "DTE %4d KM ", i16);

	// fuel_average_consumption - ignored
	byte1 = Serial.read();
	byte2 = Serial.read();

	// Gear
	i8=Serial.read();

	if(i8>0)
	{
		if(i8>2)
		{
			i8=i8-2;
			lcd.printf(0, 32, BLACK, "Gear %2d ", i8);
		}
		else lcd.printf(0, 32, BLACK, "Gear C%1d ", i8);
	}
	else
	{
		if(i8==0) lcd.printf(0, 32, BLACK, "Gear N  ");
		if(i8==-1) lcd.printf(0, 32, WHITE, "Gear R1 ");
		if(i8==-2) lcd.printf(0, 32, WHITE, "Gear R2 ");
		if(i8==-3) lcd.printf(0, 32, WHITE, "Gear R3 ");
		if(i8==-4) lcd.printf(0, 32, WHITE, "Gear R4 ");
	}

	// Truck lights byte
	byte1 = Serial.read();
	fogLights((byte1 >> 7) & 0x01);					// light_aux_front
	dashBacklights((byte1 >> 6) & 0x01);			// Tail Lights
	leftIndicator((byte1 >> 5) & 0x01);				// Left Indicator
	rightIndicator((byte1 >> 4) & 0x01);			// Right Indicator
	lightsOn((byte1 >> 3) & 0x01);					// Low Beam
	highBeam((byte1 >> 2) & 0x01);					// High Beam
	oilLight((byte1 >> 1) & 0x01);					// Brakes
	spareLeft((byte1 >> 0) & 0x01);					// Reverse
	tickTock(((byte1 >> 4) & 0x01) || ((byte1 >> 5) & 0x01));

	// Warning lights bytes
	byte1 = Serial.read();
	parkBreak((byte1 >> 7) & 0x01);					// Park Break
	rearDemist((byte1 >> 6) & 0x01);				// Motor Break
	alternator((byte1 >> 5) & 0x01);				// Break Air Pressure Warning
	breakFail((byte1 >> 4) & 0x01);					// Break Air Pressure Emergency
	fuelLow((byte1 >> 3) & 0x01);					// Fuel Warning
	battery((byte1 >> 2) & 0x01);					// Battery Voltage
	oilPressure((byte1 >> 1) & 0x01);				// Oil Pressure
	highTemp((byte1 >> 0) & 0x01);					// Water Temp

	// Enabled flags
	byte1 = Serial.read();
	checkABS((byte1 >> 7) & 0x01);					// light_aux_roof
	airBag((byte1 >> 6) & 0x01);					// light_beacon
	smartLock((byte1 >> 5) & 0x01);					// adblue_warning
	cruise((byte1 >> 4) & 0x01);					// Cruise Control
	spareRight((byte1 >> 3) & 0x01);				// Not Used
	radiator((byte1 >> 2) & 0x01);					// Wipers
	seatBelt((byte1 >> 1) & 0x01);					// Electric Enabled
	econ((byte1 >> 0) & 0x01);						// Engine Enabled

	return;
}

void setSpeed(uint16_t value)
{
	value=map(value, 0, 0x3FF,0x125, 0x3FF);

	float pos=(value*(2*M_PI))/0x3FF;

	uint16_t quad=pos/(M_PI/2);
	if(quad==4) quad=0;

	// Calculate the voltage on the PWM pins based on the angle we want
	uint16_t sinCoil = 0x3FF*sin(pos);
	uint16_t cosCoil = 0x3FF*cos(pos);
	uint16_t sinCoilValue = map(abs(sinCoil), 0, 0x3FF, 0x1FF, 0x3FF); // 8-Bit 128-255
	uint16_t cosCoilValue = map(abs(cosCoil), 0, 0x3FF, 0x1FF, 0x3FF);
	
	switch(quad)
	{
		case 0:
		OCR3A = sinCoilValue; OCR3B = ~sinCoilValue; OCR1A = cosCoilValue; OCR1B = ~cosCoilValue;
		break;

		case 1:
		OCR3A = sinCoilValue; OCR3B = ~sinCoilValue; OCR1A = ~cosCoilValue; OCR1B = cosCoilValue;
		break;

		case 2:
		OCR3A = ~sinCoilValue; OCR3B = sinCoilValue; OCR1A = ~cosCoilValue; OCR1B = cosCoilValue;
		break;

		case 3:
		OCR3A = ~sinCoilValue; OCR3B = sinCoilValue; OCR1A = cosCoilValue; OCR1B = ~cosCoilValue;
		break;
	}
}

void setTacho(uint16_t value)
{
	value=map(value, 0, 0x3FF,0x125, 0x3FF);

	float pos=(value*(2*M_PI))/0x3FF;

	uint16_t quad=pos/(M_PI/2);
	if(quad==4) quad=0;

	// Calculate the voltage on the PWM pins based on the angle we want
	uint16_t sinCoil = 0x3FF*sin(pos);
	uint16_t cosCoil = 0x3FF*cos(pos);
	uint16_t sinCoilValue = map(abs(sinCoil), 0, 0x3FF, 0x1FF, 0x3FF); // 10-Bit 1FF-3FF
	uint16_t cosCoilValue = map(abs(cosCoil), 0, 0x3FF, 0x1FF, 0x3FF);
	
	switch(quad)
	{
		case 0:
		OCR5A = sinCoilValue; OCR5B = ~sinCoilValue; OCR4A = cosCoilValue; OCR4B = ~cosCoilValue;
		break;

		case 1:
		OCR5A = sinCoilValue; OCR5B = ~sinCoilValue; OCR4A = ~cosCoilValue; OCR4B = cosCoilValue;
		break;

		case 2:
		OCR5A = ~sinCoilValue; OCR5B = sinCoilValue; OCR4A = ~cosCoilValue; OCR4B = cosCoilValue;
		break;

		case 3:
		OCR5A = ~sinCoilValue; OCR5B = sinCoilValue; OCR4A = cosCoilValue; OCR4B = ~cosCoilValue;
		break;
	}
}

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

ISR(TIMER0_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	uint16_t m = timer0_millis;
	uint8_t f = timer0_fract;

	m += (((64*256)/(F_CPU/1000000L))/1000);
	f += ((((64 * 256)/(F_CPU/1000000L))%1000)>>3);
	if (f >= (1000>>3))
	{
		f -= (1000>>3);
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

uint16_t millis()
{
	uint16_t m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}
