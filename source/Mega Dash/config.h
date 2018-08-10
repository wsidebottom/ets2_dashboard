/*
 * config.h
 *
 * Created: 12/12/2017 12:19:13 PM
 *  Author: william
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

uint16_t millis();
long map(long x, long in_min, long in_max, long out_min, long out_max);
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define setFuel(val)						{ OCR1C=val;};
#define setOilP(val)						{ OCR3C=val; };
#define setVolt(val)						{ OCR4C=val; };
#define setTemp(val)						{ OCR5C=val; };
#define leftIndicator(bool)					{ if(bool) PORTF |= _BV(PF1); else PORTF &= ~_BV(PF1); };
#define rightIndicator(bool)				{ if(bool) PORTF |= _BV(PF2); else PORTF &= ~_BV(PF2); };
#define highBeam(bool)						{ if(bool) PORTF |= _BV(PF3); else PORTF &= ~_BV(PF3); };
#define lightsOn(bool)						{ if(bool) PORTF |= _BV(PF4); else PORTF &= ~_BV(PF4); };
#define dashBacklights(bool)				{ if(bool) PORTF |= _BV(PF5); else PORTF &= ~_BV(PF5); };
#define fogLights(bool)						{ if(bool) PORTF |= _BV(PF6); else PORTF &= ~_BV(PF6); };
#define smartLock(bool)						{ if(bool) PORTF |= _BV(PF7); else PORTF &= ~_BV(PF7); };
#define parkBreak(bool)						{ if(bool) PORTK |= _BV(PK0); else PORTK &= ~_BV(PK0); };
#define breakFail(bool)						{ if(bool) PORTK |= _BV(PK1); else PORTK &= ~_BV(PK1); };
#define rearDemist(bool)					{ if(bool) PORTK |= _BV(PK2); else PORTK &= ~_BV(PK2); };
#define oilLight(bool)						{ if(bool) PORTK |= _BV(PK3); else PORTK &= ~_BV(PK3); };
#define battery(bool)						{ if(bool) PORTK |= _BV(PK4); else PORTK &= ~_BV(PK4); };
#define radiator(bool)						{ if(bool) PORTK |= _BV(PK5); else PORTK &= ~_BV(PK5); };
#define airBag(bool)						{ if(bool) PORTK |= _BV(PK6); else PORTK &= ~_BV(PK6); };
#define checkABS(bool)						{ if(bool) PORTK |= _BV(PK7); else PORTK &= ~_BV(PK7); };
#define econ(bool)							{ if(bool) PORTJ |= _BV(PJ1); else PORTJ &= ~_BV(PJ1); };
#define seatBelt(bool)						{ if(bool) PORTJ |= _BV(PJ0); else PORTJ &= ~_BV(PJ0); };
#define cruise(bool)						{ if(bool) PORTH |= _BV(PH1); else PORTH &= ~_BV(PH1); };
#define fuelLow(bool)						{ if(bool) PORTH |= _BV(PH0); else PORTH &= ~_BV(PH0); };
#define highTemp(bool)						{ if(bool) PORTD |= _BV(PD3); else PORTD &= ~_BV(PD3); };
#define alternator(bool)					{ if(bool) PORTD |= _BV(PD2); else PORTD &= ~_BV(PD2); };
#define oilPressure(bool)					{ if(bool) PORTA |= _BV(PA0); else PORTA &= ~_BV(PA0); };
#define spareLeft(bool)						{ if(bool) PORTA |= _BV(PA1); else PORTA &= ~_BV(PA1); };
#define spareRight(bool)					{ if(bool) PORTA |= _BV(PA2); else PORTA &= ~_BV(PA2); };
#define tickTock(bool)						{ if(bool) PORTA |= _BV(PA3); else PORTA &= ~_BV(PA3); };

// Pins
#define spiSetupPins()						{ DDRB |= _BV(PB1) | _BV(PB2) | _BV(PB0); };
#define spiSS(bool)							{ if(bool) PORTB |= _BV(PB0); else PORTB &= ~_BV(PB0); };

// LCD
#define lcdSetupPins()						{ DDRL |= _BV(PL0) | _BV(PL1); };
#define lcdRst(bool)						{ if(bool) PORTL |= _BV(PL1); else PORTL &= ~_BV(PL1); };
#define lcdDC(bool)							{ if(bool) PORTL |= _BV(PL0); else PORTL &= ~_BV(PL0); };

#endif /* CONFIG_H_ */