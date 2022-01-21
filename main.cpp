/*
 * OrigamiLight.cpp
 *
 * Created: 18/1/2021 16:57:19
 * Author : Windfish (Visit windfish.ddns.net)
 * 
 *
 * Chip used: ATTiny13A
 * The internal oscillator and no prescaling is used for this project.
 * The state of the chip's fuses should be: (E:FF, H:FF, L:7A).
 *
 *								 _________
 * PIN1 - Not connected		   _|	 O    |_		PIN8 - VCC
 * PIN2	- Virtual GND		   _|		  |_		PIN7 - Light sensor
 * PIN3	- Battery sensing	   _|ATTiny13A|_		PIN6 - Vibration sensor
 * PIN4	- Ground			   _|		  |_		PIN5 - LEDs (PWM)
 *							    |_________|
 */ 


#define F_CPU   1200000
#define BUAD    9600
#define BRC     ((F_CPU/16/BUAD) - 1)

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

volatile uint8_t toRampUp = 0;		//This variable is checked for performing light ramping

void inline setup()
{
	cli();
	
	DDRB = 0x00;
	PORTB = 0x00;
	//DDRB &= ~((1 << PINB1) || (1 << PINB2));						//I/O inputs
	DDRB |= 1 << PINB0;												//I/O outputs
	PORTB |= 1 << PINB1;											//PULL UP RESISTOR for input
	
	TCCR0A |=  (1 << WGM01) | (1 << WGM00);							//Waveform Generation Mode... for pin mode PWM ---> |(1 << COM0A1)
	TCCR0B |= (1 << CS01) | (1 << CS00);							//Timer clock
	TIMSK0 |= 1 << TOIE0;											//Timer0 overflow interrupt
	
	MCUCR |= (1 << SM1) | (1 << SE);								//Sleep mode selection
	PCMSK |= (1 << PCINT1);											//Pin change mask
	
	MCUSR = 0;														//Watchdog settings
	WDTCR = (1<<WDCE)|(1<<WDE);
	WDTCR = (1<<WDTIE) | (1<<WDP3) | (1<<WDP0);
	
	sei();
}

void sePCI()					//Enable pin change interrupt to look for movement of tilt sensor
{
	GIFR |= 1 << PCIF;		//Clears pin change interrupt flag
	GIMSK |= 1 << PCIE;			//Set pin change interrupt enable bit
}

inline void clPCI()				//Disables pin change interrupt
{
	GIMSK &= ~(1 << PCIE);		//Clear pin change interrupt enable bit
}

void rampUP()						//Dims the light up
{
	TCCR0A |= (1 << COM0A1);			//Sets PINB0 to PWM mode
	while(toRampUp>0)
	{
		toRampUp--;
		if (OCR0A < 0xff) OCR0A++;	//Increments PWM
		_delay_ms(32);				//Pauses for 16ms each time for a total of 255 * 16ms = 4080ms or 4.080sec for dimming to full brightness and exitig the loop
	}
	sePCI();
}

void rampDOWN()					//Dims the light down
{
	while (OCR0A > 0x00)
	{
		OCR0A--;				//Decrements PWM
		_delay_ms(36);			//Pauses for 16ms each time for a total of 255 * 16ms = 4080ms or 4.080sec for dimming to the lowest brightness and exitig the loop
	}
	TCCR0A &= ~(1 << COM0A1);		//Sets PINB0 to normal output mode effectively turning off the light completely, pulling gate to low
}

void sleep()
{
	sePCI();					//Enable pin change interrupt for awakening by reading tile sensor
	sleep_mode();
}

int main(void)
{
	setup();								//Setting up registers
    while (1)
    {
		if (toRampUp)
		{
			rampUP();
		}
		else if (OCR0A <= 32)
		{
			rampDOWN();
			sleep();
		}
    }
}

ISR (TIM0_OVF_vect)							//Timer 0 overflow interrupt used for all the timing needs. The prescalre is set to CLOCK/256. This ISR is called approximately 122 times a second
{
	static uint8_t smallTimer = 0;			//The small timer is incremented 122 times to make up one second
	
	smallTimer++;
	if (smallTimer > 37)					//This if is entered once every second
	{
		smallTimer = 0;
		if (OCR0A > 32) OCR0A--;			//OCR0A is decremented twice a second when the chip is not sleeping
		//DDRB ^= 1 << PINB0;				//Debugging
	}
}

ISR (WDT_vect)									//WDT interrupt to wake from sleep and check brightness once every 8sec
{
	static uint8_t lightTimes = 20;				//How many times light has been detected
	
	WDTCR |= (1<<WDTIE);						//The watchdog timer interrupt enable bit should be written to 1 every time the watchdog ISR executes. If a watchdog timer overflow occurs and this bit is not set, the chip will reset
	//DDRB ^= 1 << PINB0;	//Debugging
	
	if (OCR0A) return;							//If the light is on, no commands are executed and the routine returns
 	
	
	if (PINB & (1 << PINB2))					//If the photoresistor detects light
	{
		if (lightTimes < 20) lightTimes++;		//The lightTimes is incremented until it reaches 10
	}
	else if (lightTimes >= 20)					//If the photoresistor does not detect light and there have already been 10 instances of light
	{
		lightTimes = 0;							//The lightTimes is set to 0 so that the light will not keep turning on when in the dark
		toRampUp = 152;							//light is to be ramped up at half intensity that will slowly ramp down after 60" (dedicated to my white wolf)
	}
}

ISR (PCINT0_vect)								//Pin change interrupt used to read the tilt sensor, wake from sleep and extend ON time
{
	clPCI();									//When the pin change ISR is called, it disables itself with this command. It is then re-enabled in various locations in the code
	toRampUp = 50;								//Every time the tilt sensor is triggered, the ON time is extended to the maximum (60" chosen as default)
}


