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
 * PIN1 - Virtual GND		   _|	 O    |_		PIN8 - VCC
 * PIN2	- MCP73831 STAT		   _|		  |_		PIN7 - Light sensor
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

volatile uint8_t toRampUp = 0;		//Checked for performing light ramping
volatile uint8_t mode = 0;		//Operating mode of the light --> 0 - normal function, 1 - low battery function, 2 - low low battery function, 3 - charging, 4 - end of charge


void inline setup()
{
	cli();
	
	DDRB = 0x00;
	PORTB = 0x00;
	//DDRB &= ~((1 << PINB1) || (1 << PINB2));						//I/O inputs
	DDRB |= 1 << PINB0;												//I/O outputs
	PORTB |= 1 << PINB1;											//PULL UP RESISTOR for input
	
	DIDR0 |= (1 << ADC0D) | (1 << ADC2D) | (1 << ADC3D);			//Digital input disable register (disabling digital input where not needed for power saving and better ADC)
	
	TCCR0A |=  (1 << WGM01) | (1 << WGM00);							//Waveform Generation Mode... for pin mode PWM ---> |(1 << COM0A1)
	TCCR0B |= (1 << CS01) | (1 << CS00);							//Timer clock
	TIMSK0 |= 1 << TOIE0;											//Timer0 overflow interrupt
	
	MCUCR |= (1 << SM1) | (1 << SE);								//Sleep mode selection
	PCMSK |= (1 << PCINT1);											//Pin change mask
	
	MCUSR |= 0;														//Watchdog settings
	WDTCR |= (1<<WDCE)|(1<<WDE);
	WDTCR |= (1<<WDTIE) | (1<<WDP3) | (1<<WDP0);
	
	ADMUX |= (1 << MUX1) | (1 << MUX0);								//AD multiplexer set to ADC3
	ADCSRA |= 1 << ADLAR;											//ADCH will contain the MSB of the output
	
	
	sei();
}

void sePCI()					//Enable pin change interrupt to look for movement of tilt sensor
{
	GIFR |= 1 << PCIF;			//Clears pin change interrupt flag
	GIMSK |= 1 << PCIE;			//Set pin change interrupt enable bit
}

inline void clPCI()				//Disables pin change interrupt
{
	GIMSK &= ~(1 << PCIE);		//Clear pin change interrupt enable bit
}

void rampUP()						//Dims the light up
{
	TCCR0A |= (1 << COM0A1);		//Sets PINB0 to PWM mode
	while(OCR0A < 0xff)
	{
		OCR0A++;					//Increments PWM
		_delay_ms(8);				//Pauses for 8ms each time
	}
	sePCI();						//enable 
}

void pause(uint8_t sec)		//interruptible pause in seconds, returns if device shaken
{
	uint16_t i = sec*50;
	while((i > 0) && !toRampUp)
	{
		i--;
		_delay_ms(20);
	}
}

void rampDOWN()				//Dims the light down 128 PWM steps, returns if device shaken
{
	uint8_t i = 0;
	while ((i < 186) && !toRampUp && (OCR0A > 0))
	{
		i++;
		OCR0A--;								//Decrements PWM
		_delay_ms(36);							//Pauses for 36ms each time
	}
	if (OCR0A == 0) TCCR0A &= ~(1 << COM0A1);	//Sets PINB0 to normal output mode effectively turning off the light completely, pulling gate to low
}

void blink(uint8_t x)
{
	TCCR0A &= ~(1 << COM0A1);
	
	while (x > 0)
	{
		_delay_ms(100);
		TCCR0A |= (1 << COM0A1);
		_delay_ms(100);
		TCCR0A &= ~(1 << COM0A1);
		--x;
	}
	TCCR0A |= (1 << COM0A1);
}

void sleep()
{
	sePCI();					//Enable pin change interrupt for awakening by reading tile sensor
	sleep_mode();
}

int main(void)
{
	setup();					//Setting up registers
    while (1)
    {
		if (mode > 2)
		{
			
			
		}
		else
		{
			
			
		}
		if (toRampUp==1)
		{
			if (mode == 2) blink(2);
			else
			{
				rampUP();
				toRampUp = 0;
				if (mode > 0) blink(2);
				pause(4);
				if (mode > 0) blink(2);
				rampDOWN();
				pause(10);
				rampDOWN();				
			}
		}
		else if (OCR0A==0)
		{
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
		//if (OCR0A > 32) OCR0A--;			//OCR0A is decremented twice a second when the chip is not sleeping
		//DDRB ^= 1 << PINB0;				//Debugging
	}
}

inline void ADCVccRef()				//Turns ADC reference to Vcc
{
	ADMUX &= ~(1 << REFS0);
}
inline void ADCintRef()				//Turns ADC reference to internal
{
	ADMUX |= 1 << REFS0;
}

inline void ADCbat()				//Sets ADC MUX to ADC3, where the battery is
{
	ADMUX |= 1 << MUX0;
}
inline void ADCcharg()				//Sets ADC MUX to ADC2, where the charger STAT pin is
{
	ADMUX &= ~(1 << MUX0);
}

inline void seADC()					//Turns on ADC
{
	ADCSRA |= 1 << ADEN;
}

inline void clADC()					//Turns off ADC
{
	ADCSRA &= ~(1 << ADEN);
}

void ADCstart()
{
	ADCSRA |= 1 << ADSC;
}

bool ADCcc()						//Returns true if a conversion is complete, false is it is in progress
{
	return !((ADCSRA >> ADSC) & 0x01);
}

inline uint8_t ADCout()				//Returns ADC conversion output - the 8 MSBs of the result which is in the ADCH register
{
	return ADCH;
}



ISR (WDT_vect)									//WDT interrupt to wake from sleep and check brightness once every 8sec
{
	static uint8_t lightTimes = 20;				//How many times light has been detected
	
	//DDRB ^= 1 << PINB0;	//Debugging
	
	seADC();
	ADCbat();
	ADCintRef();
	ADCstart();
	while (!ADCcc()){}
	if (ADCout() < 120) mode = 2;
	else if (ADCout() < 160) mode = 1;
	else mode = 0;
	
	
	if (OCR0A) return;							//If the light is on, no further commands are executed and the routine returns
 	
	
	if (PINB & (1 << PINB2))					//If the photoresistor detects light
	{
		if (lightTimes < 20) lightTimes++;		//The lightTimes is incremented until it reaches 10
	}
	else if (lightTimes >= 20)					//If the photoresistor does not detect light and there have already been 10 instances of light
	{
		lightTimes = 0;							//The lightTimes is set to 0 so that the light will not keep turning on when in the dark
		toRampUp = 1;							//light is to be ramped up at half intensity that will slowly ramp down after 60" (dedicated to my white wolf)
	}

	WDTCR |= (1<<WDTIE);						//The watchdog timer interrupt enable bit should be written to 1 every time the watchdog ISR executes. If a watchdog timer overflow occurs and this bit is not set, the chip will reset. The bit is cleared automatically every time this interrupr is called.
}

ISR (PCINT0_vect)								//Pin change interrupt used to read the tilt sensor, and read the charger's STAT pin
{
	clPCI();									//When the pin change ISR is called, it disables itself with this command. It is then re-enabled in various locations in the code
	
	toRampUp = 1;								//Every time the tilt sensor is triggered, the ON time is extended to the maximum (60" chosen as default)
}


