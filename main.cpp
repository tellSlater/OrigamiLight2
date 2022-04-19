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


#define F_CPU   9600000 / 8
#define BUAD    9600
#define BRC     ((F_CPU/16/BUAD) - 1)

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

volatile uint8_t g_LEDtimer = 255;		//Checked for performing light ramping
volatile uint8_t g_mode = 0;			//Operating mode of the light --> 0 - normal function, 1 - low battery function, 2 - low low battery function, 3 - charging, 4 - end of charge
volatile bool g_BATalarm = false;		//When set a blinking battery alarm has to be output


void inline setup()
{
	cli();
	
	DDRB = 0x00;
	PORTB = 0x00;
	//DDRB &= ~((1 << PINB1) || (1 << PINB2));						//I/O inputs
	DDRB |= 1 << PINB0;												//I/O outputs
	PORTB |= (1 << PINB1);											//PULL UP RESISTOR for input
	
	DIDR0 |= (1 << ADC0D) | (1 << ADC2D) | (1 << ADC3D);			//Digital input disable register (disabling digital input where not needed for power saving and better ADC)
	
	TCCR0A |=  (1 << WGM01) | (1 << WGM00);							//Waveform Generation Mode... for pin mode PWM ---> |(1 << COM0A1)
	TCCR0B |= (1 << CS01) | (1 << CS00);							//Timer clock prescaled to Fcpu / 64
	TIMSK0 |= 1 << TOIE0;											//Timer0 overflow interrupt
	
	MCUCR |= (1 << SM1) | (1 << SE);								//Sleep mode selection
	PCMSK |= (1 << PCINT1);											//Pin change mask
	
	MCUSR |= 0;														//Watchdog settings
	WDTCR |= (1<<WDCE)|(1<<WDE);
	WDTCR |= (1<<WDTIE) | (1<<WDP3) | (1<<WDP0);
	
	ADMUX |= (1 << MUX1) | (1 << MUX0);								//ADC multiplexer set to ADC3
	//ADCSRA |= 1 << ADLAR;											//ADCL will contain the LSBs of the output, set ADC clock to clock/2
	
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

inline void sePWM()
{
	TCCR0A |= (1 << COM0A1);
}

inline void clPWM()
{
	TCCR0A &= ~(1 << COM0A1);
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
	while((i > 0) && !g_LEDtimer)
	{
		i--;
		_delay_ms(20);
	}
}

void rampDOWN()				//Dims the light down 128 PWM steps, returns if device shaken
{
	uint8_t i = 0;
	while ((i < 186) && !g_LEDtimer && (OCR0A > 0))
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
	_delay_ms(100);
	TCCR0A |= (1 << COM0A1);
}

void sleep()
{
	sePCI();					//Enable pin change interrupt for awakening by reading tile sensor
	sleep_mode();
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
	ADMUX &= ~(1 << MUX0);
}
inline void ADCcharg()				//Sets ADC MUX to ADC2, where the charger STAT pin is
{
	ADMUX |= 1 << MUX0;
}

inline void seADC()					//Turns on ADC
{
	ADCSRA |= 1 << ADEN;
}

inline void clADC()					//Turns off ADC
{
	ADCSRA &= ~(1 << ADEN);
}

void ADCstart()						//ADC start conversion
{
	ADCSRA |= 1 << ADSC;
}

bool ADCcc()						//Returns true if a conversion is complete, false if it is in progress
{
	return !((ADCSRA >> ADSC) & 0x01);
}

inline uint8_t ADCout()				//Returns ADC conversion output - the 8 MSBs of the result which is in the ADCH register
{
	return ADC >> 2;
}

int main(void)
{
	setup();					//Setting up registers
			
	sePCI();
    while (1)
    {
	 _delay_ms(14);
		
		
		
		if (g_mode > 2)
		{
			
		}
		else
		{
			if (g_BATalarm)
			{
				if (g_mode > 0)
				{
					for (uint8_t i = 0; i<4; ++i )
					{
						PORTB ^= 1 << PINB0;
						_delay_ms(166);
					}
					g_BATalarm = false;
					
				}
				sePWM();
			}
			
			if (g_mode == 2) 
			{
				clPWM();
				sleep();
			}
			else
			{
				if (g_LEDtimer < 9)
				{
					if (OCR0A < 255) ++OCR0A;
				}
				else if (g_LEDtimer < 100)
				{
					if (OCR0A > 80) --OCR0A;
				}
				else
				{
					if (OCR0A > 0) --OCR0A;
					else
					{
						clPWM();
						sleep();
					}
				}
			}
		}
    }
}

ISR (TIM0_OVF_vect)							//Timer 0 overflow interrupt used for all the timing needs. The prescaler is set to CLOCK/256. This ISR is called approximately 122 times a second
{
	static uint8_t smallTimer = 0;			//The small timer is incremented 122 times to make up one second
	
	smallTimer++;
	if (smallTimer > 73)					//This if is entered once every second
	{
		smallTimer = 0;
		if (g_LEDtimer < 255) g_LEDtimer++;	//OCR0A is decremented once a second when the chip is not sleeping
		//DDRB ^= 1 << PINB0;				//Debugging
	}
}

ISR (WDT_vect)									//WDT interrupt to wake from sleep and check brightness once every 8sec
{
	
	static uint8_t lightTimes = 20;				//Describes how many times light has been detected
	
	//DDRB ^= 1 << PINB0;	//Debugging

	seADC();									//Using ADC to check the battery voltage
				
	ADCbat();
	ADCintRef();
	ADCstart();
	while (!ADCcc()){}
				
	if (ADCout() < 138) g_mode = 2;				//Changing mode to normal, low battery or low low battery depending on the reading from the battery
	else if (ADCout() < 155) g_mode = 1;
	else g_mode = 0;
		
	
	clADC();									//Disable ADC to save power
	
	 
	
	WDTCR |= (1<<WDTIE);						//The watchdog timer interrupt enable bit should be written to 1 every time the watchdog ISR executes. If a watchdog timer overflow occurs and this bit is not set, the chip will reset. The bit is cleared automatically every time this interrupr is called.

	if (OCR0A) return;							//If the light is on, no further commands are executed and the routine returns
 	
	
	if (PINB & (1 << PINB2))					//If the photoresistor detects light
	{
		if (lightTimes < 20) lightTimes++;		//The lightTimes is incremented until it reaches 20
	}
	else if (lightTimes >= 20)					//If the photoresistor does not detect light and there have already been 10 instances of light
	{
		lightTimes = 0;							//The lightTimes is set to 0 so that the light will not keep turning on when in the dark
		g_LEDtimer = 0;							//light is to be ramped up
		g_BATalarm = true;
	}
	
}

ISR (PCINT0_vect)								//Pin change interrupt used to read the tilt sensor, and read the charger's STAT pin
{
	//clPCI();									//When the pin change ISR is called, it disables itself with this command. It is then re-enabled in various locations in the code
	
	g_LEDtimer = 0;								//Every time the tilt sensor is triggered, the ON time is extended to the maximum (60" chosen as default)
	g_BATalarm = true;
}


