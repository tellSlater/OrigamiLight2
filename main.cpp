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
volatile uint8_t g_mode = 0;			//Operating mode of the light --> 0 - normal function, 1 - low battery function, 2 - low low battery function, 3 - charging
volatile bool g_BATalarm = false;		//When set a blinking battery alarm has to be output
volatile bool g_chargeLock = true;		//Device locked until this becomes false when a charger is inserted for the first time

volatile bool debugFlag = false;

void inline setup()
{
	cli();
	
	//DDRB = 0x00;
	//PORTB = 0x00;
	//DDRB &= ~((1 << PINB1) || (1 << PINB2));						//I/O inputs
	DDRB |= 1 << PINB0;												//I/O outputs
	PORTB |= (1 << PINB1) | (1 << PINB3);							//PULL UP RESISTOR for vibration sensor and charger IC STAT pin
	
	DIDR0 |= (1 << ADC0D) | (1 << ADC2D);							//Digital input disable register (disabling digital input where not needed for power saving and better ADC)
	
	TCCR0A |=  (1 << WGM01) | (1 << WGM00);							//Waveform Generation Mode... for pin mode PWM ---> |(1 << COM0A1)
	TCCR0B |= (1 << CS01) | (1 << CS00);							//Timer clock prescaled to Fcpu / 64
	TIMSK0 |= 1 << TOIE0;											//Timer0 overflow interrupt
	
	MCUCR |= (1 << SM1) | (1 << SE);								//Sleep mode selection
	
	PCMSK |= (1 << PCINT3);											//Pin change mask, enable PCINT3 for charger check
	
	MCUSR |= 0x00;													//Watchdog settings
	WDTCR |= (1<<WDCE);												//Interrupt only mode
	WDTCR |= (1<<WDTIE) | (1<<WDP3);								//Once every 8"
	
	ADMUX |= (1 << MUX1) | (1 << REFS0);							//ADC multiplexer set to ADC2 with internal reference for reading battery level
	
	sei();
}

void vGroundON()					//Pin 5 to GND
{
	DDRB |= 1 << PINB5;
}

void vGroundOFF()					//Pin 5 to high Z
{
	DDRB &= ~(1 << PINB5);
}

inline void clPCIflag()				//Clears pin change interrupt flag
{
	GIFR |= 1 << PCIF;
}

inline void sePCI()					//Enable pin change interrupt
{
	GIMSK |= 1 << PCIE;				//Set pin change interrupt enable bit
}

inline void clPCI()					//Disables pin change interrupt
{
	GIMSK &= ~(1 << PCIE);			//Clear pin change interrupt enable bit
}

inline void sePWM()					//Set PWM function on PINB0
{
	TCCR0A |= (1 << COM0A1);
}

inline void clPWM()					//Set normal function on PINB0
{
	TCCR0A &= ~(1 << COM0A1);
}

void sleep()
{
	//sePCI();						//Enable pin change interrupt for awakening by reading tile sensor
	//OCR0A = 0x00;
	while(OCR0A > 0)
	{
		_delay_ms(2);
		--OCR0A;
	}
	clPWM();
	g_LEDtimer = 255;
	PORTB &= ~(1 << PINB0);
	vGroundOFF();
	_delay_ms(10);
	
	sleep_mode();
}

inline void seADC()					//Turns on ADC
{
	ADCSRA |= 1 << ADEN;
}

inline void clADC()					//Turns off ADC
{
	ADCSRA &= ~(1 << ADEN);
}

inline void ADCstart()				//ADC start conversion
{
	ADCSRA |= 1 << ADSC;
}

inline bool ADCcc()					//Returns true if a conversion is complete, false if it is in progress
{
	return !((ADCSRA >> ADSC) & 0x01);
}

inline uint8_t ADCout()				//Returns ADC conversion output - the 8 MSBs of the result which is in the ADCH register
{
	return ADC >> 2;
}

inline void blink(const uint8_t times)
{
	PORTB &= ~(1 << PINB0);
	for (uint8_t i = 0; i<(2*times); ++i )
	{
		PORTB ^= 1 << PINB0;
		_delay_ms(166);
	}
}

inline bool notCharging()
{
	return PINB & (1 << PINB3);
}

void my_delay_ms(uint16_t ms)
{
	while (0 < ms)
	{
		_delay_ms(1);
		--ms;
	}
}

void debugBlink(uint16_t delay, uint8_t times = 2)	//Light blinking to debug the program
{
	//return;
	bool PWMset = TCCR0A & (1 << COM0A1);
	clPWM();
	for (uint8_t i = 0; i < times; ++i)
	{
		sePWM();
		my_delay_ms(delay/2);
		clPWM();
		my_delay_ms(delay/2);
	}
	if (PWMset) sePWM();
}

int main(void)
{
	setup();						//Setting up registers
			
	clPCIflag();
	sePCI();
	
    while (1)
    {
		if (g_LEDtimer < 150)			//If the light is operating
			_delay_ms(14);				//This while part operates at a slower rate, executing once every 14ms

		if (!notCharging())				//If it is charging enter charging mode
			g_mode = 3;
		
		
// 		if (debugFlag)
// 		{
// 			debugFlag = false;
// 			debugBlink(600);
// 		}
		
		if (g_mode > 2)
		{
			sleep();
			//debugFlag = true;
			//debugBlink(1000);
		}
		else
		{
			if (g_BATalarm)
			{
				PORTB &= ~(1 << PINB0);
				if (g_mode > 0)
				{
					blink(2);
				}
				sePWM();
				g_BATalarm = false;
				//debugBlink(600);
			}
			
			//debugBlink(100);
			if (g_mode == 2) 
			{
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
						sleep();
					}
				}
			}
		}
    }
}

ISR (TIM0_OVF_vect)								//Timer 0 overflow interrupt used for all the timing needs. The prescaler is set to CLOCK/256. This ISR is called approximately 122 times a second
{
	static uint8_t smallTimer = 0;				//The small timer is incremented 122 times to make up one second
	
	smallTimer++;
	if (smallTimer > 73)						//This if is entered once every second
	{
		smallTimer = 0;
		if (g_LEDtimer < 255) g_LEDtimer++;		//OCR0A is decremented once a second when the chip is not sleeping
		//DDRB ^= 1 << PINB0;					//Debugging
	}
}

ISR (WDT_vect)									//WDT interrupt to wake from sleep and check brightness once every 8sec
{
	//WDTCR |= (1<<WDTIE);						//The watchdog timer interrupt enable bit should be written to 1 every time the watchdog ISR executes. If a watchdog timer overflow occurs and this bit is not set, the chip will reset. The bit is cleared automatically every time this interrupt is called.
	
	if (g_chargeLock || g_mode>2) return;		//If device is locked or charging return	
	
	//DDRB ^= 1 << PINB0;	//Debugging

	vGroundON();								//Turning on the virtual ground on pin 5 for battery sense and photo-resistor voltage dividers
	_delay_ms(1);

	static uint8_t batCheckCounter = 0;			//For battery check once in 10 WDT interrupt calls
	++batCheckCounter;
	if (batCheckCounter > 10)
	{
		batCheckCounter = 0;
		
		seADC();								//Using ADC to check the battery voltage
		
		ADCstart();
		while (!ADCcc()){}
				
		if (ADCout() < 140) g_mode = 2;				//Changing mode to normal, low battery or low low battery depending on the reading from the battery
		else if (ADCout() < 161) g_mode = 1;
		else g_mode = 0;
		
		clADC();
	}									//Disable ADC to save power
	
	if (OCR0A) return;							//If the light is on, no further commands are executed and the routine returns
 
	static uint8_t lightTimes = 20;				//Describes how many times light has been detected
	volatile static bool darkTrigger = false;			//If darkness is detected this flag is set and on the next WDT interrupt, the light is turned on
	
	if (darkTrigger)
	{
		lightTimes = 0;							//The lightTimes is set to 0 so that the light will not keep turning on when in the dark
		g_LEDtimer = 0;							//light is to be ramped up
		g_BATalarm = true;
		darkTrigger = false;
		return;
	}
	
	if (PINB & (1 << PINB2))					//If the photoresistor detects light
	{
		if (lightTimes < 20) lightTimes++;		//The lightTimes is incremented until it reaches 20
	}
	else if (lightTimes >= 20)					//If the photoresistor does not detect light and there have already been 10 instances of light
	{
		darkTrigger = true;
	}
}

ISR (PCINT0_vect)								//Pin change interrupt used to read the tilt sensor, and read the charger's STAT pin
{
	if (g_chargeLock)							//Only this if is executed until a charger is inserted for the first time. At this point only PCINT3 is active in the PCMSK
	{
		_delay_ms(10);
		if (notCharging())
		{
			return;
		}
		else
		{
			g_chargeLock = false;				//Disable lock on device when a charger is plugged in for the first time
			PCMSK |= 1 << PCINT1;				//Also enables pin change interrupt PCINT1 for vibration sensor reading
		}
	}
	
	if (notCharging())							//Changing mode to normal, low battery or low low battery depending on the reading from the battery
	{
		if (g_mode > 2)
		{
			g_mode = 0;
			_delay_ms(10);						//This small delay helps prevent a second consecutive trigger of this interrupt that would turn on the light
			clPCIflag();						//After the delay the flag is cleared
		}
		else
		{
			g_LEDtimer = 0;						//Every time the tilt sensor is triggered, the ON time is extended to the maximum (60" chosen as default)
			g_BATalarm = true;
		}
	}
	else
	{
		g_mode = 3;
	}
}



