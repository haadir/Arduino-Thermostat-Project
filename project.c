/********************************************
 *
 *  Name: Haadi Razzak
 *  Email: hrazzak@usc.edu
 *  Section: Wed 3:30
 *  Assignment: Final Project
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "lcd.h"
#include "ds18b20.h"

void play_note();
void variable_delay_us(int16_t);
void splash_screen();
unsigned int tempConvert(unsigned char, unsigned char);

volatile uint8_t new_state, old_state;
volatile uint8_t changed = 0;
volatile int16_t count = 50;
volatile uint8_t a, b;
volatile unsigned int fahrenheit = 0;
volatile int tens = 0;
volatile unsigned char dec = 0;

volatile unsigned int OCR2A_value = 23;

// For timer0
volatile uint32_t buzzer = 175;
volatile int threeFlag = 0;

// button related
enum state
{
	AC,
	HEAT
};
volatile char L, H;
volatile int buttonFlag = 0; // 0 for AC, 1 for HEAT
volatile int state = AC;
volatile int acNum = 50;
volatile int heatNum = 90;

// SERVO STATES
enum ServoMode
{
	TEMPERATURE,
	LOW,
	HIGH
};
volatile int servo_mode = TEMPERATURE;

int main(void)
{
	lcd_init();
	timer0_init();
	timer1_init();
	timer2_init();

	// Initialize DDR and PORT registers and LCD
	ds_init();
	PORTB &= ~(1 << PB4);

	// rotary encoder
	PORTC |= ((1 << PC1) | (1 << PC2));

	// Buttons for AC and HEAT
	DDRD &= ~(1 << PD2); // Configure PD2 as input
	DDRD &= ~(1 << PD3); // Configure PD3 as input
	PORTD |= (1 << PD2); // Enable pull-up resistor on PD2
	PORTD |= (1 << PD3); // Enable pull-up resistor on PD3

	// Enable Pin Change Interrupt on Port C
	PCICR |= (1 << PCIE1);
	PCICR |= (1 << PCIE2);

	// Enable interrupts
	PCMSK1 |= ((1 << PCINT9) | (1 << PCINT10));
	PCMSK2 |= ((1 << PCINT18) | (1 << PCINT19));

	// RGB
	DDRC |= (1 << PC3) | (1 << PC4) | (1 << PC5);

	// BUZZER
	DDRB |= (1 << PB5);

	// SERVO MOTOR
	DDRB |= (1 << PB3);

	// Enable global interrupts
	sei();

	// EEPROM
	acNum = eeprom_read_byte((void *)100);
	heatNum = eeprom_read_byte((void *)110);

	if (acNum < 50 || acNum > 90)
	{
		acNum = 50;
	}

	if (heatNum < 50 || heatNum > 90)
	{
		heatNum = 90;
	}

	count = acNum;

	// Write a spash screen to the LCD
	lcd_writecommand(1);

	lcd_moveto(0, 2);
	lcd_stringout("EE109 Project");
	lcd_moveto(1, 1);
	lcd_stringout("Haadi Razzak");

	_delay_ms(2000);
	lcd_writecommand(1);

	lcd_moveto(1, 0);
	lcd_stringout("Low= ");
	lcd_moveto(1, 8);
	lcd_stringout("High= ");

	// Read the A and B inputs to determine the intial state.
	a = (PINC & (1 << PC1)) >> PC1;
	b = (PINC & (1 << PC2)) >> PC2;

	if (!b && !a)
		old_state = 0;
	else if (!b && a)
		old_state = 1;
	else if (b && !a)
		old_state = 2;
	else
		old_state = 3;

	new_state = old_state;

	// initalize count on LCD
	lcd_moveto(1, 4);
	char count_str[3];
	snprintf(count_str, 4, "%2d", acNum);
	lcd_stringout(count_str);
	lcd_moveto(1, 14);
	snprintf(count_str, 3, "%2d", heatNum);
	lcd_stringout(count_str);

	unsigned char t[2];

	if (ds_init() == 0)
	{ // Initialize the DS18B20
	  // Sensor not responding
	}

	ds_convert(); // Start first temperature conversion

	while (1)
	{
		if (ds_temp(t))
		{
			unsigned int fahrenheit = tempConvert(t[1], t[0]);

			int tens = fahrenheit / 10; // Make an int
			unsigned char dec = fahrenheit % 10;
			char buf[12];
			snprintf(buf, 14, "Temp: %2d.%d", tens, dec);

			OCR2A_value = (((-23) * tens) / 60) + 50;

			if (servo_mode == LOW)
			{
				OCR2A_value = (((-23) * acNum) / 60) + 50;
			}
			else if (servo_mode == HIGH)
			{
				OCR2A_value = (((-23) * heatNum) / 60) + 50;
			}

			// 40-100 range
			if (OCR2A_value < 12)
			{
				OCR2A_value = 12;
			}
			else if (OCR2A_value > 35)
			{
				OCR2A_value = 35;
			}

			// char ocr2a_str[10];
			//snprintf(ocr2a_str, 10, "%d", OCR2A_value);
			// snprintf(ocr2a_str, 10, "%d", OCR2A_value);

			// lcd_moveto(0, 13);
			// lcd_stringout(ocr2a_str);

			//OCR2A = OCR2A_value;

			// TURN ON RED
			if ((fahrenheit / 10) < acNum)
			{
				PORTC &= ~(1 << PC3);
				PORTC |= ((1 << PC4) | (1 << PC5)); // Clear PC4 and PC5
			}

			// TURN ON BLUE
			else if ((fahrenheit / 10) > heatNum)
			{
				PORTC &= ~(1 << PC5);
				PORTC |= ((1 << PC3) | (1 << PC4)); // Clear PC3 and PC4
			}

			// TURN ON GREEN
			else
			{
				PORTC &= ~(1 << PC4);
				PORTC |= ((1 << PC3) | (1 << PC5)); // Clear PC3 and PC5
			}

			// Case where we are 3 degrees over
			if ((fahrenheit / 10) > (heatNum + 3) && threeFlag == 0)
			{
				TCCR0B |= (1 << CS12);
				// buzzer = 175;
				// lcd_moveto(0, 10);
				// lcd_stringout("ABOVE");
				threeFlag = 1;
			}

			// Case where we are 3 degrees under
			else if ((fahrenheit / 10) < (acNum - 3) && threeFlag == 0)
			{
				TCCR0B |= (1 << CS12);
				// buzzer = 175;
				// lcd_moveto(0, 10);
				// lcd_stringout("BELOW");
				threeFlag = 1;
			}

			else
			{
				// lcd_moveto(0, 10);
				// lcd_stringout("     ");
			}

			// Reset the flag if the temperature is within the threshold
			if ((fahrenheit / 10) <= (heatNum + 3) && (fahrenheit / 10) >= (acNum - 3))
			{
				threeFlag = 0;
			}

			lcd_moveto(0, 0);
			lcd_stringout(buf);

			ds_convert(); // Start next conversion
		}
		if (state == AC)
		{
			if (acNum < heatNum)
			{
				acNum = count;
				eeprom_update_byte((void *)100, acNum);
			}

			lcd_moveto(1, 0);
			lcd_stringout("Low? ");
			lcd_moveto(1, 8);
			lcd_stringout("High= ");

			lcd_moveto(1, 5);
			snprintf(count_str, 4, "%2d", acNum);
			lcd_stringout(count_str);
		}
		if (state == HEAT)
		{
			if (heatNum > acNum)
			{
				heatNum = count;
				eeprom_update_byte((void *)110, heatNum);
			}

			lcd_moveto(1, 0);
			lcd_stringout("Low= ");
			lcd_moveto(1, 8);
			lcd_stringout("High? ");

			lcd_moveto(1, 14);
			snprintf(count_str, 3, "%2d", heatNum);
			lcd_stringout(count_str);
		}

		if (changed)
		{
			changed = 0; // Reset changed flag
		}

		if (new_state != old_state)
		{
			changed = 1;
			old_state = new_state;
		}
	}
}

ISR(PCINT1_vect)
{
	// In Task 6, add code to read the encoder inputs and determine the new
	// count value

	int x = PINC;

	// Read the input bits and determine A and B.
	a = (x & (1 << PC1)) >> PC1;
	b = (x & (1 << PC2)) >> PC2;

	if (old_state == 0)
	{
		if (a == 1 && b == 0)
		{
			new_state = 1;
			count++;
		}
		else if (a == 0 && b == 1)
		{
			new_state = 2;
			count--;
		}
	}
	else if (old_state == 1)
	{
		if (a == 0 && b == 0)
		{
			new_state = 0;
			count--;
		}
		else if (a == 1 && b == 1)
		{
			new_state = 3;
			count++;
		}
	}
	else if (old_state == 2)
	{
		if (a == 0 && b == 0)
		{
			new_state = 0;
			count++;
		}
		else if (a == 1 && b == 1)
		{
			new_state = 3;
			count--;
		}
	}
	else
	{
		if (a == 1 && b == 0)
		{
			new_state = 1;
			count--;
		}
		else if (a == 0 && b == 1)
		{
			new_state = 2;
			count++;
		}
	}

	if (count <= 50)
	{
		count = 50;
	}
	else if (count >= 90)
	{
		count = 90;
	}

	if (state == AC)
	{
		if (count >= heatNum)
		{
			count = acNum;
		}
	}
	else if (state == HEAT)
	{
		if (count <= acNum)
		{
			count = heatNum;
		}
	}
}

// CELCIUS TO FAHRENHEIT
unsigned int tempConvert(unsigned char byte1, unsigned char byte0)
{
	unsigned int temp = ((byte1 << 8) + byte0) * 10;
	unsigned int fah = (((9 * temp) / 16) / 5) + (32 * 10);
	return fah;
}

// INTERRUPT FOR BUTTONS
ISR(PCINT2_vect)
{
	L = PIND & (1 << PD2);
	H = PIND & (1 << PD3);

	if (!L) // LOW STATE
	{
		state = AC;
		count = acNum;
		servo_mode = LOW;
	}
	if (!H) // HIGH STATE
	{
		state = HEAT;
		count = heatNum;
		servo_mode = HIGH;
	}

	TCNT1 = 0;			   // Clear Timer1 Counter
	TCCR1B |= (1 << CS12) | (1 << CS10); // Start Timer 1 with prescalar of 256
}

void timer0_init() // buzzer
{
	TCNT0 = 0;
	TCCR0B |= (1 << WGM02);
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = 8000000 / 349 - 1;
}

void timer1_init()
{ // 4 second delay for servo
    TCCR1B |= (1 << WGM12); 
    OCR1A = 62500;
    TIMSK1 |= (1 << OCIE1A);
}

void timer2_init(void)
{
	TIMSK2 |= (1 << OCIE2A);
	TCCR2A |= (0b11 << WGM20);
	TCCR2A |= (0b10 << COM2A0);
	TCCR2B |= (0b111 << CS20);
}

ISR(TIMER0_COMPA_vect)
{
	if (buzzer == 0)
	{
		TCCR0B &= ~(1 << CS12); // timer0 off
		buzzer = 175;
	}
	else
	{
		PORTB ^= (1 << PB5);
		buzzer--;
	}
}

ISR(TIMER1_COMPA_vect)
{
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // Stop Timer1
	servo_mode = TEMPERATURE;
}

ISR(TIMER2_COMPA_vect)
{ // Interrupt for servo
	// 40-100 range
	OCR2A = OCR2A_value;
}