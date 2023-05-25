#define F_CPU 16000000UL  //Defines F_CPU as 16,000,000 Hz, the system's clock frequency.
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> 
#include <string.h>  

#define LCD_RS_PIN 11  // Pin 12
#define LCD_EN_PIN 12 // Pin 11
#define LCD_D4_PIN 5  // Pin 5
#define LCD_D5_PIN 4  // Pin 4
#define LCD_D6_PIN 3  // Pin 3
#define LCD_D7_PIN 2  // Pin 2
#define A0 0

volatile int sensorValue = 0;  // Declare volatile to use in ISR

void delay500ms() {
	TCNT1 = 0;
	OCR1A = 15624; // Adjusted value for 500 ms delay (16 MHz clock, Prescaler 1024)
	TCCR1A = 0; // WGM=0100 (CTC)
	TCCR1B = 0x04; // Prescaler 1024 (N = 1024, CS1[2:0] = 0b100)
	while ((TIFR1 & (1 << OCF1A)) == 0) {} // Wait until OCF1A is set
	TCCR1B = 0; // Stop timer1
	TIFR1 = 1 << OCF1A; // Clear flag
}

void start() {
	// Set pin directions
	DDRC &= ~(1 << A0);  // A0 as input
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2);  // Pin 8, 9, 10 as output
	// Configure Timer/Counter1
	TCCR1B |= (1 << WGM12);  // CTC mode, TOP = OCR1A
	OCR1A = 15624;  // Generate interrupt every 1 second (assuming 16MHz clock)
	TIMSK1 |= (1 << OCIE1A);  // Enable Timer/Counter1 compare match A interrupt
	// Initialize USART
	uart_init();
	sei();  // Enable global interrupts
}

int readADC(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);  // Select ADC channel (0-7)
	ADCSRA |= (1 << ADSC);  // Start single conversion
	while (ADCSRA & (1 << ADSC));  // Wait for conversion to complete
	return ADC;
}

void lcdCommand(uint8_t cmd) {
	PORTD = cmd & 0xF0;  // Send higher nibble
	PORTD &= ~(1 << LCD_RS_PIN);  // Set RS pin low for command mode
	PORTD |= (1 << LCD_EN_PIN);  // Set enable pin high
	_delay_us(1);
	PORTD &= ~(1 << LCD_EN_PIN);  // Set enable pin low
	PORTD = (cmd << 4) & 0xF0;  // Send lower nibble
	PORTD |= (1 << LCD_EN_PIN);  // Set enable pin high
	_delay_us(1);
	PORTD &= ~(1 << LCD_EN_PIN);  // Set enable pin low

	_delay_us(100);
}

void lcdData(uint8_t data) {
	PORTD = data & 0xF0;  // Send higher nibble
	PORTD |= (1 << LCD_RS_PIN);  // Set RS pin high for data mode
	PORTD |= (1 << LCD_EN_PIN);  // Set enable pin high
	_delay_us(1);
	PORTD &= ~(1 << LCD_EN_PIN);  // Set enable pin low
	PORTD = (data << 4) & 0xF0;  // Send lower nibble
	PORTD |= (1 << LCD_EN_PIN);  // Set enable pin high
	_delay_us(1);
	PORTD &= ~(1 << LCD_EN_PIN);  // Set enable pin low
	_delay_us(100);
}

void lcdInit() {
	_delay_ms(50);

	// Set interface to 4-bit mode
	PORTD = 0x30;
	PORTD |= (1 << LCD_EN_PIN);
	_delay_us(1);
	PORTD &= ~(1 << LCD_EN_PIN);
	_delay_ms(5);

	PORTD |= (1 << LCD_EN_PIN);
	_delay_us(1);
	PORTD &= ~(1 << LCD_EN_PIN);
	_delay_us(100);

	PORTD |= (1 << LCD_EN_PIN);
	_delay_us(1);
	PORTD &= ~(1 << LCD_EN_PIN);
	_delay_us(100);

	// Set interface to 4-bit mode, 2 lines, 5x8 font
	lcdCommand(0x28);

	// Display on, cursor off, blinking off
	lcdCommand(0x0C);

	// Clear display
	lcdCommand(0x01);
	_delay_ms(2);
}

void lcdPrint(const char* str) {
	while (*str) {
		lcdData(*str);
		str++;
	}
}

void uart_init() {
	UBRR0H = (unsigned char)(103 >> 8);
	UBRR0L = (unsigned char)(103);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);  // Enable receiver and transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // Set data frame: 8-bit data, 1 stop bit
}

void uart_putc(char c) {
	while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty transmit buffer
	UDR0 = c;  // Put data into buffer, sends the data
}

void uart_puts(const char* s) {
	while (*s) {
		uart_putc(*s);
		s++;
	}
}

void uart_putln(const char* s) {
	uart_puts(s);
	uart_putc('\n');
}

// Interrupt Service Routine for Timer/Counter1 compare match A interrupt
ISR(TIMER1_COMPA_vect) {
	sensorValue = readADC(A0);  // Update the sensor value in the ISR
}

int main(void) {
	start();
	lcdInit();
	while (1) {
		lcdCommand(0x80);  // Set cursor to line 1
		lcdPrint("Sound intensity:");
		lcdCommand(0xC0);  // Set cursor to line 2
		// print the number of seconds since reset:
		int currentSensorValue = sensorValue;  // Store sensor value in a local variable
		char sensorValueStr[6];
		itoa(currentSensorValue, sensorValueStr, 10);
		lcdPrint(sensorValueStr);
		if (currentSensorValue >= 300 && currentSensorValue < 400) {
			PORTB |= (1 << PB0);  // Set pin 8 high
			PORTB &= ~(1 << PB1); // Set pin 9 low
			PORTB &= ~(1 << PB2); // Set pin 10 low
			} else if (currentSensorValue >= 400 && currentSensorValue < 500) {
			PORTB &= ~(1 << PB0); // Set pin 8 low
			PORTB |= (1 << PB1);  // Set pin 9 high
			PORTB &= ~(1 << PB2); // Set pin 10 low
			} else if (currentSensorValue >= 500) {
			PORTB &= ~(1 << PB0); // Set pin 8 low
			PORTB &= ~(1 << PB1); // Set pin 9 low
			PORTB |= (1 << PB2);  // Set pin 10 high
			} else {
			PORTB &= ~(1 << PB0); // Set pin 8 low
			PORTB &= ~(1 << PB1); // Set pin 9 low
			PORTB &= ~(1 << PB2); // Set pin 10 low
		}

		delay500ms();
	}

	return 0;
}
