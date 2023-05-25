#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


// LCD Implementation

#define RS_PIN 12
#define EN_PIN 11
#define D4_PIN 5
#define D5_PIN 4
#define D6_PIN 3
#define D7_PIN 2

#define A0_PIN 0

void LCD_command(uint8_t command) {
    PORTD = (PORTD & 0x0F) | (command & 0xF0); // Send higher nibble
    PORTD &= ~(1 << RS_PIN); // RS = 0 for command
    PORTD |= (1 << EN_PIN); // EN = 1 for high-to-low pulse
    _delay_us(1);
    PORTD &= ~(1 << EN_PIN); // EN = 0
    _delay_us(100);

    PORTD = (PORTD & 0x0F) | ((command & 0x0F) << 4); // Send lower nibble
    PORTD |= (1 << EN_PIN); // EN = 1 for high-to-low pulse
    _delay_us(1);
    PORTD &= ~(1 << EN_PIN); // EN = 0
    _delay_ms(2);
}

void LCD_data(uint8_t data) {
    PORTD = (PORTD & 0x0F) | (data & 0xF0); // Send higher nibble
    PORTD |= (1 << RS_PIN); // RS = 1 for data
    PORTD |= (1 << EN_PIN); // EN = 1 for high-to-low pulse
    _delay_us(1);
    PORTD &= ~(1 << EN_PIN); // EN = 0
    _delay_us(100);

    PORTD = (PORTD & 0x0F) | ((data & 0x0F) << 4); // Send lower nibble
    PORTD |= (1 << EN_PIN); // EN = 1 for high-to-low pulse
    _delay_us(1);
    PORTD &= ~(1 << EN_PIN); // EN = 0
    _delay_ms(2);
}

void LCD_init() {
    DDRD |= (1 << RS_PIN) | (1 << EN_PIN) | (1 << D4_PIN) | (1 << D5_PIN) | (1 << D6_PIN) | (1 << D7_PIN);
    _delay_ms(20);

    LCD_command(0x02); // Return home
    LCD_command(0x28); // 4-bit mode, 2 lines, 5x8 font
    LCD_command(0x0C); // Display on, cursor off
    LCD_command(0x06); // Entry mode: increment cursor, no display shift
    LCD_command(0x01); // Clear display
    _delay_ms(2);
}

void LCD_setCursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    LCD_command(0x80 | (col + row_offsets[row]));
}

void ADC_init() {
    ADMUX = (1 << REFS1) | (1 << REFS0); // Set analog reference to internal (2.56V)
    ADCSRA = (1 << ADEN); // Enable ADC
}

uint16_t ADC_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Select ADC channel (0-7)
    ADCSRA |= (1 << ADSC); // Start single conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

void delay500ms() {
    TCNT1 = 0;
    OCR1A = 15624; // Adjusted value for 500 ms delay (16 MHz clock, prescaler 1024)
    TCCR1A = 0; // WGM=0100 (CTC)
    TCCR1B = 0x04; // Prescaler 1024 (N = 1024, CS1[2:0] = 0b100)
    while ((TIFR1 & (1 << OCF1A)) == 0) {} // Wait until OCF1A is set
    TCCR1B = 0; // Stop timer1
    TIFR1 = 1 << OCF1A; // Clear flag
}

void UART_init() {
    // Baud Rate adjusted to 9600
    UBRR0H = (unsigned char)(103 >> 8);
    UBRR0L = (unsigned char)(103);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Set data frame: 8-bit data, 1 stop bit
}

void UART_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
    UDR0 = c; // Put data into buffer, sends the data
}

void UART_puts(const char* s) {
    while (*s) {
        UART_putc(*s);
        s++;
    }
}

int main() {
    LCD_init();
    LCD_setCursor(0, 1);
    LCD_command(0x0F); // Turn on display and cursor

    ADC_init();
    UART_init();

    sei(); // Enable global interrupts

    while (1) {
        uint16_t sensorValue = ADC_read(A0_PIN);
        char sensorValueStr[6];
        sprintf(sensorValueStr, "%5d", sensorValue);
        
        LCD_setCursor(0, 1);
        for (int i = 0; i < 5; i++) {
            LCD_data(sensorValueStr[i]);
        }

        if (sensorValue >= 300 && sensorValue < 400) {
            PORTB |= (1 << PB0); // Set pin 8 high
            PORTB &= ~(1 << PB1); // Set pin 9 low
            PORTB &= ~(1 << PB2); // Set pin 10 low
            delay500ms();
        }
        else if (sensorValue >= 400 && sensorValue < 700) {
            PORTB |= (1 << PB1); // Set pin 9 high
            PORTB &= ~(1 << PB0); // Set pin 8 low
            PORTB &= ~(1 << PB2); // Set pin 10 low
            delay500ms();
        }
        else if (sensorValue > 800) {
            PORTB |= (1 << PB2); // Set pin 10 high
            PORTB &= ~(1 << PB0); // Set pin 8 low
            PORTB &= ~(1 << PB1); // Set pin 9 low
            delay500ms();
        }
        else {
            PORTB &= ~(1 << PB0); // Set pin 8 low
            PORTB &= ~(1 << PB1); // Set pin 9 low
            PORTB &= ~(1 << PB2); // Set pin 10 low
            delay500ms();
        }

        UART_puts(sensorValueStr);
        UART_puts("\n");
    }

    return 0;
}
