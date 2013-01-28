#define F_CPU 8000000

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

#define LCD_ADDR 0x74   // Address to match for LCD
#define NODE_ADDR 0x76  // Address to for LEDs/buttons 

#define STATE_IDLE 0
#define STATE_LCD 1
#define STATE_LEDS 2
uint8_t state = STATE_IDLE;

#define RED_OC OCR1BL
#define GREEN_OC OCR1AL
#define BLUE_OC OCR0A

#define I_RED 0
#define I_GREEN 1
#define I_BLUE 2
uint8_t led_vals[3] = {0, 0, 0};

/* timer0 used for blue LEDs and voltage inverter (VLCD)
 */
void setup_timer0( void ) {

    DDRD |= 0x60;

    TCCR0A = (0xF << COM0B0); // OC0A/B inverted mode
    TCCR0A |= (3 << WGM00); // Fast PWM mode

    BLUE_OC = ~0;
    OCR0B = 0x80; // Square wave on OCR0B for voltage inverter

    TCCR0B |= (4 << CS00); // Clock source = CLK/256
}

/* timer1 used for red and green LEDs
 */
void setup_timer1( void ) {

    DDRB |= 0x06;

    TCCR1A = (0xF << COM1B0); // OC1A/B inverted mode
    TCCR1A |= (1 << WGM10); TCCR1B |= (1 << WGM12); // 8-bit fast PWM mode
    
    RED_OC = ~0;
    GREEN_OC = ~0;

    TCCR1B |= (4 << CS10); // Prescaler = CLK/256
}


void i2c_init( void ) {
    
    // Match 0x74 OR 0x76
    TWAR = (0x74 << 1);
    TWAMR = (0x02 << 1);

    TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE); 
}


int main(void) {
    _delay_ms(100);
    setup_timer1();
    setup_timer0();
    i2c_init();
    sei();
    for (;;) {

    }
	return 0;
}

ISR(TWI_vect) {
    static uint8_t led;
    switch (TWSR) {
        
        case 0x60: // ADDR matched
            if ( TWDR == (0x74 << 1) ) {
                state = STATE_LCD;
            } else {
                state = STATE_LEDS;
                led = I_RED;
            }
            break;
        case 0x80: // Received byte
            if ( state == STATE_LCD ) {
                break;
            } else {
                led_vals[led++] = TWDR;
                led %= 3;
            }
            break;
        case 0xA0: // Received STOP
            if ( state == STATE_LEDS ) {
                RED_OC = ~led_vals[I_RED];
                GREEN_OC = ~led_vals[I_GREEN];
                BLUE_OC = ~led_vals[I_BLUE];
            }
            state = STATE_IDLE;
            break;
        /*
        case 0xA8:
            if ( TWDR == (0x74 << 1) + 1 ) {
                state = STATE_IDLE;
                // matched LCD
            } else {
                state = STATE_LEDS;
                led = I_RED;
            }
            break;
        */

    } 
    TWCR |= (1 << TWINT); 
}

