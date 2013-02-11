#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LCD_ADDR 0x74   // Address to match for LCD
#define NODE_ADDR 0x76  // Address to for LEDs/buttons 

// i2c state machines
#define MODES 4
#define MODE_IDLE 0
#define MODE_LCD 1
#define MODE_LEDS 2
#define MODE_BTNS 3
volatile uint8_t mode = MODE_IDLE;

void state_machine_idle( void );
void state_machine_lcd( void );
void state_machine_leds( void );
void state_machine_btns( void );
void (*state_machines[MODES])(void) = {
        state_machine_idle,
        state_machine_lcd,
        state_machine_leds,
        state_machine_btns
};

volatile struct {
    uint8_t led;
    uint8_t target[4];
    uint8_t current[3];
    uint8_t speed;
} state_leds;

#define HOLD_TIME 32 
volatile struct {
    uint8_t pressed;
    uint8_t held;
    int8_t count[8];
} state_btns;


// RGB LED definitions
#define RED_OC OCR1BL
#define GREEN_OC OCR1AL
#define BLUE_OC OCR0A
volatile uint8_t * led_ocr[3] = {&RED_OC, &GREEN_OC, &BLUE_OC};

#define I_RED   0
#define I_GREEN 1
#define I_BLUE  2
#define I_SPEED 3

// Button IO
// PORT C[0:3]
#define BTN_PWR     0
#define BTN_REW     1
#define BTN_PLAY    2
#define BTN_FWD     3
// PORT D[0:3]
#define BTN_BACK    4
#define BTN_ENTER   5
#define BTN_UP      6
#define BTN_DOWN    7

// Other IO
#define PI_PORT PORTB
#define PI_RST  0

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

/* timer2 used for input handling 
 */
void setup_timer2( void ) {

    ASSR |= (1 << AS2); // External 32 kHz xtal

    TCCR2A |= 0x2; // CTC Mode
    OCR2A = 128; // Compare match at 32 Hz
    TIMSK2 |= (1 << OCIE2A); // Compare Match A interrupt enable

    TCCR2B |= 0x2; // Prescaler /8

}

void setup_i2c( void ) {
    
    // Match 0x74 OR 0x76
    TWAR = (0x74 << 1);
    TWAMR = (0x02 << 1);

    TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE); 
}

void setup_io( void ) {
    
    // Buttons
    DDRC &= ~(0xF);
    DDRD &= ~(0xF);
    // Internal pull-ups. Buttons are active low
    PORTC |= 0xF;
    PORTD |= 0xF;

    // Pi reset FET
    DDRB |= (1 << PI_RST);
    PORTB &= ~(1 << PI_RST);

}

int main(void) {
    _delay_ms(100);
    setup_timer0();
    setup_timer1();
    setup_timer2();
    setup_i2c();
    setup_io();

    sei();
    for (;;) {
        _delay_ms(1000);
//        PI_PORT ^= (1 << PI_PWR);
    }
	return 0;
}

// Happens at 32 Hz
ISR(TIMER2_COMPA_vect) {
    uint8_t i, snapshot;

    // Button Handling
    snapshot = ~((PORTD << 4) | (PORTC & 0xF));
    state_btns.pressed = snapshot;
    for (i = 0; i < 6; i++, snapshot >>= 1) {
        if (snapshot & 1) {
            state_btns.count[i]++;
        } else {
            state_btns.count[i] = -10;
        }
        if (state_btns.count[i] > HOLD_TIME) {
            state_btns.held |= (1 << i);
            state_btns.count[i] = -10;
        }
    }
    if (state_btns.count[BTN_UP]) {
        state_btns.pressed |= (1 << BTN_UP);
    }
    if (state_btns.count[BTN_DOWN]) {
        state_btns.pressed |= (1 << BTN_DOWN);
    }
    
    // LED Handling
    for (i = 0; i < 3; i++) {
        int16_t distance = state_leds.target[i] - state_leds.current[i];
        if ((distance < 0) && (distance < -state_leds.speed) && 
                (state_leds.speed)) {
            state_leds.current[i] -= state_leds.speed;
        } else if ((distance > 0) && (distance > state_leds.speed) && 
                (state_leds.speed)) {
            state_leds.current[i] += state_leds.speed;
        } else {
                state_leds.current[i] = state_leds.target[i];
        }
        /*
        if ((distance) && (state_leds.speed % distance)) {
                state_leds.current[i] = state_leds.target[i] - 
                                        (state_leds.speed % distance);
        } else {
            state_leds.current[i] = state_leds.target[i];
        }
        */
        *(led_ocr[i]) = state_leds.current[i];
    }


    
}

void state_machine_idle( void ) {

    switch (TWSR) {
        case 0x60: // Write ADDR matched
            if ( TWDR == (LCD_ADDR << 1) ) {
                mode = MODE_LCD;
            } else {
                mode = MODE_LEDS;
                state_leds.led = I_RED;
            }
            break;
        case 0xA8: // Read ADDR matched
            if (TWDR == ((NODE_ADDR + 1) << 1)) {
                mode = MODE_BTNS;
            } else {
                // NAK?
            }
    }
}

void state_machine_lcd( void ) {
    switch (TWSR) {
        case 0x80: // Received byte
            // Do nothing
            break;
        case 0xA0: // Received STOP
            mode = MODE_IDLE;
            break;
    }
}

void state_machine_leds( void ) {

    switch (TWSR) {
        case 0x80: // Received byte
            state_leds.target[state_leds.led++] = TWDR;
            state_leds.led %= 4;
            break;
        case 0xA0: // Received STOP
            // Set up a move
            //if ( state == MODE_LEDS ) {
            //    RED_OC = ~led_vals[I_RED];
            //    GREEN_OC = ~led_vals[I_GREEN];
            //    BLUE_OC = ~led_vals[I_BLUE];
            //}
            state_leds.speed = state_leds.target[3];
            state_leds.led = 0;
            mode = MODE_IDLE;
            break;
    }

}

void state_machine_btns( void ) {
    uint8_t i = 0;

    switch (TWSR) {
        case 0xB8:
            if (i) {
                TWDR = state_btns.held;
                state_btns.held = 0;
                TWCR &= ~(1 << TWEA);
            } else {
                TWDR = state_btns.pressed;
                if (state_btns.count[BTN_UP] && 
                        (state_btns.pressed & (1 << BTN_UP))) {
                    state_btns.count[BTN_UP]--;
                }
                if (state_btns.count[BTN_DOWN] && 
                        (state_btns.pressed & (1 << BTN_DOWN))) {
                    state_btns.count[BTN_DOWN]--;
                }
                state_btns.pressed = 0;
            }
            i ^= 1;
            break;
        case 0xC0:
        case 0xC8:
            mode = MODE_IDLE;
            TWCR |= 1 << TWEA;
            break;
    }
}

ISR(TWI_vect) {
    (*state_machines[mode])();
 
    TWCR |= (1 << TWINT); 
}

