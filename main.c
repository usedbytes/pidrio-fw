#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LCD_ADDR  0x3A  // Address to match for LCD
#define NODE_ADDR 0x3B  // Address to for LEDs/buttons 

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

volatile static struct {
    uint8_t led;
    uint8_t target[4];
    uint8_t current[3];
    uint8_t speed;
} state_leds;

#define HOLD_TIME 32 
volatile static struct {
    uint8_t pressed;
    uint8_t held;
    int8_t count[8];
    uint8_t scroll;
} state_btns;
#define CW 0x1
#define CCW 0x2

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

    TCCR0B |= (3 << CS00); // Prescaler CLK/64
}

/* timer1 used for red and green LEDs
 */
void setup_timer1( void ) {

    DDRB |= 0x06;

    TCCR1A = (0xF << COM1B0); // OC1A/B inverted mode
    TCCR1A |= (1 << WGM10); TCCR1B |= (1 << WGM12); // 8-bit fast PWM mode
    
    RED_OC = ~0;
    GREEN_OC = ~0;

    TCCR1B |= (3 << CS10); // Prescaler CLK/64 
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
    
    // Match 0x3A OR 0x3B
    TWAR = (LCD_ADDR << 1);
    TWAMR = (0x01 << 1);

    TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE); 
}

void setup_io( void ) {
    
    // Buttons
    DDRC &= ~(0xF);
    DDRD &= ~(0xF);
    // Internal pull-ups. Buttons are active low
    PORTC |= 0xF;
    PORTD |= 0xF;

    // Scroll wheel interrupts
    EICRA = 0x3;
    EIMSK = 0x1;

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
    DDRD |= 0x10;
    for (;;) {
        PORTD |= 0x10;
//        PI_PORT ^= (1 << PI_PWR);
    }
	return 0;
}

// Happens at 32 Hz
ISR(TIMER2_COMPA_vect) {
    uint8_t i, snapshot, pressed; 
    static uint8_t low = 0;

    // Button Handling
    snapshot = (((PIND << 4) | (PINC & 0xF) | 0xC0));
    pressed = (snapshot & low);
    low = snapshot = ~snapshot;
    for (i = 0; i < 6; i++, snapshot >>= 1) {
        if (snapshot & 1) {
            state_btns.count[i]++;
            if (state_btns.count[i] > HOLD_TIME) {
                state_btns.held |= (1 << i);
                state_btns.count[i] = -HOLD_TIME;
            }
        } else if (state_btns.count[i] < 0) {
            pressed &= ~(1 << i);
            state_btns.count[i] = 0;
        } else {
            state_btns.count[i] = 0;
        }
    }
    if (state_btns.scroll & CW) {
        state_btns.count[BTN_DOWN]++;
        state_btns.count[BTN_DOWN] %= 8;
    }
    if (state_btns.scroll & CCW) {
        state_btns.count[BTN_UP]++;
        state_btns.count[BTN_UP] %= 8;
    }
    state_btns.scroll = 0;
    if (state_btns.count[BTN_UP]) {
        pressed |= (1 << BTN_UP);
    }
    if (state_btns.count[BTN_DOWN]) {
        pressed |= (1 << BTN_DOWN);
    }
    state_btns.pressed |= pressed;

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
    
        *(led_ocr[i]) = ~state_leds.current[i];
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
            if (TWDR == ((NODE_ADDR << 1) + 1)) {
                mode = MODE_BTNS;
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
            } else {
                // NAK?
                TWDR = 0xFF;
                TWCR &= ~(1 << TWEA);
            }
            break;
        case 0xC0:
        case 0xC8:
            TWCR |= 1 << TWEA;
            break;
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
            state_leds.speed = state_leds.target[3];
            state_leds.led = 0;
            mode = MODE_IDLE;
            break;
    }

}

void state_machine_btns( void ) {

    switch (TWSR) {
        case 0xB8:
            TWDR = state_btns.held;
            state_btns.held = 0;
            TWCR &= ~(1 << TWEA);
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

/*
 * Scroll Wheel Handling
 * INT0 (PD2) __|```|___|```|___|```|_
 * INT1 (PD3) ____|```|___|```|___|```|_
 *            Clockwise Rotation ------>
 */
ISR(INT0_vect) {
    EIMSK = 0;
    uint8_t pins = PIND;
    if (EICRA & 0x1) { // Rising
        if (pins & 0x8) {
            state_btns.scroll |= CCW;
        } else {
            state_btns.scroll |= CW;
        }
        EICRA = 0x2;
    } else {
        if (pins & 0x8) {
            state_btns.scroll |= CW;
        } else {
            state_btns.scroll |= CCW;
        }
        EICRA = 0x3;
    }
    EIMSK = 0x1;
}
