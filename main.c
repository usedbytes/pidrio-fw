/*
 *
 * main.c - Firmware for the Pidrio, Pidrive low-level interface board
 *   Copyright Brian Starkey 2013-2014 <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LCD_ADDR  0x3A  // Address to match for LCD
#define NODE_ADDR 0x3B  // Address to for LEDs/buttons 

#define STATE_BOOTING   0
#define STATE_ON        1
#define STATE_OFF       2
volatile uint8_t state = STATE_BOOTING;

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
// Set PB0 high to RESET
#define PI_PORT PORTB
#define PI_RST  0

// PD4 pulled high by Pi when on. Must NOT be set high (Pi not 5 V tolerant)
#define STATUS_PORT PORTD
// PI_STATUS on PCINT20
#define PI_STATUS 4
#define PWR_LED 7

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

    // Status detect and power LED
    DDRD |= (1 << PWR_LED);
    DDRD &= ~(1 << PI_STATUS);
    STATUS_PORT |= (1 << PWR_LED);

    // Pi status interrupt
    PCICR |= (1 << PCIE2); 
    PCMSK2 |= (1 << PCINT20);

}

void reset_pi( void ) {
    PI_PORT |= (1 << PI_RST);
    _delay_ms(10);
    PI_PORT &= ~(1 << PI_RST);
}


void boot_lighting( void ) {
    static uint8_t phase = 0;
    const uint8_t light_cycle[6][3] = {
        {0xFF, 0x00, 0x00},
        {0xFF, 0xFF, 0x00},
        {0x00, 0xFF, 0x00},
        {0x00, 0xFF, 0xFF},
        {0x00, 0x00, 0xFF},
        {0xFF, 0x00, 0xFF}
    };

    if ((state_leds.current[I_RED]   == light_cycle[phase][I_RED])   &&
        (state_leds.current[I_GREEN] == light_cycle[phase][I_GREEN]) &&
        (state_leds.current[I_BLUE]  == light_cycle[phase][I_BLUE])) {
        phase++;
        phase %= 6;
        state_leds.target[I_RED]     = light_cycle[phase][I_RED];
        state_leds.target[I_GREEN]   = light_cycle[phase][I_GREEN];
        state_leds.target[I_BLUE]    = light_cycle[phase][I_BLUE];
        state_leds.speed = 8;
    } else if ( (state_leds.current[I_RED]   == 0) &&
                (state_leds.current[I_GREEN] == 0) &&
                (state_leds.current[I_BLUE]  == 0)) {
        phase = 0;
        state_leds.target[I_RED]     = light_cycle[phase][I_RED];
        state_leds.target[I_GREEN]   = light_cycle[phase][I_GREEN];
        state_leds.target[I_BLUE]    = light_cycle[phase][I_BLUE];
        state_leds.speed = 8;
    }

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
          if (state == STATE_OFF) {
              if (state_btns.held & (1 << BTN_PWR)) {
                  reset_pi();
                  state = STATE_BOOTING;
                  STATUS_PORT |= (1 << PWR_LED);
                  state_btns.pressed = 0;
                  state_btns.held = 0;
              } else if ((state_leds.current[I_RED] == 0) &&
                  (state_leds.current[I_GREEN] == 0) &&
                  (state_leds.current[I_BLUE] == 0)) {
                  STATUS_PORT &= ~(1 << PWR_LED);
              }
          } else if (state == STATE_BOOTING) {
              boot_lighting();
          }

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
                state_btns.count[i] = HOLD_TIME;
            }
        } else {
            if (state_btns.held & (1 << i)) {
                pressed &= ~(1 << i);
                state_btns.held &= ~(1 << i);
            }
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
            //TWDR = 0x24;
            //state_btns.held = 0;
            //state_leds.target[3] = 3;
            //TWCR &= ~(1 << TWEA);
            break;
        case 0xC0:
        case 0xC8:
            mode = MODE_IDLE;
            TWCR |= (1 << TWEA);
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

/*
 * PCINT20 (PD7)
 */

ISR(PCINT2_vect) {
    uint8_t pi_status = PIND & (1 << PI_STATUS);
    
    if (!pi_status) {
        // Pi just went off
        state = STATE_OFF;
        state_leds.target[I_RED] = 0;
        state_leds.target[I_GREEN] = 0;
        state_leds.target[I_BLUE] = 0;
        state_leds.speed = 2;
    } else if (pi_status) {
        state = STATE_ON;
        STATUS_PORT |= (1 << PWR_LED);
    }
}

