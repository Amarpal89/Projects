/*
 * main.c
 * 							 ATTiny45
 * Reset 			-	Pin 1 |o--| Pin 8	-	VCC
 * blue	[B3]		-	Pin 2 |---| Pin 7	-	LED [B2]
 * pink	[B4]		-	Pin 3 |---| Pin 6 	-	yellow [B1]
 * GND				-	Pin 4 |---| Pin 5	-	orange [B0]
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>

#define F_CPU  1000000UL

static const int rotationSteps = 4096; 		// 0-4095 (4096 half-steps / revolution) possible 4076...
int delay = 350; 							// us delay between steps
static const uint8_t rotations = 3;			// rotations to do per wind (doubled on reverse motion)
static const int rotationsPerDay = 1440;	// rotations to do per hour (most calibres need 650 turns per day, double for single direction winding calibres)

volatile uint32_t milliseconds;
uint32_t timer;
static const int rotationsPerHour = rotationsPerDay / 24;
static const uint32_t pauseTime = ((3600000 / (rotationsPerHour)) * (2 * rotations));
volatile int8_t seqN = -1; 	// keeps track of step sequence
const char hSteps [8] = {0x09, 0x08, 0x18, 0x10, 0x12, 0x02, 0x03, 0x01}; // half step

uint32_t millis_get()
{
	uint32_t ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		ms = milliseconds;
	}
	return ms;
}

void seq(int seqNum){
	uint8_t pattern = hSteps[seqNum];
	uint8_t port = PORTB;
	port &= ~(0b00011011);
	port |= pattern;
	PORTB = port;
	_delay_us(delay);
}

void move(bool clockwise, uint16_t numSteps){
	for (uint16_t n=0; n<numSteps; n++){
		if (clockwise){
			seqN++;
			if (seqN > 7) seqN = 0; // roll over
			seq(seqN);
		}
		else{
			seqN--;
			if (seqN < 0) seqN = 7; // roll over
			seq(seqN);
		}
	}
}

void wind(){ //runs winder for preset rotations, CW then CCW, motor de-energised at end
	move(true,(rotations * rotationSteps)); // run forwards
	PORTB &= ~((1<<PORTB0) | (1<<PORTB1) | (1<<PORTB3) | (1<<PORTB4)); // motor off to reduce heat
	_delay_ms(200);
	move(false,(rotations * rotationSteps)); // run reverse to starting position
	PORTB &= ~((1<<PORTB0) | (1<<PORTB1) | (1<<PORTB3) | (1<<PORTB4)); // motor off
}

void setup(void){
	DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3) | (1<<DDB4); 	// Set Outputs for FET
	PORTB &= ~((1<<PORTB0) | (1<<PORTB1) | (1<<PORTB3) | (1<<PORTB4));	// motor off
	PORTB &= ~(1<<2); 													// LED off

	//POWER SAVING
	power_adc_disable();
	power_usi_disable();

	cli(); // stop interrupts
	TCCR0A = 0; // set entire TCCR0A register to 0
	TCCR0B = 0; // same for TCCR0B
	TCNT0  = 0; // initialize counter value to 0
	// set compare match register for 1000 Hz increments
	OCR0A = 124; // = 1000000 / (8 * 1000) - 1 (must be <256)
	TCCR0A |= (1 << WGM01);								// turn on CTC mode
	TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00); 	// Set CS02, CS01 and CS00 bits for 8 prescaler
	TIMSK |= (1 << OCIE0A); 							// enable timer compare interrupt
	sei(); // allow interrupts

	timer =  millis_get();
	_delay_ms(500);
	PORTB |= (1<<PORTB2); // LED on
	_delay_ms(2000);
	wind(); // initial wind
}

int main(void){
	setup();
	while(1){
		if(millis_get() > (timer + pauseTime)){
			wind();
			timer = millis_get(); // reset the timer
		}
		// timer overflow
		if(timer > millis_get()){
		  timer = millis_get(); // if timer wraps around reset it
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
	++milliseconds;
}
