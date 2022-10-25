/*
 * ENCODER:           PD2(INT0) & PD3(INT1)
 * ENCODER SWITCH:    PD4
 * SK6812 x60:        PB3
 * W: BW 6000-7000K (6500k)     -   6.0 lm
 * W: WS 2700-3200K (3000k)     -   6.0 lm
 * A: Amber 1800-2000K (1900k)  -   1.5 lm
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#define ENC_SW  		DDD4
#define LIGHTPIN        PB3
#define NUMPIXELS       30
Adafruit_NeoPixel ring(NUMPIXELS, LIGHTPIN, NEO_BRG + NEO_KHZ800);

#define BRIGHTNESS_Address  10
#define TEMP_Address        11

#define DEBUG   0
#define F_CPU   16000000UL

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

int8_t BRIGHTNESS; //0-100 brightness value
int8_t TEMPERATURE; //0-47 colour temp (1800K-6500K)
int8_t mix[] = {0, 100, 10};   // array for color temp, saturation, intensity
uint8_t awc[] = {0, 0, 0};
bool changeLED; //variable for change in LED settings
uint8_t currentMenu = 0; //0=brightness 1=temperature
bool eepromUPDATE; //variable for change in LED settings

uint32_t timer;
uint32_t timer2;
static int8_t encoder_rotation = 0;
uint8_t clockwise;
uint8_t steps = 2;
volatile uint8_t old_AB = 3;
volatile int8_t encval = 0;   //encoder value
static const int8_t enc_states [] PROGMEM = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};  //encoder lookup table

static volatile uint32_t milliseconds;

void setLED(void){
  // create a single color from temp, intensity:
  if(currentMode ==0){
      colourToLED();
  }
  else{
      powerToLED();
  }
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    ring.setPixelColor(i, awc[0], awc[1], awc[2]);
    ring.show();   // Send the updated pixel colors to the hardware.
  }
  double calc = (awc[0] + awc[1] + awc[2]) / (3 *2.55);
  POWER = (uint8_t) calc;
}

void colourToLED(void){ //convert temp/intensity to AWC
  double a, w, c;

  if(TEMPERATURE <= 13){
    a = 255 - (6*TEMPERATURE);
	w = TEMPERATURE * 20;
    w = constrain(w, 0, 255);
    c = 0;
  }
  if(TEMPERATURE >13 ){
    a = 0;
    w = 255 - ((TEMPERATURE - 13) * 20);
    w = constrain(w, 0, 235);
    c = ((TEMPERATURE - 12) * 7) + 3;
    c = constrain(c, 0, 255);
  }
  a = a * ((double)BRIGHTNESS / 100);
  w = w * ((double)BRIGHTNESS / 100);
  c = c * ((double)BRIGHTNESS / 100);
  awc[0] = (uint8_t) a;
  awc[1] = (uint8_t) w;
  awc[2] = (uint8_t) c;
}

void powerToLED(void){
  double a;

  a = ((double)BRIGHTNESS * 2.55);
  awc[0] = (uint8_t) a;
  awc[1] = (uint8_t) a;
  awc[2] = (uint8_t) a;
}


void setup() {
	  cli(); //disable interrupts
	  BRIGHTNESS = EEPROM.read(BRIGHTNESS_Address);
	  if(BRIGHTNESS < 10){
	    BRIGHTNESS = 10;
	  }
	  TEMPERATURE = EEPROM.read(TEMP_Address);
	  mix[0] = TEMPERATURE;
	  mix[1] = BRIGHTNESS;

	  //---ENCODER---//
	  //Encoder Sw
	  DDRD &= ~(1<<DDD4);  //Set Encoder Switch input
	  DDRD &= ~(1<<DDD2);    //Configure PORTD pin 0 as an input
	  DDRD &= ~(1<<DDD3);    //Configure PORTD pin 1 as an input
	  PORTD |= (1<<PORTD2) | (1<<PORTD3) | (1<<PORTD4); //Enable Pullups
	  EICRA &= ~(1<<ISC01); // trigger INT0 on rising or falling edge
	  EICRA |= (1<<ISC00);
	  EICRA &= ~(1<<ISC11); // trigger INT1 on rising or falling edge
	  EICRA |= (1<<ISC10);
	  EIMSK |= (1<<INT0) | (1<<INT1);

	  old_AB = ( PIND & 0x03 );  //lookup table index
	  sei(); //Sets Global Interrupt Enable bit in SREG

	  //LIGHT
	  ring.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
	  ring.clear(); // Set all pixel colors to 'off'
	  setLED();
	  timer = millis();
	  timer2 = millis();
	  idletimer = millis();

}

void saveSettings(void){
  EEPROM.update(BRIGHTNESS_Address, BRIGHTNESS);
  EEPROM.update(TEMP_Address, TEMPERATURE);
  EEPROM.update(MODE_Address, currentMode);
}

void setLED(void){
  // create a single color from temp, intensity:
  colourToLED();
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    ring.setPixelColor(i, awc[0], awc[1], awc[2]);
    ring.show();   // Send the updated pixel colors to the hardware.
  }
  double calc = (awc[0] + awc[1] + awc[2]) / (3 *2.55);
  }

void colourToLED(void){ //convert temp/intensity to AWC
    double a, w, c;

    if(TEMPERATURE <= 13){
      a = 255 - (6*TEMPERATURE);
  	w = TEMPERATURE * 20;
      w = constrain(w, 0, 255);
      c = 0;
    }
    if(TEMPERATURE >13 ){
      a = 0;
      w = 255 - ((TEMPERATURE - 13) * 20);
      w = constrain(w, 0, 235);
      c = ((TEMPERATURE - 12) * 7) + 3;
      c = constrain(c, 0, 255);
    }
    a = a * ((double)BRIGHTNESS / 100);
    w = w * ((double)BRIGHTNESS / 100);
    c = c * ((double)BRIGHTNESS / 100);
    awc[0] = (uint8_t) a;
    awc[1] = (uint8_t) w;
    awc[2] = (uint8_t) c;
}


void read_encoder(void){
	old_AB <<=2;  //remember previous state
	uint8_t state = ( PIND & 0x0c );
	state >>=2;
	old_AB |= state;
	encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
	if(encval >= steps){
	  encoder_rotation--;
	  encval %= steps;
	}
	else if(encval <= -steps){
	  encoder_rotation++;
	  encval %= steps;
  }
}

ISR(INT0_vect){
	uint8_t SaveSREG = SREG;   // save interrupt flag
	cli();
	read_encoder();
	SREG = SaveSREG;   // restore the interrupt flag
}

ISR(INT1_vect){
	uint8_t SaveSREG = SREG;   // save interrupt flag
	cli();
	read_encoder();
	SREG = SaveSREG;   // restore the interrupt flag
}

void loop() {
    //----------- Encoder ------------ //
    if(encoder_rotation != 0){
      constrain(encoder_rotation, -20, 20);
      if(encoder_rotation > 0){
        clockwise = 1;
        encoder_rotation--;
      }
      else
      {
        clockwise = 0;
        encoder_rotation++;
      }
      switch(currentMenu){
        case 0: //brightness
          if(clockwise){
            BRIGHTNESS++;
            BRIGHTNESS = constrain(BRIGHTNESS, 0, 100);
            changeLED = 1;
          }
          else{
            BRIGHTNESS--;
            BRIGHTNESS = constrain(BRIGHTNESS, 0, 100);
            changeLED = 1;
          }
          break;
        case 1: //temperature
          if(clockwise){
            TEMPERATURE++;
            TEMPERATURE = constrain(TEMPERATURE, 0, 47);
            changeLED = 1;
          }
          else{
            TEMPERATURE--;
            TEMPERATURE = constrain(TEMPERATURE, 0, 47);
            changeLED = 1;
          }
          break;
      }
      delay(5);
    }

    if(!(PIND & 0x20)){
      switch(currentMenu){
        case 0: //brightness
          currentMenu = 1;
          break;
        case 1: //temperature
          currentMenu = 0;
          break;
      }
      while(!(PIND & 0x20)){
        //hold loop for pressed button
      }
      delay(100); //debounce
    }
    if(changeLED && (((millis() - timer) > 1000)) || (encoder_rotation == 0)){
      timer = millis(); // reset the timer
      eepromUPDATE = 1;
      changeLED = 0;
      setLED();
    }

    //update settings in eeprom
    if(eepromUPDATE && ((millis() - timer2) > 5000)){
      timer2 = millis(); // reset the timer
      eepromUPDATE = 0;
      saveSettings();
    }

    //----timer overflow----//
    if(timer > millis()){
      timer = millis(); // if millis() or timer wraps around, we'll just reset it
    }
    if(timer2 > millis()){
      timer2 = millis();
    }
  }
