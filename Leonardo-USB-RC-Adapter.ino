/**************************************************************************************************
This sketch turns Arduino Leonardo (or Pro Micro) into a USB adapter for RC transmitter.

Prerequisite: 
Install Arduino Joystik library: https://github.com/MHeironimus/ArduinoJoystickLibrary

Connections:
- RC PPM out <--> Digital Pin 4 of Arduino Leonardo
- RC GND  <--> Arduino GND

Based on https://github.com/voroshkov/Leonardo-USB-RC-Adapter  and https://github.com/i--storm/rc-leonardo-joy , but enhanced as follows:
  Now use MHeironimus Joystick Library.
  Add ZAxis and RzAxis
Done by Denis Gayraud in December 2017, 

**************************************************************************************************/

#include "Arduino.h"
#include <avr/interrupt.h>
#include <Joystick.h>
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,2,0,
true,true,false,true,true,false,
true,true,false,false,false);
//Joystick_ Joystick;
// Use for Futaba transmitters (they have shifted center value and narrower range by default)
#define FUTABA

// Use to enable output of PPM values to serial
//#define SERIALOUT

#ifdef FUTABA
  #define RC_CHANNELS_COUNT 8
	#define STICK_HALFWAY 500
	#define STICK_CENTER 1530
	#define THRESHOLD 500
#else
  #define RC_CHANNELS_COUNT 6
	#define STICK_HALFWAY 500
	#define STICK_CENTER 1500
	#define THRESHOLD 100 // threshold is used to detect PPM values (added to range at both ends)
#endif

#define USB_STICK_VALUE_MAX 1000

#define MIN_PULSE_WIDTH (STICK_CENTER - STICK_HALFWAY)
#define MAX_PULSE_WIDTH (STICK_CENTER + STICK_HALFWAY)
#define NEWFRAME_PULSE_WIDTH 3000

// timer capture ICP1 pin corresponds to Leonardo digital pin 4
#define PPM_CAPTURE_PIN 4
#define LED_PIN 13

// for timer prescaler set to 1/8 of 16MHz, counter values should be
//  divided by 2 to get the number of microseconds
#define TIMER_COUNT_DIVIDER 2 

// this array contains the lengths of read PPM pulses in microseconds
volatile int16_t rcValue[RC_CHANNELS_COUNT];

// enum defines the order of channels
/*enum {
	ROLL,
	PITCH,  
	THROTTLE,
	YAW,
	AUX1,
	AUX2
};*/
enum {
  ROLL,
  PITCH,  
  THROTTLE,
  YAW,
  AUX1,
  AUX2,
  CH7,
  CH8
};

void setup() {
	setupPins();
	initTimer();
	Joystick.begin(false);

#ifdef SERIALOUT
	Serial.begin(115000);
#endif
}

void loop(){
	setControllerDataJoystick();
	Joystick.sendState();

#ifdef SERIALOUT	
	Serial.print(rcValue[YAW]); 
	Serial.print("\t");
	Serial.print(rcValue[THROTTLE]); 
	Serial.print("\t");
	Serial.print(rcValue[ROLL]); 
	Serial.print("\t");
	Serial.print(rcValue[PITCH]); 
	Serial.print("\t");
	Serial.print(rcValue[AUX1]); 
	Serial.print("\t");
	Serial.print(rcValue[AUX2]);
  Serial.print("\t");
  Serial.print(rcValue[CH7]); 
  Serial.print("\t");
  Serial.print(stickValue(rcValue[CH8]));
	Serial.println("\t");
#endif
}

void initTimer(void) { 
	// Input Capture setup
	// ICNC1: =0 Disable Input Capture Noise Canceler to prevent delay in reading
	// ICES1: =1 for trigger on rising edge
	// CS11: =1 set prescaler to 1/8 system clock (F_CPU)
	TCCR1A = 0;
	TCCR1B = (0<<ICNC1) | (1<<ICES1) | (1<<CS11);
	TCCR1C = 0;

	// Interrupt setup
	// ICIE1: Input capture 
	TIFR1 = (1<<ICF1); // clear pending
	TIMSK1 = (1<<ICIE1); // and enable
}

void setupPins(void){
	// Set up the Input Capture pin
	pinMode(PPM_CAPTURE_PIN, INPUT);
	digitalWrite(PPM_CAPTURE_PIN, 1); // enable the pullup

	pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode( LED_BUILTIN_RX, INPUT); /led RX off
}

// set joystick data in HID descriptor
void setControllerDataJoystick(){
	Joystick.setXAxis(stickValue(rcValue[ROLL]));
	Joystick.setYAxis(stickValue(rcValue[PITCH]));
  Joystick.setZAxis(stickValue(rcValue[AUX1]));

	Joystick.setRxAxis(stickValue(rcValue[CH7]));
	Joystick.setRyAxis(stickValue(rcValue[CH8]));
  Joystick.setRzAxis(stickValue(rcValue[AUX2]));

  Joystick.setThrottle(stickValue(rcValue[THROTTLE]));
  //Joystick.setAccelerator(stickValue[THROTTLE]);
  Joystick.setRudder(stickValue(rcValue[YAW]));
  //Joystick.setBrake(stickValue(STICK_CENTER));
  //Joystick.setSteering(stickValue(STICK_CENTER));*/
	Joystick.setButton(0, rcValue[AUX1] > STICK_CENTER);
	Joystick.setButton(1, rcValue[AUX2] > STICK_CENTER);
}

// Convert a value in the range of [Min Pulse - Max Pulse] to [0 - USB_STICK_VALUE_MAX]
uint16_t stickValue(int16_t rcVal) {
	return constrain(
		map(rcVal - MIN_PULSE_WIDTH, 
			0, MAX_PULSE_WIDTH - MIN_PULSE_WIDTH, 
			0, USB_STICK_VALUE_MAX
		),
		0, USB_STICK_VALUE_MAX
	);
}

uint16_t adjust(uint16_t diff, uint8_t chan) {
	// Here you can trim your rc values (e.g. if TX has no trims).
	
	// switch (chan) {
	//   case THROTTLE: return diff + 50;
	//   case YAW:      return diff + 60;
	//   case PITCH:    return diff + 60;
	//   case ROLL:     return diff + 90;
	//   case AUX1:     return diff + 10;
	// }

	//convert to microseconds (because of timer prescaler usage)
	return diff / TIMER_COUNT_DIVIDER;
}

ISR(TIMER1_CAPT_vect) {
	union twoBytes {
		uint16_t word;
		uint8_t  byte[2];
	} timeValue;

	uint16_t now, diff;
	static uint16_t last = 0;
	static uint8_t chan = 0;
	
	timeValue.byte[0] = ICR1L;    // grab captured timer value (low byte)
	timeValue.byte[1] = ICR1H;    // grab captured timer value (high byte)

	now = timeValue.word;
	diff = now - last;
	last = now;

	//all numbers are microseconds multiplied by TIMER_COUNT_DIVIDER (as prescaler is set to 1/8 of 16 MHz)
	if(diff > (NEWFRAME_PULSE_WIDTH * TIMER_COUNT_DIVIDER)) {
		chan = 0;  // new data frame detected, start again
	}
	else {
		if(diff > (MIN_PULSE_WIDTH * TIMER_COUNT_DIVIDER - THRESHOLD) 
			&& diff < (MAX_PULSE_WIDTH * TIMER_COUNT_DIVIDER + THRESHOLD) 
			&& chan < RC_CHANNELS_COUNT) 
		{
			rcValue[chan] = adjust(diff, chan); //store detected value
		}
		chan++; //no value detected within expected range, move to next channel
	}
}
