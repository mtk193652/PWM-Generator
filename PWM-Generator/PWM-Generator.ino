//#define USESERIAL
//#define PERIODICUPDATE

// Screen
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // no reset pin
#define SCREEN_ADDRESS 0x3C // I2C Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //create the display object
bool update_screen = 0;

// encoder 
#define encClickCount 4
byte pins_state = 0; // the current pin state
uint16_t debounce_time = 100; // debounce time for buttons
int last_press = 0; // time of the last press
#define state0 0b00000000 // encoder state 0
#define state1 0b00000001 // encoder state 1
#define state2 0b00000011 // encoder state 2
#define state3 0b00000010 // encoder state 3

// encoder 1
#define enc1Mask 0b00000011 // encoder pins 
#define enc1ButtonMask 0b00000100 // encoder pins 
int enc1Position = 0; // the position counter
uint8_t enc1PinsPrev = 0; // the last pin state
uint8_t enc1Intervals = 3; //number of intervals
uint16_t enc1_interval_vec[] = {1, 100, 1000}; // intervals
uint8_t enc1_interval_index = 0; // index of the interval to use
uint16_t enc1_interval = 1;

// encoder 2
#define enc2_offset 3
#define enc2_mask 0b00011000 // encoder pins 
#define enc2_button_mask 0b00100000 // encoder pins 
int enc2_position = 0; // the position counter
uint8_t enc2_pins_prev = 0; // the last pin state
uint8_t enc2_intervals = 3; //number of intervals
double enc2_interval_vec[] = {0.1, 1, 10}; // intervals
uint8_t enc2_interval_index = 1; // index
double enc2_interval = 1;

// pulse control
#define pulse_mask 0b10000000 // pulse pin 
#define timer_resolution 8 // resolution of the timer in use
uint16_t timer_top = pow(2,timer_resolution)-1; // top value
uint8_t set_high_mask = 0; // mask for setting pulse high
uint8_t set_low_mask = 0; // mask for setting pulse low

// timing 
bool in_range = 1; // if there is not a good prescalar it will be out of range
double max_duty_cycle = 100; // max duty cycle 100%
double max_frequency = 200000; // max frequency
double clock_correction = 1.005; // the oscilator may not be exactly 16 MHz, use this number to correct it 
double clock_frequency = 16.0*pow(10,6)/clock_correction; // clock speed
double frequency_target = 80; // frequency to try to hit
double period = 1/frequency_target; // period of the square wave
double duty_cycle_target = 50; // duty cycle to try to hit
uint16_t prescalar = 1; // current prescalar
uint16_t prescalars[] = {1, 8, 32, 64, 128, 256, 1024}; // current prescalar
uint16_t timer_count = 0; // current timer count
uint16_t pulse_count = 0; // count for the match where pulse ends
double needed_prescalar = 0; // prescalar to use
double clock_cycles = 0; // clock cycles needed for the period of the square wave

// low frequency timing
#define minFrequency 10
uint32_t currentIterations = 0;
uint32_t iterationsInPeriod = 0;
uint32_t iterationsInPulse = 0;


#define update_delay 100
int last_millis = 0; // last time something was displayed
int delta = 0; // current time since last update

// SETUP ///////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  #ifdef USESERIAL
    Serial.begin(115200); // start serial to send back data
    Serial.println("Starting");
    Serial.println("send dXXX to change duty cycle (%) or fXXX to change frequency (Hz)");
  #endif
  
  // pin modes
  DDRB &= ~enc1Mask; // set the pin mode for the encoder pins
  DDRB &= ~enc1ButtonMask; // set the pin mode for the encoder pins
  DDRB &= ~enc2_mask; // set the pin mode for the encoder pins
  DDRB &= ~enc2_button_mask; // set the pin mode for the encoder pins
  DDRD |= pulse_mask; // set the pin mode for the pulse pin

  // interrupt setup for encoder
  SREG |= 0b10000000; // enable global interrupts
  PCICR |= 0b00000001; // enable port change interrupts on PCIE0 (PCINT7-0)
  PCMSK0 |= enc1Mask; // enable interrupts for enc1
  PCMSK0 |= enc1ButtonMask; // enable interrupts for enc1 button
  PCMSK0 |= enc2_mask; // enable interrupts for enc2
  PCMSK0 |= enc2_button_mask; // enable interrupts for enc2 button

  // Timer Interrupt
  TCCR2A = 0b00000010; // Put the timer in CTC mode
  TCCR2B = 0b00000100; // 64 prescalar
  TIMSK2 |= B00000110; // Enable compare interrupts A and 

  // Calculate initial values
  updateTimers();

  // start the display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  // Update the display
  updateScreen();

}

// LOOP ////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  #ifdef USESERIAL
    // check if there is a value recieved from serial
    if(Serial.available()>1){
      String thing;
      thing = Serial.readString();
      if (thing[0]==100){
        thing[0] = 32;
        duty_cycle_target = thing.toDouble();
        if (duty_cycle_target>max_duty_cycle){
          duty_cycle_target = max_duty_cycle;
        } else if (duty_cycle_target<0){
          duty_cycle_target = 0;
        }
      }   
      if (thing[0]==102){
        thing[0] = 32;
        frequency_target = thing.toDouble();
      }  
      updateTimers();
    }
  #endif
  
  // check if the frequency has changed
  if (abs(enc1Position) >= encClickCount ){
    double intermediate = frequency_target + floor(enc1Position/encClickCount)*enc1_interval; // round down the change by the current interval
    // clamp the result
    if (intermediate<0){
      intermediate = 0;
    } else if (intermediate>max_frequency){
      intermediate = max_frequency;
    }
    frequency_target = intermediate; // update the frequency target
    enc1Position = 0; // reset the encoder position
    updateTimers(); // update the prescalar
    update_screen = 1; // screen needs to be updated
  }

  // check if the duty cycle has changed
  if (abs(enc2_position) >= encClickCount ){
    double intermediate = duty_cycle_target + floor(enc2_position/encClickCount)*enc2_interval; // round down the change by the current interval
    // clamp the result
    if (intermediate<0){
      intermediate = 0;
    } else if (intermediate>max_duty_cycle){
      intermediate = max_duty_cycle;
    }
    duty_cycle_target = intermediate; // update the duty cycle target
    enc2_position = 0; // reset the encoder position
    updateTimers(); // update the prescalar
    update_screen = 1; // screen needs to be updated
  }
  
  // update the screen if needed
  if (update_screen){
    updateScreen();
    update_screen = 0;
  }

  #ifdef PERIODICUPDATE
    // make updates at a constant rate
    delta = millis() - last_millis;
    if (delta > update_delay){
      last_millis = millis();
      #ifdef USESERIAL
        Serial.print("Frequency: ");
        Serial.print(frequency_target);
        Serial.print(" Duty Cycle: ");
        Serial.print(duty_cycle_target);
        Serial.print(" enc1_interval: ");
        Serial.print(enc1_interval);
        Serial.print(" enc1Position: ");
        Serial.print(enc1Position);
        Serial.println("");
      #endif
      updateScreen();
    }
  #endif

}

// FUNCTIONS ///////////////////////////////////////////////////////////////////////////////////////////////

// encoder pin change interrupt
ISR(PCINT0_vect){ // create the interrupt for the encoder pins
  int this_time = millis();
  pins_state = PINB & enc1Mask; // get the pin state for only the encoder pins
  if (pins_state == state0) { // check current state
    if (enc1PinsPrev == state3){ // if its moving forward increase the count
      enc1Position++;
    } else if (enc1PinsPrev == state1){ // if its moving backward decrease the count
      enc1Position--;
    }
  } else if (pins_state == state1) { // check current state
    if (enc1PinsPrev == state0){ // if its moving forward increase the count
      enc1Position++;
    } else if (enc1PinsPrev == state2){ // if its moving backward decrease the count
      enc1Position--;
    }
  } else if (pins_state == state2) { // check current state
    if (enc1PinsPrev == state1){ // if its moving forward increase the count
      enc1Position++;
    } else if (enc1PinsPrev == state3){ // if its moving backward decrease the count
      enc1Position--;
    }
  } else if (pins_state == state3) { // check current state
    if (enc1PinsPrev == state2){ // if its moving forward increase the count
      enc1Position++;
    } else if (enc1PinsPrev == state0){ // if its moving backward decrease the count
      enc1Position--;
    }
  }
  enc1PinsPrev = pins_state; // update the previous state

  // check enc1 button
  if ((PINB & enc1ButtonMask) && ((this_time - last_press) > debounce_time)){
    last_press = this_time; // updatet the debounce time
    updateEnc1Interval(); // update the interval
  }

  pins_state = (PINB & enc2_mask)>>enc2_offset; // get the pin state for only the encoder pins
  if (pins_state == state0) { // check current state
    if (enc2_pins_prev == state3){ // if its moving forward increase the count
      enc2_position--;
    } else if (enc2_pins_prev == state1){ // if its moving backward decrease the count
      enc2_position++;
    }
  } else if (pins_state == state1) { // check current state
    if (enc2_pins_prev == state0){ // if its moving forward increase the count
      enc2_position--;
    } else if (enc2_pins_prev == state2){ // if its moving backward decrease the count
      enc2_position++;
    }
  } else if (pins_state == state2) { // check current state
    if (enc2_pins_prev == state1){ // if its moving forward increase the count
      enc2_position--;
    } else if (enc2_pins_prev == state3){ // if its moving backward decrease the count
      enc2_position++;
    }
  } else if (pins_state == state3) { // check current state
    if (enc2_pins_prev == state2){ // if its moving forward increase the count
      enc2_position--;
    } else if (enc2_pins_prev == state0){ // if its moving backward decrease the count
      enc2_position++;
    }
  }
  enc2_pins_prev = pins_state; // update the previous state

  // check enc2 button
  if ((PINB & enc2_button_mask) && ((this_time - last_press) > debounce_time)){
    last_press = this_time; // updatet the debounce time
    updateEnc2Interval(); // update the interval
  }
}

// Timer interrupt to start a pulse
ISR(TIMER2_COMPA_vect){
  if (true){
    PORTD |= set_high_mask; // start a pulse
  } else if (currentIterations >= iterationsInPeriod) {
    currentIterations = 0;
    PORTD |= set_high_mask; // start a pulse
  } else if (currentIterations == iterationsInPulse) {
    PORTD &= set_low_mask; //end a pulse
    currentIterations++;
  } else {
    currentIterations++;
  }
}

// Timer interrupt to end a pulse
ISR(TIMER2_COMPB_vect){
  if (in_range){
    PORTD &= set_low_mask; //end a pulse
  }
}

// Update the screen with current numbers
void updateScreen(void) {
  // setup to write text on screen
  display.clearDisplay(); 
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  // print frequency target with leading zeros
  if (frequency_target<10){
    display.print("  ");
  } else if (frequency_target<100){
    display.print(" ");
  } else if (frequency_target<1000){
    display.print("");
  } else if (frequency_target<10000){
    display.print(" ");
  }
  // use Hz or kHz as needed
  if (frequency_target>=100000){
    display.print(frequency_target/1000, 3);
    display.println("kHz");
  } else if (frequency_target>=1000){
    display.print(frequency_target/1000, 3);
    display.println(" kHz");
  } else {
    display.print(frequency_target);
    display.println(" Hz");
  }

  // print the frequency interval with leading zeros
  if (enc1_interval>=1000){
    display.print("  ");
  } else if (enc1_interval>=100){
    display.print("   ");
  } else if (enc1_interval>=10){
    display.print("    ");
  } else {
    display.print("     ");
  }
  display.print(enc1_interval);
  display.println(" Hz");
  
  // print the duty cycle with leading zeros
  if (duty_cycle_target<100){
    display.print(" ");
  }
  if (duty_cycle_target<10){
    display.print(" ");
  }
  display.print(duty_cycle_target);
  display.println(" %");

  // print the duty cycle interval with leading zeros
  if (enc2_interval<100){
    display.print(" ");
  }
  if (enc2_interval<10){
    display.print(" ");
  }
  display.print(enc2_interval);
  display.println(" %");
  display.println(" ");

  // update the display
  display.display();
}

// choose the value of the prescalar and timers
void updateTimers(){
  // clamp the frequency
  if (frequency_target < minFrequency){
    frequency_target = minFrequency;
  } else if (frequency_target>max_frequency){
    frequency_target = max_frequency;
  }

  // clamp the duty cycle
  if (duty_cycle_target<=0){ // duty cycle below zero
    duty_cycle_target = 0;
    set_high_mask = 0; // pwm pin can not be set high
    set_low_mask = ~pulse_mask;
  } else if (duty_cycle_target>=max_duty_cycle){ // duty cycle above the max
    duty_cycle_target = max_duty_cycle;
    set_high_mask = pulse_mask;
    set_low_mask = 0; // pwm pin can not be set low
  } else { // duty cycle in range
    set_high_mask = pulse_mask;
    set_low_mask = ~pulse_mask;
  }

  // figure out a prescalar to use
  period = 1/frequency_target; // find the period of the square wave
  clock_cycles = period * clock_frequency; // number of clock cycles in one period of the square wave
  needed_prescalar = clock_cycles/timer_top; // estimate the prescalar that would be needed

  // check each of the available prescalars to see if the needed prescalar is less than the available ones
  if (needed_prescalar<prescalars[0]){
    prescalar = prescalars[0];
  } else if (needed_prescalar<prescalars[1]){
    prescalar = prescalars[1];
  } else if (needed_prescalar<prescalars[2]){
    prescalar = prescalars[2];
  } else if (needed_prescalar<prescalars[3]){
    prescalar = prescalars[3];
  } else if (needed_prescalar<prescalars[4]){
    prescalar = prescalars[4];
  } else if (needed_prescalar<prescalars[5]){
    prescalar = prescalars[5];
  } else if (needed_prescalar<prescalars[6]){
    prescalar = prescalars[6];
  } else {
    prescalar = prescalars[6];
    in_range = 0; // if the largest prescalar does not work its out of range
  }
  
  if (true){
    // find the timer values
    timer_count = clock_cycles/prescalar; // the count for the entire period
    clampTimer(&timer_count); // clamp this value
    pulse_count = timer_count*duty_cycle_target/max_duty_cycle; // the count where the pwm pin will be high
    clampTimer(&pulse_count); // clamp this value
    
    // ensures interrupt B occurrs before A
    if (pulse_count >= timer_count){
      pulse_count = timer_count-1;
    }

    // update the timer values
    OCR2A = timer_count;
    OCR2B = pulse_count;
  } else {
    prescalar = prescalars[1];
    iterationsInPeriod = clock_cycles/prescalar/timer_top;
    iterationsInPulse = iterationsInPeriod*duty_cycle_target/max_duty_cycle;
  }

  // update the prescalar
  updatePrescalar();
}

// Change the interval of encoder 1
void updateEnc1Interval(){ // update the interval 
  enc1_interval_index++; // change to the next interval
  if (enc1_interval_index >= enc1Intervals){
    enc1_interval_index = 0;
  }
  enc1_interval = enc1_interval_vec[enc1_interval_index]; // update the interval
  update_screen = 1; // screen needs to be updated
}

// Change the interval of encoder 2
void updateEnc2Interval(){ // update the interval 
  enc2_interval_index++; // change to the next interval
  if (enc2_interval_index >= enc2_intervals){
    enc2_interval_index = 0;
  }
  enc2_interval = enc2_interval_vec[enc2_interval_index]; // update the interval
  update_screen = 1; // screen needs to be updated
}

// clamp the valuse given to the timer
void clampTimer(uint16_t val){ // ensures that the value of the timer is less than the top value
  if (val > timer_top){
    val = timer_top;
  }
}

// update the prescalar register 
void updatePrescalar(){ // update the prescalar
  if (prescalar == 1){
    TCCR2B = 0b00000001;
  } else if (prescalar == 8){
    TCCR2B = 0b00000010;
  } else if (prescalar == 32){
    TCCR2B = 0b00000011;
  } else if (prescalar == 64){
    TCCR2B = 0b00000100;
  } else if (prescalar == 128){
    TCCR2B = 0b00000101;
  } else if (prescalar == 256){
    TCCR2B = 0b00000110;
  } else if (prescalar == 1024){
    TCCR2B = 0b00000111;
  }
}