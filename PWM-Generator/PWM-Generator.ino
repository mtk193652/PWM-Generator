#define USESERIAL
// Screen
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

// encoder 1
#define state0 0b00000000 // encoder state 0
#define state1 0b00000001 // encoder state 1
#define state2 0b00000011 // encoder state 2
#define state3 0b00000010 // encoder state 3
#define enc1_mask 0b00000011 // encoder pins 
#define enc1_button_mask 0b00000100 // encoder pins 
int32_t enc1_position = 0; // the position counter
int32_t enc1_position_last = 0; // the position counter
byte pins_state = 0; // the current pin state
byte enc1_pins_prev = 0; // the last pin state
uint8_t enc1_intervals = 3;
uint16_t enc1_deltas[] = {1, 100, 1000};
uint8_t enc1_delta_index = 0;
uint16_t enc1_delta = 1;
uint16_t debounce_time = 100;
int last_press = 0;

void update_enc1_delta(){
  enc1_delta_index++;
  if (enc1_delta_index >= enc1_intervals){
    enc1_delta_index = 0;
  }
  enc1_delta = enc1_deltas[enc1_delta_index];
}

// encoder 2
#define enc2_offset 3
#define enc2_mask 0b00011000 // encoder pins 
#define enc2_button_mask 0b00100000 // encoder pins 
double enc2_position = 0; // the position counter
double enc2_position_last = 0; // the position counter
byte enc2_pins_prev = 0; // the last pin state
uint8_t enc2_intervals = 3;
double enc2_deltas[] = {0.1, 1, 10};
uint8_t enc2_delta_index = 1;
double enc2_delta = 1;


void update_enc2_delta(){
  enc2_delta_index++;
  if (enc2_delta_index >= enc2_intervals){
    enc2_delta_index = 0;
  }
  enc2_delta = enc2_deltas[enc2_delta_index];
}

// pulse control
#define pulse_mask 0b10000000 // pulse pin 
#define timer_resolution 8
uint16_t timer_top = pow(2,timer_resolution)-1;

// timing 
double max_duty_cycle = 100;
double clock_correction = 1.005; // the oscilator may not be exactly 16 MHz, use this number to correct it 
double clock_frequency = 16.0*pow(10,6)/clock_correction;
double frequency_target = 0; 
double period = 1/frequency_target; 
double duty_cycle_target = 0;
bool in_range = 0;

uint16_t prescalar = 1; // current prescalar
uint16_t prescalars[] = {1, 8, 32, 64, 128, 256, 1024}; // current prescalar
uint16_t timer_count = 0; // current timer count
uint16_t pulse_count = 0;
double needed_prescalar = 0;
double clock_cycles = 0;

uint8_t set_high_mask = 0;
uint8_t set_low_mask = 0;


void clamp_timer(uint16_t val){
  if (val > timer_top){
    val = timer_top;
  }
}

void update_prescalar(){
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

#define serial_update 100
int last_millis = 0;
int delta = 0;

void select_prescalar(){
  if (frequency_target<0){
    frequency_target = 0;
  }
  period = 1/frequency_target; 
  clock_cycles = period * clock_frequency;
  needed_prescalar = clock_cycles/timer_top;
  in_range = 1;
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
    in_range = 0;
  }

  if (duty_cycle_target <= 0.0 || duty_cycle_target > max_duty_cycle || in_range == 0){
    set_high_mask = 0;
    set_low_mask = ~pulse_mask;
  } else if ((duty_cycle_target - max_duty_cycle)< .001) {
    set_high_mask = pulse_mask;
    set_low_mask = 0;
  } else {
    set_high_mask = pulse_mask;
    set_low_mask = ~pulse_mask;
  }

  double period_count = clock_cycles/prescalar;
  timer_count = period_count;
  clamp_timer(&timer_count);
  double high_count = period_count * duty_cycle_target / max_duty_cycle;
  pulse_count = high_count;
  clamp_timer(&high_count);
  
  if (pulse_count >= timer_count){ // ensures interrupt B occurrs before A
    pulse_count = timer_count-1;
  }

  OCR2A = timer_count;
  OCR2B = pulse_count;
  update_prescalar();
}

///////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  #ifdef USESERIAL
    Serial.begin(115200); // start serial to send back data
    Serial.println("Starting");
    Serial.println("send dXXX to change duty cycle (%) or fXXX to change frequency (Hz)");
  #endif
  
  // pin modes
  DDRB &= ~enc1_mask; // set the pin mode for the encoder pins
  DDRB &= ~enc1_button_mask; // set the pin mode for the encoder pins
  DDRB &= ~enc2_mask; // set the pin mode for the encoder pins
  DDRB &= ~enc2_button_mask; // set the pin mode for the encoder pins
  DDRD |= pulse_mask; // set the pin mode for the enable pin

  // interrupt setup for encoder
  SREG |= 0b10000000; // enable global interrupts
  PCICR |= 0b00000001; // enable port change interrupts on PCIE0 (PCINT7-0)
  PCMSK0 |= enc1_mask | enc1_button_mask; // enable interrupts on D8 and D9
  PCMSK0 |= enc2_mask | enc2_button_mask; // enable interrupts on D8 and D9

  // Timer Interrupt
  TCCR2A = 0b00000010; // Put the timer in CTC mode
  TCCR2B = 0b00000100; // 64 prescalar
  TIMSK2 |= B00000110; // Enable compare interrupts
  select_prescalar();

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();

}

///////////////////////////////////////////////////////////////////////////////////////////////
String thing;
void loop() {
  // use serial to recieve target positions
  #ifdef USESERIAL
    if(Serial.available()>1){
      thing = Serial.readString();
      Serial.println(" ");
      //Serial.println(thing); 
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
      select_prescalar();
    }
  #endif
  
  if (enc1_position != enc1_position_last){
    if (enc1_position<0){
      enc1_position = 0;
    }
    enc1_position_last = enc1_position;
    enc1_position = round(enc1_position/enc1_delta/4)*enc1_delta;
    frequency_target = enc1_position;
    select_prescalar();
  }

  if (enc2_position != enc2_position_last){
    if (enc2_position<0){
      enc2_position = 0;
    }
    enc2_position_last = enc2_position;
    enc2_position = round(enc2_position/enc2_delta/4)*enc2_delta;
    duty_cycle_target = enc2_position;
    select_prescalar();
  }
  

  delta = millis() - last_millis;
  if (delta > serial_update){
    last_millis = millis();
    // print current info
    #ifdef USESERIAL
      Serial.print("Frequency: ");
      Serial.print(frequency_target);
      Serial.print(" Duty Cycle: ");
      Serial.print(duty_cycle_target);
      Serial.print(" timer top: ");
      Serial.print(timer_count);
      Serial.print(" pulse compare: ");
      Serial.print(pulse_count);
      Serial.print(" prescalar: ");
      Serial.print(prescalar);
      Serial.print(" enc1_position: ");
      Serial.print(enc1_position);
      Serial.print(" enc1_delta: ");
      Serial.print(enc1_delta);
      Serial.print(" enc2_position: ");
      Serial.print(enc2_position);
      Serial.print(" enc2_delta: ");
      Serial.print(enc2_delta);
      
      Serial.println("");
      delay(100);
      
    #endif
    testdrawstyles();
  }

  
  // wait before printing again
  //delay(20);
}

///////////////////////////////////////////////////////////////////////////////////////////////

// encoder pin change interrupt
ISR(PCINT0_vect){ // create the interrupt for the encoder pins
  //Serial.println("*");

  int this_time = millis();
  pins_state = PINB & enc1_mask; // get the pin state for only the encoder pins
  if (pins_state == state0) { // check current state
    if (enc1_pins_prev == state3){ // if its moving forward increase the count
      enc1_position+=enc1_delta;
    } else if (enc1_pins_prev == state1){ // if its moving backward decrease the count
      enc1_position-=enc1_delta;
    }
  } else if (pins_state == state1) { // check current state
    if (enc1_pins_prev == state0){ // if its moving forward increase the count
      enc1_position+=enc1_delta;
    } else if (enc1_pins_prev == state2){ // if its moving backward decrease the count
      enc1_position-=enc1_delta;
    }
  } else if (pins_state == state2) { // check current state
    if (enc1_pins_prev == state1){ // if its moving forward increase the count
      enc1_position+=enc1_delta;
    } else if (enc1_pins_prev == state3){ // if its moving backward decrease the count
      enc1_position-=enc1_delta;
    }
  } else if (pins_state == state3) { // check current state
    if (enc1_pins_prev == state2){ // if its moving forward increase the count
      enc1_position+=enc1_delta;
    } else if (enc1_pins_prev == state0){ // if its moving backward decrease the count
      enc1_position-=enc1_delta;
    }
  }
  enc1_pins_prev = pins_state; // update the previous state

  if ((PINB & enc1_button_mask) && ((this_time - last_press) > debounce_time)){
    last_press = this_time;
    update_enc1_delta();
  }

  pins_state = (PINB & enc2_mask)>>enc2_offset; // get the pin state for only the encoder pins
  if (pins_state == state0) { // check current state
    if (enc2_pins_prev == state3){ // if its moving forward increase the count
      enc2_position-=enc2_delta;
    } else if (enc1_pins_prev == state1){ // if its moving backward decrease the count
      enc2_position+=enc2_delta;
    }
  } else if (pins_state == state1) { // check current state
    if (enc2_pins_prev == state0){ // if its moving forward increase the count
      enc2_position-=enc2_delta;
    } else if (enc2_pins_prev == state2){ // if its moving backward decrease the count
      enc2_position+=enc2_delta;
    }
  } else if (pins_state == state2) { // check current state
    if (enc2_pins_prev == state1){ // if its moving forward increase the count
      enc2_position-=enc2_delta;
    } else if (enc2_pins_prev == state3){ // if its moving backward decrease the count
      enc2_position+=enc2_delta;
    }
  } else if (pins_state == state3) { // check current state
    if (enc2_pins_prev == state2){ // if its moving forward increase the count
      enc2_position-=enc2_delta;
    } else if (enc2_pins_prev == state0){ // if its moving backward decrease the count
      enc2_position+=enc2_delta;
    }
  }
  enc2_pins_prev = pins_state; // update the previous state

  if ((PINB & enc2_button_mask) && ((this_time - last_press) > debounce_time)){
    last_press = this_time;
    update_enc2_delta();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////

// stepper pulse interrupt
ISR(TIMER2_COMPA_vect){
  PORTD |= set_high_mask; // set step high
}

ISR(TIMER2_COMPB_vect){
  PORTD &= set_low_mask; // set step low
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  if (frequency_target<10){
    display.print("  ");
  } else if (frequency_target<100){
    display.print(" ");
  } else if (frequency_target<1000){
    display.print("");
  } else if (frequency_target<10000){
    display.print(" ");
  }

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

  if (enc1_delta>=1000){
    display.print(" ");
  } else if (enc1_delta>=100){
    display.print("  ");
  } else if (enc1_delta>=10){
    display.print("   ");
  } else {
    display.print("    ");
  }
  
  display.print(enc1_delta);
  display.println(" Hz");
  
  if (duty_cycle_target<100){
    display.print(" ");
  }
  if (duty_cycle_target<10){
    display.print(" ");
  }
  display.print(duty_cycle_target);
  display.println(" %");

  if (enc2_delta<100){
    display.print(" ");
  }
  if (enc2_delta<10){
    display.print(" ");
  }
  display.print(enc2_delta);
  display.println(" %");
  display.println(" ");

  display.display();
}
