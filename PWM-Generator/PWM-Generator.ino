// encoder
#define state0 0b00000000 // encoder state 0
#define state1 0b00000001 // encoder state 1
#define state2 0b00000011 // encoder state 2
#define state3 0b00000010 // encoder state 3
#define enc_mask 0b00000011 // encoder pins 
#define encoder_ppr 2400 //number of discrete positions of the encoder, A 600ppr encoder will have 2400 points because of the two offset phases
int32_t enc_position = 0; // the position counter
byte pins_state = 0; // the current pin state
byte pins_prev = 0; // the last pin state
//int32_t encoder_target = 0; // target position for the encoder

// pulse control
#define pulse_mask 0b00001000 // pulse pin 
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

#define serial_update 1000
int last_millis = 0;
int delta = 0;

void select_prescalar(){
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

  if (duty_cycle_target <= 0.0 || duty_cycle_target > max_duty_cycle){
    in_range = 0;
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
  Serial.begin(115200); // start serial to send back data
  Serial.println("Starting");
  Serial.println("send dXXX to change duty cycle (%) or fXXX to change frequency (Hz)");
  
  // pin modes
  DDRB &= ~enc_mask; // set the pin mode for the encoder pins
  DDRB |= pulse_mask; // set the pin mode for the enable pin

  // interrupt setup for encoder
  SREG |= 0b10000000; // enable global interrupts
  PCICR |= 0b00000001; // enable port change interrupts on PCIE0 (PCINT7-0)
  PCMSK0 |= enc_mask; // enable interrupts on D8 and D9

  // Timer Interrupt
  TCCR2A = 0b00000010; // Put the timer in CTC mode
  TCCR2B = 0b00000100; // 64 prescalar
  TIMSK2 |= B00000110; // Enable compare interrupts
  select_prescalar();
}

///////////////////////////////////////////////////////////////////////////////////////////////
String thing;
void loop() {
  // use serial to recieve target positions
  if(Serial.available()>1){
    thing = Serial.readString();
    Serial.println(" ");
    //Serial.println(thing); 
    if (thing[0]==100){
      thing[0] = 32;
      duty_cycle_target = thing.toDouble();
    }   
    if (thing[0]==102){
      thing[0] = 32;
      frequency_target = thing.toDouble();
    }  
    select_prescalar();
  }
  
  delta = millis() - last_millis;
  if (delta > serial_update){
    last_millis = millis();
    // print current info
    Serial.print("Frequency: ");
    Serial.print(frequency_target);
    Serial.print(" Duty Cycle: ");
    Serial.print(duty_cycle_target);
    
    Serial.println("");
  }
  // wait before printing again
  delay(20);
}

///////////////////////////////////////////////////////////////////////////////////////////////

// encoder pin change interrupt
ISR(PCINT0_vect){ // create the interrupt for the encoder pins

  pins_state = PINB & enc_mask; // get the pin state for only the encoder pins
  if (pins_state == state0) { // check current state
    if (pins_prev == state3){ // if its moving forward increase the count
      enc_position--;
    } else if (pins_prev == state1){ // if its moving backward decrease the count
      enc_position++;
    }
  } else if (pins_state == state1) { // check current state
    if (pins_prev == state0){ // if its moving forward increase the count
      enc_position--;
    } else if (pins_prev == state2){ // if its moving backward decrease the count
      enc_position++;
    }
  } else if (pins_state == state2) { // check current state
    if (pins_prev == state1){ // if its moving forward increase the count
      enc_position--;
    } else if (pins_prev == state3){ // if its moving backward decrease the count
      enc_position++;
    }
  } else if (pins_state == state3) { // check current state
    if (pins_prev == state2){ // if its moving forward increase the count
      enc_position--;
    } else if (pins_prev == state0){ // if its moving backward decrease the count
      enc_position++;
    }
  }
  pins_prev = pins_state; // update the previous state
}

///////////////////////////////////////////////////////////////////////////////////////////////

// stepper pulse interrupt
ISR(TIMER2_COMPA_vect){
  if (in_range){
    PORTB |= pulse_mask; // set step high
  }
}

ISR(TIMER2_COMPB_vect){
  PORTB &= ~pulse_mask; // set step low
}
