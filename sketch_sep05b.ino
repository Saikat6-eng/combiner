#include "EnableInterrupt.h"

#define SERIAL_PORT_SPEED 38400
#define RC_NUM_CHANNELS  10
#define status_led (1<<3)

#define alpha 0.968

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5
#define RC_CH7  6
#define RC_CH8  7
#define uc_default 1500

#define RC_CH1_INPUT  2
#define RC_CH2_INPUT  3
#define RC_CH3_INPUT  4
#define RC_CH4_INPUT  5
#define RC_CH5_INPUT  6
#define RC_CH6_INPUT  7
#define RC_CH7_INPUT  8
#define RC_CH8_INPUT  11

uint16_t rc_values[RC_NUM_CHANNELS]={uc_default,uc_default,900,uc_default,uc_default,uc_default,uc_default,uc_default,uc_default,uc_default};
uint32_t rc_start[RC_NUM_CHANNELS];
uint32_t temp1;
volatile uint16_t rc_shared[RC_NUM_CHANNELS]={uc_default,uc_default,900,uc_default,uc_default,uc_default,uc_default,uc_default,uc_default,uc_default};

char c=0;

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  PORTC^=status_led;
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare&0x0FFF;
  }
}
uint16_t set_filter(uint16_t value_out, uint16_t value_in)
{
  value_out=(1-alpha)*value_out + (value_in*alpha);
  
return value_out;
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }
void calc_ch6() { calc_input(RC_CH6, RC_CH6_INPUT); }
void calc_ch7() { calc_input(RC_CH7, RC_CH7_INPUT); }
void calc_ch8() { calc_input(RC_CH8, RC_CH8_INPUT); }

void setup() {
  
  DDRC|=0b00001000;

  Serial.begin(SERIAL_PORT_SPEED);
 
  while (!Serial) {
    ;
  }
  delay(100);
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);
  pinMode(RC_CH7_INPUT, INPUT);
  pinMode(RC_CH8_INPUT, INPUT);
  pinMode(status_led,OUTPUT);

  PORTD|=0b11111100;
  PORTB|=0b00001001;

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);
  enableInterrupt(RC_CH7_INPUT, calc_ch7, CHANGE);
  enableInterrupt(RC_CH8_INPUT, calc_ch8, CHANGE);

  while(Serial.available()==0);
  c=Serial.read();
  while(!((c=='4')||(c=='6')||(c=='8')||(c=='A')))
  c=Serial.read();

}

void loop() {
rc_read_values();
delay(1);
  switch(c)
  {
    case '4':
      Serial.print(rc_values[RC_CH1]); Serial.print(',');
      Serial.print(rc_values[RC_CH2]); Serial.print(',');
      Serial.print(rc_values[RC_CH3]); Serial.print(',');    
      Serial.print(rc_values[RC_CH4]); Serial.print(','); 
      Serial.print("\r\n");
    break;
    case '6':
      Serial.print(rc_values[RC_CH1]); Serial.print(',');
      Serial.print(rc_values[RC_CH2]); Serial.print(',');
      Serial.print(rc_values[RC_CH3]); Serial.print(',');    
      Serial.print(rc_values[RC_CH4]); Serial.print(',');   
      Serial.print(rc_values[RC_CH5]); Serial.print(',');      
      Serial.print(rc_values[RC_CH6]); Serial.print(','); 
      Serial.print("\r\n");
    break;
    case 'A':
      Serial.print(rc_values[RC_CH7]); Serial.print(',');
      Serial.print(rc_values[RC_CH8]); Serial.print(','); 
      Serial.print("\r\n");
    break;
  }
delay(16);
}
