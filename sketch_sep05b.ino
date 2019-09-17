#include <avr/interrupt.h> 
#include <avr/io.h>
#include "EnableInterrupt.h"
#include <EEPROM.h>

#define SERIAL_PORT_SPEED 38400
#define RC_NUM_CHANNELS  10
#define status_led (1<<3)

#define alpha 0.986

#define RC_CH1  0 //AIL
#define RC_CH2  1 //ELE
#define RC_CH3  2 //THR
#define RC_CH4  3 //RUD
#define RC_CH5  4
#define RC_CH6  5
#define RC_CH7  6
#define RC_CH8  7
#define us_default 1500

#define X 0
#define Y 1
#define Z 2

#define RC_CH1_INPUT  2
#define RC_CH2_INPUT  3
#define RC_CH3_INPUT  4
#define RC_CH4_INPUT  5
#define RC_CH5_INPUT  6
#define RC_CH6_INPUT  7
#define RC_CH7_INPUT  8
#define RC_CH8_INPUT  11

#define RCin_offset 6


void Calibrate_RCT(void);
void decode_imu_data(char *);

uint16_t rc_values[RC_NUM_CHANNELS]={us_default,us_default,900,us_default,us_default,us_default,us_default,us_default,us_default,us_default};
uint32_t rc_start[RC_NUM_CHANNELS];
uint16_t calibration_value[6]={0,0,0,0,0,0,0};
volatile uint16_t rc_shared[RC_NUM_CHANNELS]={us_default,us_default,900,us_default,us_default,us_default,us_default,us_default,us_default,us_default};

uint16_t cal_RCusec_max[4]={0x00FF,0x00FF,0x00FF,0x00FF};
uint16_t cal_RCusec_min[4]={0x0FFF,0x0FFF,0x0FFF,0x0FFF};
volatile uint8_t i=0;

float cal_MAG[3]={0.0f,0.0f,0.0f};
float cal_ACCL[3]={0.0f,0.0f,0.0f};
float cal_GYRO[3]={0.0f,0.0f,0.0f};

char buf_imu[40]={0};
volatile char buf_isr[40]={0};
volatile char isr_cmplt_flag=0;
int eeAddress = 0;
int temp_addr = 16;

ISR(USART_RX_vect)
{	
isr_cmplt_flag=0;
cli();
	char c = UDR0;
	buf_isr[i++]=c;
	if(c=='\n'){
	i=buf_isr[i-1]=0;
	}
Serial.println(c);	
isr_cmplt_flag=1;
sei();

}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  PORTC^=status_led;
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  uint16_t rc_compare,temp;
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    rc_compare = (uint16_t)(micros() - rc_start[channel]);
    temp = rc_compare&0x0FFF;
    rc_shared[channel] = (uint16_t)((1-alpha)*rc_shared[channel] + (alpha * temp));    
  }
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
	
UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
 // Use 8-bit character sizes 
UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
 
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
	
  sei();

  while(!((buf_isr[0]=='4')||(buf_isr[0]=='6')||(buf_isr[0]=='8')||(buf_isr[0]=='A')))
  {
	if(isr_cmplt_flag==1)
	{
	  if(buf_isr[0]=='K')
	  {
	    eeAddress=0;

	    Calibrate_RCT(void);
	    //save the calibration value to eeprom
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_max[RC_CH1]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_max[RC_CH2]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_max[RC_CH3]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_max[RC_CH4]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_min[RC_CH1]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_min[RC_CH2]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_min[RC_CH3]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    EEPROM.put(eeAddress,cal_RCusec_min[RC_CH4]);  eeAddress+=sizeof(uint16_t);
	    delay(10);
	    Serial.println('1');
	  }

	  else if(buf_isr[0]=='R')
	  {
	    //read the calibration value from eeprom for RC
	    eeAddress=0;
	    EEPROM.get(eeAddress,cal_RCusec_max[RC_CH1]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH1]); Serial.print(',');
	    EEPROM.get(eeAddress,cal_RCusec_max[RC_CH2]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH2]); Serial.print(',');
	    EEPROM.get(eeAddress,cal_RCusec_max[RC_CH3]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH3]); Serial.print(',');
	    EEPROM.get(eeAddress,cal_RCusec_max[RC_CH4]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH4]); Serial.print(',');
	    EEPROM.get(eeAddress,cal_RCusec_min[RC_CH1]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH1]); Serial.print(',');
	    EEPROM.get(eeAddress,cal_RCusec_min[RC_CH2]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH2]); Serial.print(',');
	    EEPROM.get(eeAddress,cal_RCusec_min[RC_CH3]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH3]); Serial.print(',');
	    EEPROM.get(eeAddress,cal_RCusec_min[RC_CH4]);  eeAddress+=sizeof(uint16_t);
	    Serial.print(cal_RCusec_max[RC_CH4]); Serial.println(',');

	  }
	  else if(buf_isr[0]=='I')
	  {
	    //save the IMU calibration data	  
		decode_imu_data(&buf_isr[1]);
		eeAddress = temp_addr;

		    EEPROM.put(eeAddress,cal_MAG[X]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_MAG[Y]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_MAG[Z]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_ACCL[X]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_ACCL[Y]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_ACCL[Z]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_GYRO[X]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_GYRO[Y]);  eeAddress+=sizeof(float);
		    delay(10);
		    EEPROM.put(eeAddress,cal_GYRO[Z]);  eeAddress+=sizeof(float);
		    delay(10);
		  Serial.print('1');
	  }
	  else if(buf_isr[0]=='G')
	  {
	   //read calibration data for IMU 
		eeAddress = temp_addr;

		    EEPROM.get(eeAddress,cal_MAG[X]);  eeAddress+=sizeof(float);
		    Serial.print(cal_MAG[X]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_MAG[Y]);  eeAddress+=sizeof(float);
		    Serial.print(cal_MAG[Y]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_MAG[Z]);  eeAddress+=sizeof(float);
		    Serial.print(cal_MAG[Z]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_ACCL[X]);  eeAddress+=sizeof(float);
		    Serial.print(cal_ACCL[X]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_ACCL[Y]);  eeAddress+=sizeof(float);
		    Serial.print(cal_ACCL[Y]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_ACCL[Z]);  eeAddress+=sizeof(float);
		    Serial.print(cal_ACCL[Z]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_GYRO[X]);  eeAddress+=sizeof(float);
		    Serial.print(cal_GYRO[X]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_GYRO[Y]);  eeAddress+=sizeof(float);
		    Serial.print(cal_GYRO[X]); Serial.print(',');
		    EEPROM.get(eeAddress,cal_GYRO[Z]);  eeAddress+=sizeof(float);
		    Serial.print(cal_GYRO[Z]); Serial.println(',');	  
	  }
	}
  delay(10);
  }
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

void Calibrate_RCT(void)
{
	uint16_t rc_in_temp[6];
	uint16_t j;
		
	for(j=0;j<1000;j++)
	{
		memset(rc_in_temp,0,sizeof(rc_in_temp));	
    		rc_read_values();
   		memcpy(rc_in_temp,(const void *)rc_values,sizeof(rc_in_temp));
    
//Take Max value of pwm Max width time in micro_sec 
		
		if(rc_in_temp[RC_CH1]>cal_RCusec_max[RC_CH1])
		{
			cal_RCusec_max[RC_CH1]=rc_in_temp[RC_CH1];
		}	
		if(rc_in_temp[RC_CH2]>cal_RCusec_max[RC_CH2])
		{
			cal_RCusec_max[RC_CH2]=rc_in_temp[RC_CH2];
		}		
		if(rc_in_temp[RC_CH3]>cal_RCusec_max[RC_CH3])
		{
			cal_RCusec_max[RC_CH3]=rc_in_temp[RC_CH3];
		}		
		if(rc_in_temp[RC_CH4]>cal_RCusec_max[RC_CH4])
		{
			cal_RCusec_max[RC_CH4]=rc_in_temp[RC_CH4];
		}
		
//Take Min value of pwm Min width time in micro_sec 
		
		if(rc_in_temp[RC_CH1]<cal_RCusec_min[RC_CH1])
		{
			cal_RCusec_min[RC_CH1]=rc_in_temp[RC_CH1];
		}
		if(rc_in_temp[RC_CH2]<cal_RCusec_min[RC_CH2])
		{
			cal_RCusec_min[RC_CH2]=rc_in_temp[RC_CH2];
		}		
		if(rc_in_temp[RC_CH3]<cal_RCusec_min[RC_CH3])
		{
			cal_RCusec_min[RC_CH3]=rc_in_temp[RC_CH3];
		}		
		if(rc_in_temp[RC_CH4]<cal_RCusec_min[RC_CH4])
		{
			cal_RCusec_min[RC_CH4]=rc_in_temp[RC_CH4];
		}
		PORTC^=status_led;
		delay(100);
	}
	
	cal_RCusec_max[RC_CH1]+=RCin_offset; cal_RCusec_min[RC_CH1]-=RCin_offset;
	cal_RCusec_max[RC_CH2]+=RCin_offset; cal_RCusec_min[RC_CH2]-=RCin_offset;
	cal_RCusec_max[RC_CH3]+=RCin_offset; cal_RCusec_min[RC_CH3]-=RCin_offset;
	cal_RCusec_max[RC_CH4]+=RCin_offset; cal_RCusec_min[RC_CH4]-=RCin_offset;
  
  PORTC&=~status_led;
  delay(100);
}

void decode_imu_data(char *p)
{
sscanf(p,"%f,%f,%f,%f,%f,%f,%f,%f,%f",cal_MAG[X],cal_MAG[Y],cal_MAG[Z],cal_ACCL[X],cal_ACCL[Y],cal_ACCL[Z],cal_GYRO[X],cal_GYRO[Y],cal_GYRO[Z]);
}
		   
		   
   
	   
