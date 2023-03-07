// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include <Arduino.h>
#include <Wire.h>

int led = LED_BUILTIN;

#include <stdint.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <psyonic_hand_control/handVal.h>

#define NUM_CHANNELS 6
#define API_TX_SIZE	 15

typedef union api_i16_t
{
	int16_t i16[NUM_CHANNELS];
	uint8_t u8[NUM_CHANNELS*sizeof(int16_t)];
}api_i16_t;


/*Helper function to get the signed 8bit checksum*/
uint8_t get_checksum(uint8_t * arr, int size)
{
	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
}

/*Takes 6x floating point inputs for hand position arguments in DEGREES, and creates an
API frame to send out*/
void format_packet(float fpos_in[NUM_CHANNELS], uint8_t tx_buf[API_TX_SIZE])
{
	tx_buf[0] = 0x50; //hand slave address (use default)	
	tx_buf[1] = 0x10;
	api_i16_t pld;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		pld.i16[ch] = (int16_t)((fpos_in[ch] * 32767.0f) / 150.0f);
	}
	for(int i = 0; i < NUM_CHANNELS * sizeof(int16_t); i++)
	{
		tx_buf[i+2] = pld.u8[i];
	}
	tx_buf[API_TX_SIZE-1] = get_checksum((uint8_t*)tx_buf, API_TX_SIZE-1);  //full checksum
}

float position_converter(int16_t data, int16_t data2){
  int data_1 = data/16;
  int data_2 = data - data_1*16;
  int data_3 = data2/16;
  int data_4 = data2 - data_3*16;
  int pos =  16 * data_1 + 1 * data_2 + 16*16*16*data_3 + 16*16*data_2;

  float theta = ((pos / 32767.0f) * 150.0f);
  return theta;
}

float current_converter(int16_t data, int16_t data2){
  int data_1 = data/16;
  int data_2 = data - data_1*16;
  int data_3 = data2/16;
  int data_4 = data2 - data_3*16;
  int amp =  16 * data_1 + 1 * data_2 + 16*16*16*data_3 + 16*16*data_2;

  float current = ((amp / 7000.0f) * 0.54f);
  return current;
}


ros::NodeHandle nh;
std_msgs::Int16 val_msg;
std_msgs::Float32 val_msg_f;
psyonic_hand_control::handVal hand_msg;
// ros::Publisher pub("psyonic_hand_vals", &val_msg);
// ros::Publisher pub("psyonic_hand_vals", &val_msg_f);
ros::Publisher pub("psyonic_hand_vals", &hand_msg);
// float32[6] positions
// float32[6] currents
// float32[6] velocities
// float32[36] fingertips
float position[6];
float current[6];
float velocity[6];
float fingertip[36];

void setup()
{
  nh.getHardware()->setBaud(4000000);
  nh.initNode();   
  nh.advertise(pub);

  for(int i=0;i<6;i++){
    position[i] = 0;
    current[i]  = 0;
    velocity[i] = 0;

  }
  for(int i=0;i<36;i++){
    fingertip[i] = 0;
    
  }

  pinMode(led, OUTPUT);
  // Wire1.begin(); 
  // Wire1.setClock(400000);
              // join i2c bus (address optional for master)
  Serial4.begin(460800); //460800
  // Serial.begin(9600);       // start serial for output
  // Serial.println("Begin");
}

void read_values_1()
{
  int len_reception = 72; // 39:EXtended variant3 //72:EXtended variant1,2 //10: standard I2C
  int c[len_reception];
  int c_2[len_reception];
  int data[len_reception];
  float joint_angle[6];
  float joint_current[6];
  int sum =0;
  // Serial.println("Reading");
  // int rlen = Serial4.readBytes(c,len_reception);
  int count_buffer=0;
  int count_buffer_2=0;
  // Serial4.flush();
  // Serial4.flush();
  // val_msg_f.data = 1111;
  // pub.publish(&val_msg_f);
  // if( Serial4.read){
    for (int i=0;i<len_reception;i++)
    {
      if (Serial4.available()){
        // val_msg.data = Serial4.available();
        // pub.publish(&val_msg);
        c[i] = Serial4.read();
        sum = (sum + c[i]);
        // val_msg.data = c[i];
        // pub.publish(&val_msg);
        count_buffer =i;
      }
      else
        break;
    }
    Serial4.flush();
    for (int i=0;i<len_reception;i++)
    {
      if (Serial4.available()){
        // val_msg.data = Serial4.available();
        // pub.publish(&val_msg);
        c_2[i] = Serial4.read();
        // val_msg.data = c[i];
        // pub.publish(&val_msg);
        sum = (sum + c_2[i]);
        count_buffer_2 =i;
      }
      else
        break;
    }
    for(int i=0; i<len_reception;i++){
      if (i <(count_buffer_2 +1)){
        data[i] = c_2[i];
      }
      else{
        data[i] = c[i-(count_buffer_2 +1)];
      }
      // val_msg.data = data[i];
      // // val_msg.data = count_buffer;
      // pub.publish(&val_msg);

    }
    for(int i=0;i<6;i++){
      
      joint_angle[i] = position_converter(data[i*4+1],data[i*4+2]);
      joint_current[i] = current_converter(data[i*4+1+2],data[i*4+2+2]);
      position[i] = joint_angle[i];
      current[i]  = joint_current[i];
      hand_msg.positions[i] = position[i];
      hand_msg.currents[i] = current[i];
      // val_msg_f.data = joint_angle[i];
      // pub.publish(&val_msg_f);
    }
  // }
  // val_msg.data = Serial4.available();
  // pub.publish(&val_msg);
  sum = sum % 256;
  // val_msg.data = c[len_reception-1];
  // pub.publish(&val_msg);
  // Serial4.flush();
  pub.publish(&hand_msg);

}

void loop()
{
  
  float fpos[NUM_CHANNELS] = {15.f,15.f,15.f,15.f,15.f,-15.f};
	uint8_t tx_buf[API_TX_SIZE] = {0};
  
  float t_start = ((float)millis())*.001f;

  char input;
  while(1)
  {
        
    // *****************
    // sinusoidal movement example
    // but jerky though

    float t = ((float)millis())*.001f;
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			fpos[ch] = (0.5f*cos((t) + (float)ch)+0.5f)*30.f + 15.f;
    
		}
		fpos[5] = -fpos[5];
    // Serial.println("write");
		format_packet(fpos, tx_buf);
    Serial4.write(tx_buf, 15);
    // Serial.println("Read");
    read_values_1();
    // if (Serial.read()==0x50){
    //   read_values_1();
    // }
    // else
    //   Serial.clear(); 
    delay(2);
    
    nh.spinOnce();
   }
  
  }