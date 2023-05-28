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
bool openHand = true;

#include <stdint.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <psyonic_hand_control/handVal.h>
#include <std_msgs/Float32MultiArray.h>

#define NUM_CHANNELS 6
#define API_TX_SIZE	 15

ros::NodeHandle nh;
std_msgs::Int16 val_msg;
std_msgs::Float32 val_msg_f;
psyonic_hand_control::handVal hand_msg;
ros::Publisher pub("psyonic_hand_vals", &hand_msg);

float position[6];
float current[6];
float velocity[6];
float fingertip[36];


float fpos[NUM_CHANNELS] = {30.f,30.f,30.f,30.f,30.f, 100.f};
void openHandCallback(const std_msgs::Float32MultiArray& msg) {
  for (int i = 0; i < msg.data_length && i < NUM_CHANNELS; i++) {
    fpos[i] = msg.data[i];
  }
}
ros::Subscriber<std_msgs::Float32MultiArray> openHandSub("psyonic_controller", openHandCallback);

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

float position_converter(uint8_t data, uint8_t data2){
  int16_t pos = (data2);
  pos = pos << 8;
  pos = pos | data;
  
  float theta = ((float(pos) / 32767.0f) * 150.0f);
  return theta;
}

float current_converter(uint8_t data, uint8_t data2){
  int16_t amp = (data2);
  amp = amp << 8;
  amp = amp | data;
  float current = ((float(amp) / 620.606079)); // BLE command Rv
  return current;
}

void unpack_8bit_into_12bit(uint8_t* arr, uint16_t* vals, int valsize)
{
  for(int i = 0; i < valsize; i++)
    vals[i] = 0; //Clear the buffer before loading with |=
  for(int bidx = valsize * 12 - 4; bidx >= 0; bidx -= 4)
  {
    int validx = bidx / 12;
    int arridx = bidx / 8;
    int shift_val = (bidx % 8);
    vals[validx] |= ((arr[arridx] >> shift_val) &0x0F) << (bidx % 12);
  }
}

float tipforce_converter_1(uint8_t data, uint8_t data2, uint8_t data3){ // 12bit
  int valsize = 2;
  int16_t vals[valsize];
  uint8_t arr[3] = {data,data2,data3};
  for(int i = 0; i < valsize; i++)
    vals[i] = 0; //Clear the buffer before loading with |=
  for(int bidx = valsize * 12 - 4; bidx >= 0; bidx -= 4)
  {
    int validx = bidx / 12;
    int arridx = bidx / 8;
    int shift_val = (bidx % 8);
    vals[validx] |= ((arr[arridx] >> shift_val) &0x0F) << (bidx % 12);
  }
  
  float V = float(vals[0]) * 3.3/4096;
  float R = 33000 / V + 10000;
  float C1 = 121591.0;
  float C2 = 0.878894;
  float force = C1/R + C2;  
  return force;
}
float tipforce_converter_2(uint8_t data, uint8_t data2, uint8_t data3){ // 12bit
  int valsize = 2;
  int16_t vals[valsize];
  uint8_t arr[3] = {data,data2,data3};
  for(int i = 0; i < valsize; i++)
    vals[i] = 0; //Clear the buffer before loading with |=
  for(int bidx = valsize * 12 - 4; bidx >= 0; bidx -= 4)
  {
    int validx = bidx / 12;
    int arridx = bidx / 8;
    int shift_val = (bidx % 8);
    vals[validx] |= ((arr[arridx] >> shift_val) &0x0F) << (bidx % 12);
  }
  
  float V = float(vals[1]) * 3.3/4096;
  float R = 33000 / V + 10000;
  float C1 = 121591.0;
  float C2 = 0.878894;
  float force = C1/R + C2;
  return force;
}


void setup()
{
  nh.getHardware()->setBaud(4000000);
  nh.initNode();   
  nh.advertise(pub);
  nh.subscribe(openHandSub);
  for(int i=0;i<6;i++){
    position[i] = 0;
    current[i]  = 0;
    velocity[i] = 0;

  }
  for(int i=0;i<36;i++){
    fingertip[i] = 0;
  }
  pinMode(led, OUTPUT);
  Serial1.begin(460800);
  digitalWrite(led, HIGH);
}
int8_t add(uint8_t data[72]){
  int8_t x = 0;
  for(int i = 0; i < 72; i++){
     x += (int8_t) data[i];
  }
  return x;
}
void read_values_1()
{
  int len_reception = 72; // 39:EXtended variant3 //72:EXtended variant1,2 //10: standard I2C
  uint8_t c[len_reception];
  uint8_t c_2[len_reception];
  uint8_t data[len_reception];
  float joint_angle[6];
  float joint_current[6];
  float tip_force[36];
  int8_t sum =0;
  for(int i=0;i<6;i++){
    position[i] = 0;
    current[i]  = 0;
    velocity[i] = 0;
    joint_angle[i]=0;
    joint_current[i]=0;

  }
  for(int i=0;i<36;i++){
    fingertip[i] = 0;
    tip_force[i]=0;
  }
  for(int i=0;i<len_reception;i++){
    c[i] = 0;
    c_2[i]=0;
    data[i]=0;
  }
  byte buffer[72];
  Serial1.addMemoryForRead(buffer,sizeof(buffer));
  int count_buffer=0;
  int count_buffer_2=0;
  int time = 1000;
  int len1 = Serial1.available();
  for (int i=0;i<len1;i++)
  {
      c[i] = Serial1.read();
      count_buffer =i;
  }
  delayMicroseconds(time);
  Serial1.flush();
  delayMicroseconds(time);
  int len2 = Serial1.available();
  for (int i=0;i<len2;i++)
  {
      c_2[i] = Serial1.read();
      count_buffer_2 =i;
  }
  for(int i=0; i<len_reception;i++){
    if (i <(count_buffer_2 +1)){
      data[i] = c_2[i];
    }
    else{
      data[i] = c[i-(count_buffer_2 +1)];
    }
  }
  sum = add(data);
  for(int i=0;i<6;i++){
    
    joint_angle[i] = position_converter(data[i*4+1],data[i*4+2]);
    joint_current[i] = current_converter(data[i*4+1+2],data[i*4+2+2]);
    if(i<5)
    {
      fingertip[i*6]= tipforce_converter_1(data[i*9+25],data[i*9+25+1],data[i*9+25+2]);
      fingertip[i*6+1]= tipforce_converter_2(data[i*9+25],data[i*9+25+1],data[i*9+25+2]);
      fingertip[i*6+2]= tipforce_converter_1(data[i*9+25+3],data[i*9+25+4],data[i*9+25+5]);
      fingertip[i*6+3]= tipforce_converter_2(data[i*9+25+3],data[i*9+25+4],data[i*9+25+5]);
      fingertip[i*6+4]= tipforce_converter_1(data[i*9+25+6],data[i*9+25+7],data[i*9+25+8]);
      fingertip[i*6+5]= tipforce_converter_2(data[i*9+25+6],data[i*9+25+7],data[i*9+25+8]);            
    }         
    position[i] = joint_angle[i];
    current[i]  = joint_current[i];
    hand_msg.positions[i] = position[i];
    hand_msg.currents[i] = current[i];
  }
  fingertip[35] = data[0];
  fingertip[34] = data[71];
  fingertip[32] = data[70];
  fingertip[31] = data[69];
  sum = sum % 256;
  fingertip[33] = sum;

  for(int i=0;i<36;i++){
    hand_msg.fingertips[i] = fingertip[i];
  }
  pub.publish(&hand_msg);
}

void loop()
{
  
  uint8_t tx_buf[API_TX_SIZE] = {0};
  format_packet(fpos, tx_buf);
  Serial1.write(tx_buf, 15);
  read_values_1();
  nh.spinOnce();

	// uint8_t tx_buf[API_TX_SIZE] = {0};
  // format_packet(fpos, tx_buf);
  // Serial1.write(tx_buf, 15);
  // digitalWrite(led, LOW);

  // // int time = 1000;

  // // float fpos[NUM_CHANNELS] = {30.f,30.f,30.f,30.f,0.f,0.f};
	// // uint8_t tx_buf[API_TX_SIZE] = {0};
  
  // // // float t = ((float)millis())*.001f;
  // // // for(int ch = 0; ch < 1; ch++)
  // // // {
  // // //   fpos[ch] = (1.5f*cos((t) + (float)ch)+0.5f)*30.f + 30.f;
  // // //   // fpos[ch] = (0.5f*cos((t) )+0.5f)*30.f + 15.f;
  
  // // // }
  
  // // fpos[1] = 0;
  // // fpos[2] = 0;
  // // // fpos[3] = 0;
  // // fpos[3] = fpos[0];
  // // fpos[4] = 0;
  // // fpos[5] = 0;
  // // format_packet(fpos, tx_buf);
  // // Serial1.write(tx_buf, 15);
  // // // delayMicroseconds(time);
  // // // Serial1.flush();
  // // // delayMicroseconds(time);
  // // // delay(1);
  // // read_values_1();
  // // nh.spinOnce();
  // // // digitalWrite(led, HIGH);

  // // // read_values_1();
  // // // nh.spinOnce();
 
}