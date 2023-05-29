/* Authors: Jooyoung Hong, Sankalp Yamsani, Chaerim Moon, Kazuki Shin 
   Documentation and Example code for Psyonic Here: https://github.com/psyonicinc/ability-hand-api
*/
#include <Arduino.h>
#include <Wire.h>
#include "helper_functions.h"
#include <stdint.h>
#include <math.h>

// Call back Function Header
void moveHandCallback(const std_msgs::Float32MultiArray& msg);


// Setup ROS Related variables
ros::NodeHandle nh;
psyonic_hand_control::handVal hand_msg;
ros::Publisher pub("psyonic_hand_vals", &hand_msg);
ros::Subscriber<std_msgs::Float32MultiArray> openHandSub("psyonic_controller", moveHandCallback);

// Setup finger position variable
float fpos[NUM_CHANNELS] = {15.f,15.f,15.f,15.f,15.f, -15.f};

uint8_t rx_buffer[API_RX_SIZE];

// GET RID OF ME
// float position[6];
// float current[6];
// float velocity[6];
// float fingertip[36];


/*Moves Fingers to the position user has set*/
void moveHandCallback(const std_msgs::Float32MultiArray& msg){
  for (uint32_t i = 0; i < msg.data_length && i < NUM_CHANNELS; i++) {
    fpos[i] = msg.data[i];
  }
}

void read_values(){

  int8_t sum = 0;
  uint8_t data[API_RX_SIZE];
  int len1 = Serial1.available();
  // hand_msg.fingertips[33] = len1;
  // pub.publish(&hand_msg);
  // return;
  // if(len1 != API_RX_SIZE){
  //   return;
  // }

  for (int i = 0; i < API_RX_SIZE; i++){
    data[i] = Serial1.read();
  }
  
  sum = get_checksum(data, API_RX_SIZE-1);
  sum += (int8_t)data[API_RX_SIZE-1];

  // if(sum != 0){
  //   return;
  // }

  for(int i = 0; i < NUM_CHANNELS; i++){
    
    hand_msg.positions[i] = position_converter(data[i*4+1],data[i*4+2]);
    hand_msg.currents[i] = current_converter(data[i*4+1+2],data[i*4+2+2]);
    if(i < 5)
    {
      hand_msg.fingertips[i*6] = tipforce_converter_1(data[i*9+25],data[i*9+25+1],data[i*9+25+2]);
      hand_msg.fingertips[i*6+1] = tipforce_converter_2(data[i*9+25],data[i*9+25+1],data[i*9+25+2]);
      hand_msg.fingertips[i*6+2] = tipforce_converter_1(data[i*9+25+3],data[i*9+25+4],data[i*9+25+5]);
      hand_msg.fingertips[i*6+3] = tipforce_converter_2(data[i*9+25+3],data[i*9+25+4],data[i*9+25+5]);
      hand_msg.fingertips[i*6+4] = tipforce_converter_1(data[i*9+25+6],data[i*9+25+7],data[i*9+25+8]);
      hand_msg.fingertips[i*6+5] = tipforce_converter_2(data[i*9+25+6],data[i*9+25+7],data[i*9+25+8]);            
    }         
  }
  hand_msg.fingertips[35] = len1;
  hand_msg.fingertips[34] = data[71];
  hand_msg.fingertips[32] = data[70];
  hand_msg.fingertips[31] = data[69];
  sum = sum % 256;
  hand_msg.fingertips[33] = sum;

  pub.publish(&hand_msg);
}

void setup()
{
  nh.getHardware()->setBaud(4000000);
  nh.initNode();   
  nh.advertise(pub);
  nh.subscribe(openHandSub);
  Serial1.begin(460800);
  Serial1.addMemoryForRead(rx_buffer,API_RX_SIZE-63);
}

void loop()
{
  
  uint8_t tx_buf[API_TX_SIZE] = {0};
  format_packet(fpos, tx_buf);
  Serial1.write(tx_buf, API_TX_SIZE);
  Serial1.clear();
  int time = 1000;
  delayMicroseconds(time);
  Serial1.flush();
  delayMicroseconds(time);
  read_values();
  nh.spinOnce(); 
}