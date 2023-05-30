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

/*Moves Fingers to the position user has set*/
void moveHandCallback(const std_msgs::Float32MultiArray& msg){
  for (uint32_t i = 0; i < msg.data_length && i < NUM_CHANNELS; i++) {
    fpos[i] = msg.data[i];
  }
}

void setup()
{
  nh.getHardware()->setBaud(4000000);
  nh.initNode();   
  nh.advertise(pub);
  nh.subscribe(openHandSub);
  Serial1.begin(460800);
  Serial1.addMemoryForRead(rx_buffer,API_RX_SIZE-63); // 63 is default buffer size, so trying to make it 72 bytes
  Serial1.clear();
}

void loop()
{
  
  uint8_t tx_buf[API_TX_SIZE] = {0};
  format_packet(fpos, tx_buf);
  Serial1.write(tx_buf, API_TX_SIZE);
  int time = 1000;
  delayMicroseconds(time); // needed for correct read data
  Serial1.flush();
  delayMicroseconds(time); // needed for correct read data
  read_values(hand_msg,Serial1);
  pub.publish(&hand_msg);
  nh.spinOnce(); 
}