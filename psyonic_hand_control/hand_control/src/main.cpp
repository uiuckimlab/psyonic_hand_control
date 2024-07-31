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
void enableThumbCallback(const std_msgs::Bool & msg);

// Setup ROS Related variables
ros::NodeHandle nh;
psyonic_hand_control::handVal hand_msg;
ros::Publisher pub("robot1/psyonic_hand_vals", &hand_msg);
ros::Subscriber<std_msgs::Float32MultiArray> openHandSub("robot1/psyonic_controller/command", moveHandCallback);
ros::Subscriber<std_msgs::Bool> enableThumbUpsamleSub("robot1/psyonic_controller/enable_upsample", enableThumbCallback);

// Setup finger position variable
float fpos[NUM_CHANNELS] = {15.f,15.f,15.f,15.f,15.f, -15.f};
uint8_t rx_buffer[API_RX_SIZE];

/*Moves Fingers to the position user has set*/
void moveHandCallback(const std_msgs::Float32MultiArray& msg){
  for (uint32_t i = 0; i < msg.data_length && i < NUM_CHANNELS; i++) {
    fpos[i] = msg.data[i] * 180.0 / M_PI;
  }
}

void enableThumbCallback(const std_msgs::Bool& msg){
  int time = 1000;
  delayMicroseconds(time*5);
  uint8_t tx_buf_enable[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable, msg.data);
  Serial1.write(tx_buf_enable, API_TX_SIZE);
  delayMicroseconds(time);

}

void setup()
{
  nh.getHardware()->setBaud(4000000);
  nh.initNode();   
  nh.advertise(pub);
  nh.subscribe(openHandSub);
  nh.subscribe(enableThumbUpsamleSub);
  Serial1.begin(460800);
  Serial1.addMemoryForRead(rx_buffer,API_RX_SIZE-63); // 63 is default buffer size, so trying to make it 72 bytes
  Serial1.clear();
  int time = 1000;
  delayMicroseconds(time*5);
  uint8_t tx_buf_enable[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable, true);
  Serial1.write(tx_buf_enable, API_TX_SIZE);
  delayMicroseconds(time);


}

void loop()
{
  float start_time = millis();
  uint8_t tx_buf[API_TX_SIZE] = {0};
  format_packet(fpos, tx_buf);
  Serial1.write(tx_buf, API_TX_SIZE);
  int time = 1000;
  delayMicroseconds(time); // needed for correct read data
  Serial1.flush();
  delayMicroseconds(time); // needed for correct read data
  read_values(hand_msg,Serial1);
  hand_msg.fingertips[35] = millis() - start_time;
  pub.publish(&hand_msg);

  nh.spinOnce(); 
}