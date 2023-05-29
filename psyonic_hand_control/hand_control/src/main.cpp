/* Authors: Jooyoung Hong, Sankalp Yamsani, Chaerim Moon, Kazuki Shin 
   Documentation and Example code for Psyonic Here: https://github.com/psyonicinc/ability-hand-api
*/
#include <Arduino.h>
#include <Wire.h>

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


/*Moves Fingers to the position user has set*/
void moveHandCallback(const std_msgs::Float32MultiArray& msg){
  for (int i = 0; i < msg.data_length && i < NUM_CHANNELS; i++) {
    fpos[i] = msg.data[i];
  }
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
  sum = get_checksum(data, 72);
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



void setup()
{
  nh.getHardware()->setBaud(4000000);
  nh.initNode();   
  nh.advertise(pub);
  nh.subscribe(openHandSub);
  Serial1.begin(460800);
}

void loop()
{
  
  uint8_t tx_buf[API_TX_SIZE] = {0};
  format_packet(fpos, tx_buf);
  Serial1.write(tx_buf, 15);
  read_values_1();
  nh.spinOnce(); 
}