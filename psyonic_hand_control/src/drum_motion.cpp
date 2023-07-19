/* Authors: Jooyoung Hong, Sankalp Yamsani, Chaerim Moon, Kazuki Shin */
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <sstream>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "drum_motion");
  ros::NodeHandle n;

  ros::Publisher wave_pub = n.advertise<std_msgs::Float32MultiArray>("psyonic_controller", 1000);
  float looprate = 10;
  bool sine_wave_flag = true;
  // 10 20 50 100 200
  ros::Rate loop_rate((int)looprate);

  std_msgs::Float32MultiArray msg;

  std::vector<float> finger_pose = {15.f,15.f,15.f,15.f,15.f,-15.f};

  std::vector<float> open_pose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  std::vector<float> close_pose = {120.f, 120.f, 120.f, 120.f, 120.f, 120.f};


  double current_time = 0.0000000;
  double add_time = 1./looprate;

  double amplitude = 60.0f;
  double multiply = 5;

  std::ofstream MyFile("/home/kimlab/ws_music/src/psyonic_hand_control/psyonic_hand_control/src/amp_" + std::to_string(amplitude) + "_" + std::to_string(looprate)+"_" + std::to_string(5) + "_hz_06_20.txt");

  while (ros::ok())
  {
    
    float t = current_time * multiply;

    if(sine_wave_flag){
      // for(int i = 0; i < finger_pose.size(); i++){
          finger_pose[0] = amplitude*cos(t) + amplitude;  
    }else{
      for(int i = 0; i < finger_pose.size(); i++){
        if( (amplitude*cos(t) + amplitude ) < amplitude){
          finger_pose[i] = open_pose[i];
        }else{
          finger_pose[i] = close_pose[i];
        }
      }
    }

    // finger_pose[0] = 0;
    MyFile << finger_pose[0] << "\n";
    finger_pose[1] = 0;
    finger_pose[2] = 0;
    finger_pose[3] = 0;
    finger_pose[4] = 0;
    finger_pose[5] = 0;
  
    finger_pose[5] = -finger_pose[5];
    msg.data = finger_pose;

    wave_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    // if(sine_wave_flag == false ){
    ros::Duration(2).sleep();
    // }
    current_time = current_time + add_time;
  }
  MyFile.close();

  return 0;
}