/* Authors: Jooyoung Hong, Sankalp Yamsani, Chaerim Moon, Kazuki Shin */
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <sstream>
#include <ctime>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "sinewave");
  ros::NodeHandle n;

  ros::Publisher wave_pub = n.advertise<std_msgs::Float32MultiArray>("psyonic_controller", 1000);
  float looprate = 200;
  bool sine_wave_flag = true;
  // 10 20 50 100 200
  ros::Rate loop_rate((int)looprate);

  std_msgs::Float32MultiArray msg;

  std::vector<float> finger_pose = {15.f,15.f,15.f,15.f,15.f,-15.f};

  std::vector<float> open_pose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  std::vector<float> close_pose = {114.f, 114.f, 114.f, 114.f, 114.f, 114.f};


  double current_time = 0.0000000;
  double add_time = 1./looprate;
  while (ros::ok())
  {
    
    float t = current_time * 5;

    if(sine_wave_flag){
      for(int i = 0; i < finger_pose.size(); i++){
          finger_pose[i] = 57.0f*cos(t + (float)i) + 57.0f;  
      }
    }else{
      for(int i = 0; i < finger_pose.size(); i++){
        if( (57.0f*cos(t + (float)i) + 57.0f) < 57.0f){
          finger_pose[i] = open_pose[i];
        }else{
          finger_pose[i] = close_pose[i];
        }
      }
    }


    finger_pose[4] = 0;
    finger_pose[5] = 0;
  
    msg.data = finger_pose;

    wave_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    current_time = current_time + add_time;
  }


  return 0;
}