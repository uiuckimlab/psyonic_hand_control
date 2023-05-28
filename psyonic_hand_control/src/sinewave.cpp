#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <sstream>
#include <ctime>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "sinewave");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher wave_pub = n.advertise<std_msgs::Float32MultiArray>("psyonic_controller", 1000);

  ros::Rate loop_rate(125);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  
  std_msgs::Float32MultiArray msg;

  std::vector<float> sine_wave = {15.f,15.f,15.f,15.f,15.f,-15.f};
  double current_time = 0.0000000;
  double add_time = 0.008;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    float t = current_time * 5;
    for(int i = 0; i < sine_wave.size(); i++){
        sine_wave[i] = 57.0f*cos(t + (float)i) + 57.0f;  
    }

    // sine_wave[5] = -sine_wave[5]; 
    sine_wave[4] = 0;
    sine_wave[5] = 0;
    // ROS_INFO_STREAM("HERE");
    // ROS_INFO_STREAM("TIME: " << t );
    // ROS_INFO_STREAM("VALUES: " << sine_wave[0]);
    msg.data = sine_wave;

    wave_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    current_time = current_time + add_time;
  }


  return 0;
}