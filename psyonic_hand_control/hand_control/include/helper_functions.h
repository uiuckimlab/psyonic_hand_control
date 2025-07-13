/* Authors: Jooyoung Hong, Sankalp Yamsani, Chaerim Moon, Kazuki Shin 
   Documentation and Example code for Psyonic Here: https://github.com/psyonicinc/ability-hand-api
*/
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

#include <psyonic_hand_control/msg/hand_val.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>


#define NUM_CHANNELS 6
#define API_TX_SIZE	 15
#define API_RX_SIZE 72

/*Helper function to get the signed 8bit checksum*/
uint8_t get_checksum(uint8_t * arr, int size);

/*Takes 6x floating point inputs for hand position arguments in DEGREES, and creates an
API frame to send out*/
void format_packet(float fpos_in[NUM_CHANNELS], uint8_t tx_buf[API_TX_SIZE]);

// Converts data from little endian to big endian and into degree format
float position_converter(uint8_t data, uint8_t data2);

// // Converts data from little endian to big endian and into current format
// float current_converter(uint8_t data, uint8_t data2);

// enables upsampling on the thumb
void enable_thumb_upsample( uint8_t tx_buf[API_TX_SIZE], bool enable = true);


// Converts data from little endian to big endian and into current format
float velocity_converter(uint8_t data, uint8_t data2);

// Gets the force value from the finger tip version 1
float tipforce_converter_1(uint8_t data, uint8_t data2, uint8_t data3);

// Gets the force value from the finger tip version 2
float tipforce_converter_2(uint8_t data, uint8_t data2, uint8_t data3);

void read_values(psyonic_hand_control__msg__HandVal &hand_msg, HardwareSerial &Serial1);