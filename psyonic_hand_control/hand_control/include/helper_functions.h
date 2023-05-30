/* Authors: Jooyoung Hong, Sankalp Yamsani, Chaerim Moon, Kazuki Shin 
   Documentation and Example code for Psyonic Here: https://github.com/psyonicinc/ability-hand-api
*/
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <psyonic_hand_control/handVal.h>
#include <std_msgs/Float32MultiArray.h>


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

// Converts data from little endian to big endian and into current format
float current_converter(uint8_t data, uint8_t data2);

// Gets the force value from the finger tip version 1
float tipforce_converter_1(uint8_t data, uint8_t data2, uint8_t data3);

// Gets the force value from the finger tip version 2
float tipforce_converter_2(uint8_t data, uint8_t data2, uint8_t data3);

void read_values(psyonic_hand_control::handVal &hand_msg, HardwareSerial &Serial1);