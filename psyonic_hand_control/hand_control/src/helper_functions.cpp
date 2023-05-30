/* Authors: Jooyoung Hong, Sankalp Yamsani, Chaerim Moon, Kazuki Shin 
   Documentation and Example code for Psyonic Here: https://github.com/psyonicinc/ability-hand-api
*/

#include "helper_functions.h"

/*Struct to help with sending bytes little endian*/
typedef union api_i16_t{
	int16_t i16[NUM_CHANNELS];
	uint8_t u8[NUM_CHANNELS*sizeof(int16_t)];
}api_i16_t;


/*Helper function to get the signed 8bit checksum*/
uint8_t get_checksum(uint8_t * arr, int size){
	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
}

/*Takes 6x floating point inputs for hand position arguments in DEGREES, and creates an
API frame to send out*/
void format_packet(float fpos_in[NUM_CHANNELS], uint8_t tx_buf[API_TX_SIZE]){
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

// Converts data from little endian to big endian and into degree format
float position_converter(uint8_t data, uint8_t data2){
  int16_t pos = (data2);
  pos = pos << 8;
  pos = pos | data;
  
  float theta = ((float(pos) / 32767.0f) * 150.0f);
  return theta;
}

// Converts data from little endian to big endian and into correct current format
float current_converter(uint8_t data, uint8_t data2){
  int16_t amp = (data2);
  amp = amp << 8;
  amp = amp | data;
  float current = ((float(amp) / 620.606079)); // BLE command Rv
  return current;
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

void read_values(psyonic_hand_control::handVal &hand_msg, HardwareSerial &Serial1){

  int8_t sum = 0;
  uint8_t data[API_RX_SIZE];
  int len1 = Serial1.available();

  if(len1 != API_RX_SIZE){
    return;
  }

  for (int i = 0; i < API_RX_SIZE; i++){
    data[i] = Serial1.read();
  }
  
  sum = get_checksum(data, API_RX_SIZE-1);
  sum -= (int8_t)data[API_RX_SIZE-1];

  if(sum != 0){
    return;
  }

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
  hand_msg.fingertips[33] = sum;
  hand_msg.fingertips[32] = data[70];
  hand_msg.fingertips[31] = data[69];

}


