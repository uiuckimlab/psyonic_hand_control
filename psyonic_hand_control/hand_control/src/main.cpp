// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include <Arduino.h>
#include <Wire.h>

int led = LED_BUILTIN;

#include <stdint.h>
#include <math.h>

#define NUM_CHANNELS 6
#define API_TX_SIZE	 15

typedef union api_i16_t
{
	int16_t i16[NUM_CHANNELS];
	uint8_t u8[NUM_CHANNELS*sizeof(int16_t)];
}api_i16_t;


/*Helper function to get the signed 8bit checksum*/
uint8_t get_checksum(uint8_t * arr, int size)
{
	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
}

/*Takes 6x floating point inputs for hand position arguments in DEGREES, and creates an
API frame to send out*/
void format_packet(float fpos_in[NUM_CHANNELS], uint8_t tx_buf[API_TX_SIZE])
{
	tx_buf[0] = 0x50; //hand slave address (use default)	
	tx_buf[1] = 0x12;
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


void setup()
{
  pinMode(led, OUTPUT);
  // Wire1.begin();             // join i2c bus (address optional for master)
  Serial4.begin(460800);
  Serial.begin(9600);       // start serial for output
  Serial.println("Begin");
}

void read_values()
{
  int len_reception = 39; // 39:EXtended variant3 //72:EXtended variant1,2 //10: standard I2C
  int c[API_TX_SIZE];
  Serial.println("Reading");
  for (int i=0;i<len_reception;i++)
  {
    if (Serial4.available()){
      c[i] = Serial4.read(); 
      Serial.print(" ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(c[i]);
    }
  }
  Serial.println("");
}

void loop()
{
  
  float fpos[NUM_CHANNELS] = {15.f,15.f,15.f,15.f,15.f,-15.f};
	uint8_t tx_buf[API_TX_SIZE] = {0};
  
  float t_start = ((float)millis())*.001f;

  char input;
  while(1)
  {
        
    // *****************
    // sinusoidal movement example
    // but jerky though

    float t = ((float)millis())*.001f;
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			fpos[ch] = (0.5f*cos((t) + (float)ch)+0.5f)*30.f + 15.f;
		}
		fpos[5] = -fpos[5];

		format_packet(fpos, tx_buf);
    Serial4.write(tx_buf, 15);
    Serial.println("Read");
    read_values();
    delay(1);
   }
  
  }