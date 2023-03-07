#ifndef _ROS_psyonic_hand_control_handVal_h
#define _ROS_psyonic_hand_control_handVal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace psyonic_hand_control
{

  class handVal : public ros::Msg
  {
    public:
      float positions[6];
      float currents[6];
      float velocities[6];
      float fingertips[36];

    handVal():
      positions(),
      currents(),
      velocities(),
      fingertips()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_positionsi;
      u_positionsi.real = this->positions[i];
      *(outbuffer + offset + 0) = (u_positionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positionsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positions[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_currentsi;
      u_currentsi.real = this->currents[i];
      *(outbuffer + offset + 0) = (u_currentsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currents[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_velocitiesi;
      u_velocitiesi.real = this->velocities[i];
      *(outbuffer + offset + 0) = (u_velocitiesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocitiesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocitiesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocitiesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities[i]);
      }
      for( uint32_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_fingertipsi;
      u_fingertipsi.real = this->fingertips[i];
      *(outbuffer + offset + 0) = (u_fingertipsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fingertipsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fingertipsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fingertipsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fingertips[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_positionsi;
      u_positionsi.base = 0;
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->positions[i] = u_positionsi.real;
      offset += sizeof(this->positions[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_currentsi;
      u_currentsi.base = 0;
      u_currentsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currents[i] = u_currentsi.real;
      offset += sizeof(this->currents[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_velocitiesi;
      u_velocitiesi.base = 0;
      u_velocitiesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocitiesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocitiesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocitiesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocities[i] = u_velocitiesi.real;
      offset += sizeof(this->velocities[i]);
      }
      for( uint32_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_fingertipsi;
      u_fingertipsi.base = 0;
      u_fingertipsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fingertipsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fingertipsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fingertipsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fingertips[i] = u_fingertipsi.real;
      offset += sizeof(this->fingertips[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "psyonic_hand_control/handVal"; };
    virtual const char * getMD5() override { return "2c5ecc99fbd76ae2d2463465cb2deaf6"; };

  };

}
#endif
