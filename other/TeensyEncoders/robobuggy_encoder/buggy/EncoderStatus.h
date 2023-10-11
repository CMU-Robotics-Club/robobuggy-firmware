#ifndef _ROS_buggy_EncoderStatus_h
#define _ROS_buggy_EncoderStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace buggy
{

  class EncoderStatus : public ros::Msg
  {
    public:
      typedef double _left_wheel_encoder_type;
      _left_wheel_encoder_type left_wheel_encoder;
      typedef double _right_wheel_encoder_type;
      _right_wheel_encoder_type right_wheel_encoder;
      typedef double _front_wheel_encoder_type;
      _front_wheel_encoder_type front_wheel_encoder;

    EncoderStatus():
      left_wheel_encoder(0),
      right_wheel_encoder(0),
      front_wheel_encoder(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_left_wheel_encoder;
      u_left_wheel_encoder.real = this->left_wheel_encoder;
      *(outbuffer + offset + 0) = (u_left_wheel_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_wheel_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_wheel_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_wheel_encoder.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_wheel_encoder.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_wheel_encoder.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_wheel_encoder.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_wheel_encoder.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_wheel_encoder);
      union {
        double real;
        uint64_t base;
      } u_right_wheel_encoder;
      u_right_wheel_encoder.real = this->right_wheel_encoder;
      *(outbuffer + offset + 0) = (u_right_wheel_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wheel_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wheel_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wheel_encoder.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_wheel_encoder.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_wheel_encoder.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_wheel_encoder.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_wheel_encoder.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_wheel_encoder);
      union {
        double real;
        uint64_t base;
      } u_front_wheel_encoder;
      u_front_wheel_encoder.real = this->front_wheel_encoder;
      *(outbuffer + offset + 0) = (u_front_wheel_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_wheel_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_wheel_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_wheel_encoder.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_front_wheel_encoder.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_front_wheel_encoder.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_front_wheel_encoder.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_front_wheel_encoder.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->front_wheel_encoder);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_left_wheel_encoder;
      u_left_wheel_encoder.base = 0;
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_wheel_encoder = u_left_wheel_encoder.real;
      offset += sizeof(this->left_wheel_encoder);
      union {
        double real;
        uint64_t base;
      } u_right_wheel_encoder;
      u_right_wheel_encoder.base = 0;
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_wheel_encoder = u_right_wheel_encoder.real;
      offset += sizeof(this->right_wheel_encoder);
      union {
        double real;
        uint64_t base;
      } u_front_wheel_encoder;
      u_front_wheel_encoder.base = 0;
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_front_wheel_encoder.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->front_wheel_encoder = u_front_wheel_encoder.real;
      offset += sizeof(this->front_wheel_encoder);
     return offset;
    }

    virtual const char * getType() override { return "buggy/EncoderStatus"; };
    virtual const char * getMD5() override { return "334b7df70f4b1a48a8e442db0dd4459d"; };

  };

}
#endif
