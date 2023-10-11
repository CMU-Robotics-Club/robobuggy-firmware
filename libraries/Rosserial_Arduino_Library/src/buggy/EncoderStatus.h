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
      typedef float _left_wheel_encoder_type;
      _left_wheel_encoder_type left_wheel_encoder;
      typedef float _right_wheel_encoder_type;
      _right_wheel_encoder_type right_wheel_encoder;
      typedef float _front_wheel_encoder_type;
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
      offset += serializeAvrFloat64(outbuffer + offset, this->left_wheel_encoder);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_wheel_encoder);
      offset += serializeAvrFloat64(outbuffer + offset, this->front_wheel_encoder);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_wheel_encoder));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_wheel_encoder));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->front_wheel_encoder));
     return offset;
    }

    virtual const char * getType() override { return "buggy/EncoderStatus"; };
    virtual const char * getMD5() override { return "334b7df70f4b1a48a8e442db0dd4459d"; };

  };

}
#endif
