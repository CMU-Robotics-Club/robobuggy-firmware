#ifndef _ROS_buggy_LoRaEvent_h
#define _ROS_buggy_LoRaEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace buggy
{

  class LoRaEvent : public ros::Msg
  {
    public:
      typedef float _rssi_type;
      _rssi_type rssi;
      typedef float _snr_type;
      _snr_type snr;
      typedef uint16_t _hops_type;
      _hops_type hops;
      typedef int16_t _code_type;
      _code_type code;
      typedef bool _crc_type;
      _crc_type crc;

    LoRaEvent():
      rssi(0),
      snr(0),
      hops(0),
      code(0),
      crc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_rssi;
      u_rssi.real = this->rssi;
      *(outbuffer + offset + 0) = (u_rssi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rssi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rssi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rssi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rssi);
      union {
        float real;
        uint32_t base;
      } u_snr;
      u_snr.real = this->snr;
      *(outbuffer + offset + 0) = (u_snr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_snr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_snr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_snr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->snr);
      *(outbuffer + offset + 0) = (this->hops >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hops >> (8 * 1)) & 0xFF;
      offset += sizeof(this->hops);
      union {
        int16_t real;
        uint16_t base;
      } u_code;
      u_code.real = this->code;
      *(outbuffer + offset + 0) = (u_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_code.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->code);
      union {
        bool real;
        uint8_t base;
      } u_crc;
      u_crc.real = this->crc;
      *(outbuffer + offset + 0) = (u_crc.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->crc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_rssi;
      u_rssi.base = 0;
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rssi = u_rssi.real;
      offset += sizeof(this->rssi);
      union {
        float real;
        uint32_t base;
      } u_snr;
      u_snr.base = 0;
      u_snr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_snr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_snr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_snr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->snr = u_snr.real;
      offset += sizeof(this->snr);
      this->hops =  ((uint16_t) (*(inbuffer + offset)));
      this->hops |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->hops);
      union {
        int16_t real;
        uint16_t base;
      } u_code;
      u_code.base = 0;
      u_code.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_code.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->code = u_code.real;
      offset += sizeof(this->code);
      union {
        bool real;
        uint8_t base;
      } u_crc;
      u_crc.base = 0;
      u_crc.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->crc = u_crc.real;
      offset += sizeof(this->crc);
     return offset;
    }

    virtual const char * getType() override { return "buggy/LoRaEvent"; };
    virtual const char * getMD5() override { return "deac2c5fd57c8fe6d4563cbf03b07af5"; };

  };

}
#endif
