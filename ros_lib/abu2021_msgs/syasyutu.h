#ifndef _ROS_abu2021_msgs_syasyutu_h
#define _ROS_abu2021_msgs_syasyutu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2021_msgs
{

  class syasyutu : public ros::Msg
  {
    public:
      typedef int64_t _num_type;
      _num_type num;
      typedef int64_t _hoge_type;
      _hoge_type hoge;

    syasyutu():
      num(0),
      hoge(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_num;
      u_num.real = this->num;
      *(outbuffer + offset + 0) = (u_num.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_num.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_num.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_num.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_num.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->num);
      union {
        int64_t real;
        uint64_t base;
      } u_hoge;
      u_hoge.real = this->hoge;
      *(outbuffer + offset + 0) = (u_hoge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hoge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hoge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hoge.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_hoge.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_hoge.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_hoge.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_hoge.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->hoge);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_num;
      u_num.base = 0;
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_num.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->num = u_num.real;
      offset += sizeof(this->num);
      union {
        int64_t real;
        uint64_t base;
      } u_hoge;
      u_hoge.base = 0;
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_hoge.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->hoge = u_hoge.real;
      offset += sizeof(this->hoge);
     return offset;
    }

    const char * getType(){ return "abu2021_msgs/syasyutu"; };
    const char * getMD5(){ return "d505eb1fd304128c1e1fcdac2af5f197"; };

  };

}
#endif
