#ifndef _ROS_abu2021_msgs_cmd_vw_h
#define _ROS_abu2021_msgs_cmd_vw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2021_msgs
{

  class cmd_vw : public ros::Msg
  {
    public:
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _w_type;
      _w_type w;

    cmd_vw():
      vx(0),
      vy(0),
      w(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->vy);
      offset += serializeAvrFloat64(outbuffer + offset, this->w);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->w));
     return offset;
    }

    const char * getType(){ return "abu2021_msgs/cmd_vw"; };
    const char * getMD5(){ return "461e74593b26c77a7c97f00fe82c6d79"; };

  };

}
#endif
