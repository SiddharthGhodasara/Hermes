#ifndef _ROS_mrpt_msgs_NodeIDWithLaserScan_h
#define _ROS_mrpt_msgs_NodeIDWithLaserScan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/LaserScan.h"

namespace mrpt_msgs
{

  class NodeIDWithLaserScan : public ros::Msg
  {
    public:
      typedef uint64_t _nodeID_type;
      _nodeID_type nodeID;
      typedef sensor_msgs::LaserScan _scan_type;
      _scan_type scan;

    NodeIDWithLaserScan():
      nodeID(0),
      scan()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nodeID >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodeID >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodeID >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodeID >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->nodeID >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->nodeID >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->nodeID >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->nodeID >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nodeID);
      offset += this->scan.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->nodeID =  ((uint64_t) (*(inbuffer + offset)));
      this->nodeID |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->nodeID |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->nodeID |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nodeID |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->nodeID |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->nodeID |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->nodeID |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->nodeID);
      offset += this->scan.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mrpt_msgs/NodeIDWithLaserScan"; };
    const char * getMD5(){ return "135d436b647c8470f71b2c97722a4352"; };

  };

}
#endif
