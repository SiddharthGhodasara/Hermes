#ifndef _ROS_mrpt_msgs_NodeIDWithPose_h
#define _ROS_mrpt_msgs_NodeIDWithPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"

namespace mrpt_msgs
{

  class NodeIDWithPose : public ros::Msg
  {
    public:
      typedef uint64_t _nodeID_type;
      _nodeID_type nodeID;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef std_msgs::String _str_ID_type;
      _str_ID_type str_ID;
      typedef uint64_t _nodeID_loc_type;
      _nodeID_loc_type nodeID_loc;

    NodeIDWithPose():
      nodeID(0),
      pose(),
      str_ID(),
      nodeID_loc(0)
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
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->str_ID.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->nodeID_loc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodeID_loc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodeID_loc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodeID_loc >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->nodeID_loc >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->nodeID_loc >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->nodeID_loc >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->nodeID_loc >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nodeID_loc);
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
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->str_ID.deserialize(inbuffer + offset);
      this->nodeID_loc =  ((uint64_t) (*(inbuffer + offset)));
      this->nodeID_loc |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->nodeID_loc |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->nodeID_loc |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nodeID_loc |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->nodeID_loc |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->nodeID_loc |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->nodeID_loc |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->nodeID_loc);
     return offset;
    }

    const char * getType(){ return "mrpt_msgs/NodeIDWithPose"; };
    const char * getMD5(){ return "87c8db37a689c10a5c5dc1aa39838320"; };

  };

}
#endif
