#ifndef _ROS_mrpt_msgs_GraphConstraint_h
#define _ROS_mrpt_msgs_GraphConstraint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseWithCovariance.h"

namespace mrpt_msgs
{

  class GraphConstraint : public ros::Msg
  {
    public:
      typedef uint64_t _nodeID_from_type;
      _nodeID_from_type nodeID_from;
      typedef uint64_t _nodeID_to_type;
      _nodeID_to_type nodeID_to;
      typedef geometry_msgs::PoseWithCovariance _constraint_type;
      _constraint_type constraint;

    GraphConstraint():
      nodeID_from(0),
      nodeID_to(0),
      constraint()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nodeID_from >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodeID_from >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodeID_from >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodeID_from >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->nodeID_from >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->nodeID_from >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->nodeID_from >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->nodeID_from >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nodeID_from);
      *(outbuffer + offset + 0) = (this->nodeID_to >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodeID_to >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodeID_to >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodeID_to >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->nodeID_to >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->nodeID_to >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->nodeID_to >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->nodeID_to >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nodeID_to);
      offset += this->constraint.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->nodeID_from =  ((uint64_t) (*(inbuffer + offset)));
      this->nodeID_from |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->nodeID_from |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->nodeID_from |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nodeID_from |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->nodeID_from |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->nodeID_from |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->nodeID_from |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->nodeID_from);
      this->nodeID_to =  ((uint64_t) (*(inbuffer + offset)));
      this->nodeID_to |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->nodeID_to |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->nodeID_to |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nodeID_to |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->nodeID_to |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->nodeID_to |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->nodeID_to |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->nodeID_to);
      offset += this->constraint.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mrpt_msgs/GraphConstraint"; };
    const char * getMD5(){ return "27a8223828dcd501cdc97873eecdd09e"; };

  };

}
#endif
