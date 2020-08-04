#ifndef _ROS_mrpt_msgs_NetworkOfPoses_h
#define _ROS_mrpt_msgs_NetworkOfPoses_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mrpt_msgs/NodeIDWithPose_vec.h"
#include "mrpt_msgs/GraphConstraint.h"

namespace mrpt_msgs
{

  class NetworkOfPoses : public ros::Msg
  {
    public:
      typedef uint64_t _root_type;
      _root_type root;
      typedef mrpt_msgs::NodeIDWithPose_vec _nodes_type;
      _nodes_type nodes;
      uint32_t constraints_length;
      typedef mrpt_msgs::GraphConstraint _constraints_type;
      _constraints_type st_constraints;
      _constraints_type * constraints;

    NetworkOfPoses():
      root(0),
      nodes(),
      constraints_length(0), constraints(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->root >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->root >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->root >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->root >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->root >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->root >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->root >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->root >> (8 * 7)) & 0xFF;
      offset += sizeof(this->root);
      offset += this->nodes.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->constraints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->constraints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->constraints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->constraints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->constraints_length);
      for( uint32_t i = 0; i < constraints_length; i++){
      offset += this->constraints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->root =  ((uint64_t) (*(inbuffer + offset)));
      this->root |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->root |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->root |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->root |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->root |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->root |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->root |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->root);
      offset += this->nodes.deserialize(inbuffer + offset);
      uint32_t constraints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      constraints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      constraints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      constraints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->constraints_length);
      if(constraints_lengthT > constraints_length)
        this->constraints = (mrpt_msgs::GraphConstraint*)realloc(this->constraints, constraints_lengthT * sizeof(mrpt_msgs::GraphConstraint));
      constraints_length = constraints_lengthT;
      for( uint32_t i = 0; i < constraints_length; i++){
      offset += this->st_constraints.deserialize(inbuffer + offset);
        memcpy( &(this->constraints[i]), &(this->st_constraints), sizeof(mrpt_msgs::GraphConstraint));
      }
     return offset;
    }

    const char * getType(){ return "mrpt_msgs/NetworkOfPoses"; };
    const char * getMD5(){ return "2f7e17a78e81d376b3e999d635df4228"; };

  };

}
#endif
