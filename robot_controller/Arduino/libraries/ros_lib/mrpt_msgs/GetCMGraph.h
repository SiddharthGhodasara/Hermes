#ifndef _ROS_SERVICE_GetCMGraph_h
#define _ROS_SERVICE_GetCMGraph_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mrpt_msgs/NetworkOfPoses.h"

namespace mrpt_msgs
{

static const char GETCMGRAPH[] = "mrpt_msgs/GetCMGraph";

  class GetCMGraphRequest : public ros::Msg
  {
    public:
      uint32_t nodeIDs_length;
      typedef uint64_t _nodeIDs_type;
      _nodeIDs_type st_nodeIDs;
      _nodeIDs_type * nodeIDs;

    GetCMGraphRequest():
      nodeIDs_length(0), nodeIDs(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nodeIDs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodeIDs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodeIDs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodeIDs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodeIDs_length);
      for( uint32_t i = 0; i < nodeIDs_length; i++){
      *(outbuffer + offset + 0) = (this->nodeIDs[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodeIDs[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodeIDs[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodeIDs[i] >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->nodeIDs[i] >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->nodeIDs[i] >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->nodeIDs[i] >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->nodeIDs[i] >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nodeIDs[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t nodeIDs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodeIDs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodeIDs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodeIDs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodeIDs_length);
      if(nodeIDs_lengthT > nodeIDs_length)
        this->nodeIDs = (uint64_t*)realloc(this->nodeIDs, nodeIDs_lengthT * sizeof(uint64_t));
      nodeIDs_length = nodeIDs_lengthT;
      for( uint32_t i = 0; i < nodeIDs_length; i++){
      this->st_nodeIDs =  ((uint64_t) (*(inbuffer + offset)));
      this->st_nodeIDs |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_nodeIDs |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_nodeIDs |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_nodeIDs |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->st_nodeIDs |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->st_nodeIDs |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->st_nodeIDs |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->st_nodeIDs);
        memcpy( &(this->nodeIDs[i]), &(this->st_nodeIDs), sizeof(uint64_t));
      }
     return offset;
    }

    const char * getType(){ return GETCMGRAPH; };
    const char * getMD5(){ return "5f27c6ad84ac86a19b3f9376555dd464"; };

  };

  class GetCMGraphResponse : public ros::Msg
  {
    public:
      typedef mrpt_msgs::NetworkOfPoses _cm_graph_type;
      _cm_graph_type cm_graph;

    GetCMGraphResponse():
      cm_graph()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->cm_graph.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->cm_graph.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETCMGRAPH; };
    const char * getMD5(){ return "bbabb31f01a04e85d128509e707fbf64"; };

  };

  class GetCMGraph {
    public:
    typedef GetCMGraphRequest Request;
    typedef GetCMGraphResponse Response;
  };

}
#endif
