#define MCAP_IMPLEMENTATION
#include "mcap/writer.hpp"
#include "protobuf/BuildFileDescriptorSet.h"
#include <boxes_msg.pb.h>
#include <iostream>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gtest/gtest.h>


template <typename T>
class Log
{
    public: explicit Log(const std::string &_filePath)
    {
     auto options = mcap::McapWriterOptions("");
     const auto s = writer.open(_filePath.c_str(), options);
     if(!s.ok())
     {
        std::cout << "file not created" << _filePath << std::endl;
     }
    }

    public: void setBoxMsg(const std::string &_physicsEngine, 
               double &_dt, bool &_complex, bool &_collision,
               int &_modelCount, bool &_logMultiple)
    {
      mcap::Schema schema("benchmark_proto.BoxesMsg", "protobuf",
                         foxglove::BuildFileDescriptorSet(benchmark_proto::BoxesMsg::descriptor()).SerializeAsString());
    
      writer.addSchema(schema);
      mcap::Channel channel("model_states", "protobuf", schema.id);
      writer.addChannel(channel);
      channelId = channel.id;
      msg.set_physics_engine(_physicsEngine);
      msg.set_dt(_dt);
      msg.set_complex(_complex);
      msg.set_collision(_collision);
      msg.set_model_count(_modelCount);
      msg.set_log_multiple(_logMultiple);
    
      if(_logMultiple)
      {
        for(int model_no = 1; model_no<=_modelCount; model_no++)
        {
          msg.add_data()->set_model_no(model_no);
        }
      }
    
      else
        msg.add_data()->set_model_no(1);
    }

    public: void stop()
    {
      std::string serialized = msg.SerializeAsString();
      mcap::Message mcapMsg;
      mcapMsg.channelId = channelId;
      mcapMsg.data = reinterpret_cast<const std::byte*>(serialized.data());
      mcapMsg.dataSize = serialized.size();
      const auto res = writer.write(mcapMsg);
    }

    public: void recordSimTime(double &_t)
    {
      msg.add_sim_time(_t);
    }

    public: void recordComputationTime(double &_computationTime)
    {
      msg.set_computation_time(_computationTime);
    }

    public:  void recordPose(int &_modelIdx, const std::vector<double> _position, 
                            const std::vector<double> _quaternion)
    {
      auto pose = msg.mutable_data(_modelIdx)->add_poses();   
  
      pose->mutable_position()->set_x(_position[0]);
      pose->mutable_position()->set_y(_position[1]);
      pose->mutable_position()->set_z(_position[3]);
  
      pose->mutable_orientation()->set_w(_quaternion[0]);
      pose->mutable_orientation()->set_x(_quaternion[1]);
      pose->mutable_orientation()->set_y(_quaternion[2]);
      pose->mutable_orientation()->set_z(_quaternion[3]);                           
    }

    public: void recordPose(int &_modelIdx, const ignition::math::Pose3d &_pose)
    {  
      ignition::math::Vector3d p = _pose.Pos();
      ignition::math::Quaterniond r = _pose.Rot(); 
    
      auto pose = msg.mutable_data(_modelIdx)->add_poses();   
       
      pose->mutable_position()->set_x(p.X());
      pose->mutable_position()->set_y(p.Y());
      pose->mutable_position()->set_z(p.Z());
    
      pose->mutable_orientation()->set_w(r.W());
      pose->mutable_orientation()->set_x(r.X());
      pose->mutable_orientation()->set_y(r.Y());
      pose->mutable_orientation()->set_z(r.Z());
    }

    public: void recordTwist(int &_modelIdx, const std::vector<double> &_linVelocity,
                             const std::vector<double> &_angVelocity)
    {
      auto twist = msg.mutable_data(_modelIdx)->add_twists();
    
      twist->mutable_linear()->set_x(_linVelocity[0]);
      twist->mutable_linear()->set_y(_linVelocity[1]);
      twist->mutable_linear()->set_z(_linVelocity[2]);
    
      twist->mutable_angular()->set_x(_angVelocity[0]);
      twist->mutable_angular()->set_y(_angVelocity[1]);
      twist->mutable_angular()->set_z(_angVelocity[2]);
    }

    public: void recordTwist(int &_modelIdx, const ignition::math::Vector3d &_linVelocity,
                             const ignition::math::Vector3d &_angVelocity)
    {
      auto twist = msg.mutable_data(_modelIdx)->add_twists();
    
      twist->mutable_linear()->set_x(_linVelocity.X());
      twist->mutable_linear()->set_y(_linVelocity.Y());
      twist->mutable_linear()->set_z(_linVelocity.Z());
    
      twist->mutable_angular()->set_x(_angVelocity.X());
      twist->mutable_angular()->set_y(_angVelocity.Y());
      twist->mutable_angular()->set_z(_angVelocity.Z());           
    }

    private: mcap::McapWriter writer;
    private: mcap::ChannelId channelId;
    private: T msg;
};