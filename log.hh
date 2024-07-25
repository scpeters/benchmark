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

    public: void setTriballMsg(const std::string &_physicsEngine, 
                     double &_slope, float &_frictionCoefficient,
                     bool &_complex, const std::string &_frictionModel)
    {
      mcap::Schema schema("benchmark_proto.Triballs_Msg", "protobuf",
                         foxglove::BuildFileDescriptorSet(benchmark_proto::TriballsMsg::descriptor()).SerializeAsString());
    
      writer.addSchema(schema);
      mcap::Channel channel("model_states", "protobuf", schema.id);
      writer.addChannel(channel);
      channelId = channel.id;
      msg.set_physics_engine(_physicsEngine);
      msg.set_slope(_slope);
      msg.set_friction_coefficient(_frictionCoefficient);
      msg.set_complex(_complex);
      msg.set_friction_model(_frictionModel);

      int modelCount;
      if(_complex)
      {
       modelCount = 32;
      }
      else
      {
       modelCount = 5;
      }
     
      for(int i = 0; i < modelCount; i++)
      {
        msg.add_date()->set_model_no(i);
      }
    
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

    public: void recordAccel(int &_modelIdx, const std::vector<double> &_linAccel,
                             const std::vector<double> &_angAccel)
    {
      auto accel = msg.mutable_data(_modelIdx)->add_acceleration();

      accel->mutable_linear()->set_x(_linAccel[0]);
      accel->mutable_linear()->set_y(_linAccel[1]);
      accel->mutable_linear()->set_z(_linAccel[3]);
    
      accel->mutable_angular()->set_x(_angAccel[0]);
      accel->mutable_angular()->set_y(_angAccel[1]);
      accel->mutable_angular()->set_z(_angAccel[3]); 
    }

    public: void recordAccel(int &_modelIdx, const ignition::math::Vector3d &_linAccel,
                             const ignition::math::Vector3d &_angAccel)
    {
      auto accel = msg.mutable_data(_modelIdx)->add_acceleration();

      accel->mutable_linear()->set_x(_linAccel.X());
      accel->mutable_linear()->set_y(_linAccel.Y());
      accel->mutable_linear()->set_z(_linAccel.Z());
    
      accel->mutable_angular()->set_x(_angAccel.X());
      accel->mutable_angular()->set_y(_angAccel.Y());
      accel->mutable_angular()->set_z(_angAccel.Z()); 
    }

    public: void recordContactInfo(int &_modelIdx, const std::vector<double> &_position,
                             const std::vector<double> &_normal, const std::vector<double> 
                             &_force, const std::vector<double> &_torque)
    {
      auto contact_pos = msg.mutable_data(_modelIdx)->add_contact_position();
      auto contact_normal = msg.mutable_data(_modelIdx)->add_contact_normal(); 
      auto contact_wrench = msg.mutable_date(_modelIdx)->add_contact_wrench();

      contact_pos->set_x(_position[0]);
      contact_pos->set_y(_position[1]);
      contact_pos->set_z(_position[2]);
    
      contact_normal->set_x(_normal[0]);
      contact_normal->set_y(_normal[1]);
      contact_normal->set_z(_normal[2]); 

      contact_wrench->mutale_forces()->set_x(_force[0]);
      contact_wrench->mutale_forces()->set_y(_force[1]);
      contact_wrench->mutale_forces()->set_z(_force[2]);

      contact_wrench->mutale_torques()->set_x(_torque[0]);
      contact_wrench->mutale_torques()->set_y(_torque[2]);
      contact_wrench->mutale_torques()->set_z(_torque[3]);
    }

    public: void recordContactInfo(int &_modelIdx, const ignition::math::Vector3d&_position,
                             const ignition::math::Vector3d &_normal, const ignition::math::Vector3d 
                             &_force, const ignition::math::Vector3d &_torque)
    {
      auto contact_pos = msg.mutable_data(_modelIdx)->add_contact_position();
      auto contact_normal = msg.mutable_data(_modelIdx)->add_contact_normal(); 
      auto contact_wrench = msg.mutable_date(_modelIdx)->add_contact_wrench();

      contact_pos->set_x(_position.X());
      contact_pos->set_y(_position.Y());
      contact_pos->set_z(_position.Z());
    
      contact_normal->set_x(_normal.X());
      contact_normal->set_y(_normal.Y());
      contact_normal->set_z(_normal.Z()); 

      contact_wrench->mutale_forces()->set_x(_force.X());
      contact_wrench->mutale_forces()->set_y(_force.Y());
      contact_wrench->mutale_forces()->set_z(_force.Z());

      contact_wrench->mutale_torques()->set_x(_torque.X());
      contact_wrench->mutale_torques()->set_y(_torque.Y());
      contact_wrench->mutale_torques()->set_z(_torque.Z());
    }

    

    private: mcap::McapWriter writer;
    private: mcap::ChannelId channelId;
    private: T msg;                   
};