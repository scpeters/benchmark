/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#define MCAP_IMPLEMENTATION
#include "mcap/writer.hpp"
#include "protobuf/BuildFileDescriptorSet.h"
#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "PathConfig.h"
#include "boxes.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include <boost/format.hpp>
#include <sstream>
#include <boxes_msg.pb.h>

using namespace gazebo;
using namespace benchmark;

/////////////////////////////////////////////////
// Boxes:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void BoxesTest::Boxes(const std::string &_physicsEngine, double _dt,
                      int _modelCount, bool _collision, bool _complex) {
  ASSERT_GT(_modelCount, 0);
  std::string world_erb_path =
      boost::str(boost::format("%1%/boxes.world.erb") % WORLDS_DIR_PATH);
  

  std::string world_path = boost::str(
      boost::format(
          "%1%/%2%/"
          "boxes_collision%3%_complex%4%_dt%5$.0e_modelCount%6%.world") %
      WORLDS_DIR_PATH % TEST_NAME % _collision % _complex % _dt % _modelCount);

  std::string result_name = boost::str(
    boost::format("%1%/%2%/MCAP/boxes_collision"
            "%3%_complex%4%_dt%5$.0e_modelCount%6%_%7%.mcap") % RESULT_DIR_PATH %
      TEST_NAME % _collision % _complex % _dt % _modelCount % _physicsEngine);

  std::string command = boost::str(
      boost::format(
          "erb collision=%1% complex=%2% dt=%3% modelCount=%4% %5% > %6%") %
      _collision % _complex % _dt % _modelCount % world_erb_path % world_path);
  
  bool log_all = true;
  // mcap writer 
  mcap::McapWriter writer;
  auto options = mcap::McapWriterOptions("");
  const auto s = writer.open(result_name.c_str(), options);
  ASSERT_EQ(s.ok(), true);

  mcap::ChannelId channelId;

  {
    mcap::Schema schema(
      "benchmark_proto.Boxes_msg", "protobuf",
      foxglove::BuildFileDescriptorSet(benchmark_proto::Boxes_msg::descriptor()).SerializeAsString());
    writer.addSchema(schema);
    mcap::Channel channel("boxes_states", "protobuf", schema.id);
    writer.addChannel(channel);
    channelId = channel.id;
  }

  // creating model with desired configuration
  auto model_check = system(command.c_str());
  // checking if model is created
  ASSERT_EQ(model_check, 0);

  // Load a blank world (no ground plane)
  Load(world_path, true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  ignition::math::Vector3d g = world->Gravity();

  const double Ixx = 0.80833333;
  const double Iyy = 0.68333333;
  const double Izz = 0.14166667;
  const ignition::math::Matrix3d I0(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0,
                                    Izz);

  benchmark_proto::Boxes_msg boxes_msg;

  boxes_msg.set_physics_engine(_physicsEngine);
  boxes_msg.set_dt(_dt);
  boxes_msg.set_complex(_complex);
  boxes_msg.set_collision(_collision);
  boxes_msg.set_model_count(_modelCount);
  

  // physics::ModelPtr model;
  std::size_t model_count = world->ModelCount();
  ASSERT_EQ(model_count, _modelCount);

  auto models = world->Models();
  physics::LinkPtr link;

  // initial linear velocity in global frame
  ignition::math::Vector3d v0;

  // initial angular velocity in global frame
  ignition::math::Vector3d w0;

  // initial energy value
  double E0;

  if (!_complex) {
    v0.Set(-0.9, 0.4, 0.1);
    // Use angular velocity with one non-zero component
    // to ensure linear angular trajectory
    w0.Set(0.5, 0, 0);
    E0 = 5.001041625;
  } else {
    v0.Set(-2.0, 2.0, 8.0);
    // Since Ixx > Iyy > Izz,
    // angular velocity with large y component
    // will cause gyroscopic tumbling
    w0.Set(0.1, 5.0, 0.1);
    E0 = 368.54641249999997;
  }
  // adding a small delay (waiting for the model to load properly)
  common::Time::MSleep(50);


  if(log_all){
    for(int model_no =1; model_no<=_modelCount; model_no++){
      boxes_msg.add_data()->set_model_no(model_no);
    }
  }
  else{
      boxes_msg.add_data()->set_model_no(1);
  }

  for (auto model : models) {
    link = model->GetLink();
    ASSERT_NE(link, nullptr);
    ASSERT_EQ(v0, link->WorldCoGLinearVel());
    ASSERT_EQ(w0, link->WorldAngularVel());
    ASSERT_EQ(I0, link->GetInertial()->MOI(link->GetInertial()->Pose()));
    ASSERT_NEAR(link->GetWorldEnergy(), E0, 1e-5);
  }

  // initial time
  common::Time t0 = world->SimTime();

  // initial linear position in global frame
  ignition::math::Vector3d p0 = link->WorldInertialPose().Pos();

  // initial angular momentum in global frame
  ignition::math::Vector3d H0 = link->WorldAngularMomentum();
  ASSERT_EQ(H0, ignition::math::Vector3d(Ixx, Iyy, Izz) * w0);

  const double simDuration = 10.0;
  int steps = ceil(simDuration / _dt);

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  int model_no = 1;
  for (int i = 0; i < steps; ++i) {
    world->Step(1);
    // current wall time
    if (log_all){
      for(auto model : models){

        link = model->GetLink();
        common::Time elapsedTime = common::Time::GetWallTime() - startTime;
        boxes_msg.mutable_data(model_no - 1)->add_computation_time(elapsedTime.Double());
        common::Time loop_start_time = common::Time::GetWallTime();
        // current time
        double t = (world->SimTime() - t0).Double();
        boxes_msg.mutable_data(model_no - 1)->add_sim_time(t);
    
        // linear velocity 
        ignition::math::Vector3d v = link->WorldCoGLinearVel();
        // angular velocity
        ignition::math::Vector3d a = link->WorldAngularVel();
        auto twist = boxes_msg.mutable_data(model_no - 1)->add_twists();
        twist->mutable_linear()->set_x(v.X());
        twist->mutable_linear()->set_y(v.Y());
        twist->mutable_linear()->set_z(v.Z());
        twist->mutable_angular()->set_x(a.X());
        twist->mutable_angular()->set_y(a.Y());
        twist->mutable_angular()->set_z(a.Z());
    
        // linear position 
        ignition::math::Vector3d p = link->WorldInertialPose().Pos();
        // angular position
        ignition::math::Quaterniond o = link->WorldInertialPose().Rot();
        auto pose = boxes_msg.mutable_data(model_no - 1)->add_poses();
        pose->mutable_position()->set_x(p.X());
        pose->mutable_position()->set_y(p.Y());
        pose->mutable_position()->set_z(p.Z());
        pose->mutable_orientation()->set_w(o.W());
        pose->mutable_orientation()->set_x(o.X());
        pose->mutable_orientation()->set_y(o.Y());
        pose->mutable_orientation()->set_z(o.Z());
        model_no++;
      }
      model_no = 1;
      }
    else{

      common::Time elapsedTime = common::Time::GetWallTime() - startTime;
      boxes_msg.mutable_data(0)->add_computation_time(elapsedTime.Double());
      common::Time loop_start_time = common::Time::GetWallTime();
      // current time
      double t = (world->SimTime() - t0).Double();
      boxes_msg.mutable_data(0)->add_sim_time(t);
  
      // linear velocity 
      ignition::math::Vector3d v = link->WorldCoGLinearVel();
      // angular velocity
      ignition::math::Vector3d a = link->WorldAngularVel();
      auto twist = boxes_msg.mutable_data(0)->add_twists();
      twist->mutable_linear()->set_x(v.X());
      twist->mutable_linear()->set_y(v.Y());
      twist->mutable_linear()->set_z(v.Z());
      twist->mutable_angular()->set_x(a.X());
      twist->mutable_angular()->set_y(a.Y());
      twist->mutable_angular()->set_z(a.Z());
  
      // linear position 
      ignition::math::Vector3d p = link->WorldInertialPose().Pos();
      // angular position
      ignition::math::Quaterniond o = link->WorldInertialPose().Rot();
      auto pose = boxes_msg.mutable_data(0)->add_poses();
      pose->mutable_position()->set_x(p.X());
      pose->mutable_position()->set_y(p.Y());
      pose->mutable_position()->set_z(p.Z());
      pose->mutable_orientation()->set_w(o.W());
      pose->mutable_orientation()->set_x(o.X());
      pose->mutable_orientation()->set_y(o.Y());
      pose->mutable_orientation()->set_z(o.Z());
    }
    }
  

  common::Time simTime = (world->SimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt * 1.1);

  std::string serialized = boxes_msg.SerializeAsString();
  mcap::Message msg;
  msg.channelId = channelId;
  msg.data = reinterpret_cast<const std::byte*>(serialized.data());
  msg.dataSize = serialized.size();
  const auto res = writer.write(msg);
  if (!res.ok()) {
    writer.terminate();
    writer.close();
    ASSERT_EQ(res.ok(), true);
  }
  
  writer.close();
}

/////////////////////////////////////////////////
TEST_P(BoxesTest, Boxes) {
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  double dt = std::tr1::get<1>(GetParam());
  int modelCount = std::tr1::get<2>(GetParam());
  bool collision = std::tr1::get<3>(GetParam());
  bool isComplex = std::tr1::get<4>(GetParam());
  gzdbg << physicsEngine << ", dt: " << dt << ", modelCount: " << modelCount
        << ", collision: " << collision << ", isComplex: " << isComplex
        << std::endl;
  Boxes(physicsEngine, dt, modelCount, collision, isComplex);
}
