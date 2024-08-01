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
#include "log.hh"

using namespace gazebo;
using namespace benchmark;

/////////////////////////////////////////////////
// Boxes:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void BoxesTest::Boxes(const std::string &_physicsEngine, double _dt,
                      int _modelCount, bool _collision, bool _complex) {
   ASSERT_GT(_modelCount, 0);

  // Initialize MCAP logging
  std::string result_name = boost::str(
    boost::format("%1%/%2%/MCAP/boxes_collision"
            "%3%_complex%4%_dt%5$.0e_modelCount%6%_%7%.mcap") % RESULT_DIR_PATH %
      TEST_NAME % _collision % _complex % _dt % _modelCount % _physicsEngine);
  
  Log<benchmark_proto::BoxesMsg> log(result_name);
  bool logMultiple = false;

  // benchmark parameter 
  log.setBoxMsg(_physicsEngine, _dt,  _complex, _collision, _modelCount, logMultiple);


  // Generate world file
  std::string world_erb_path =
      boost::str(boost::format("%1%/boxes.world.erb") % WORLDS_DIR_PATH);

  std::string world_path = boost::str(
      boost::format(
          "%1%/%2%/"
          "boxes_collision%3%_complex%4%_dt%5$.0e_modelCount%6%.world") %
      WORLDS_DIR_PATH % TEST_NAME % _collision % _complex % _dt % _modelCount);

  std::string command = boost::str(
      boost::format(
          "erb collision=%1% complex=%2% dt=%3% modelCount=%4% %5% > %6%") %
      _collision % _complex % _dt % _modelCount % world_erb_path % world_path);

  // creating model with desired configuration
  auto command_check = system(command.c_str());
  // checking if world is created
  ASSERT_EQ(command_check, 0);

  // Load the generated world file
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

  int log_no;
  if(logMultiple)
  {
   log_no = _modelCount; 
  } 
  else
  {
    log_no = 1;
  }

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);

  common::Time startTime = common::Time::GetWallTime();

  for (int i = 0; i < steps; ++i) {
    world->Step(1);
    // current sim time
    double t = (world->SimTime() - t0).Double();
    log.recordSimTime(t);

    // current wall time
    for(int model_no = 0; model_no < log_no ; model_no++){

      auto model = models[model_no];
      link = model->GetLink();
      // linear velocity in world frame
      ignition::math::Vector3d v = link->WorldCoGLinearVel();
      // angular velocity in body frame
      ignition::math::Vector3d a = link->RelativeAngularVel();
      log.recordTwist(model_no, v, a);
  
      // linear position in world frame
      ignition::math::Pose3d pose = link->WorldInertialPose();
      log.recordPose(model_no, pose);
    }
    }

  double elapsedTime = (common::Time::GetWallTime() - startTime).Double();
  log.recordComputationTime(elapsedTime);

  common::Time simTime = (world->SimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt * 1.1);

  log.stop();
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
