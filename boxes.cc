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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <sstream>
#include "boxes.hh"

using namespace gazebo;
using namespace benchmark;

/////////////////////////////////////////////////
// Boxes:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void BoxesTest::Boxes(const std::string& _physicsEngine, double _dt, int _modelCount, bool _collision, bool _complex)
{
  ASSERT_GT(_modelCount, 0);

  std::stringstream command;

  command << "engine_arg=" << _physicsEngine << " count_arg=" << _modelCount << " collision_arg=" << _collision
          << " complex_arg=" << _complex << " dt_arg =" << _dt << " erb " << WORLDS_PATH << "/boxes.world.erb > "
          << WORLDS_PATH << "/boxes.world";

  // creating model with desired configuration
  auto model_check = system(command.str().c_str());
  // checking if model is created
  ASSERT_EQ(model_check, 0);

  // Load a blank world (no ground plane)
  Load("worlds/boxes.world", true, _physicsEngine);
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
  const ignition::math::Matrix3d I0(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);

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

  if (!_complex)
  {
    v0.Set(-0.9, 0.4, 0.1);
    // Use angular velocity with one non-zero component
    // to ensure linear angular trajectory
    w0.Set(0.5, 0, 0);
    E0 = 5.001041625;
  }
  else
  {
    v0.Set(-2.0, 2.0, 8.0);
    // Since Ixx > Iyy > Izz,
    // angular velocity with large y component
    // will cause gyroscopic tumbling
    w0.Set(0.1, 5.0, 0.1);
    E0 = 368.54641249999997;
  }

  for (auto model : models)
  {
    link = model->GetLink();
    ASSERT_NE(link, nullptr);
  }
  // adding a small delay (waiting for the model to load properly)
  common::Time::MSleep(50);

  ASSERT_EQ(v0, link->WorldCoGLinearVel());
  ASSERT_EQ(w0, link->WorldAngularVel());
  ASSERT_EQ(I0, link->GetInertial()->MOI());
  ASSERT_NEAR(link->GetWorldEnergy(), E0, 1e-5);

  // initial time
  common::Time t0 = world->SimTime();

  // initial linear position in global frame
  ignition::math::Vector3d p0 = link->WorldInertialPose().Pos();

  // initial angular momentum in global frame
  ignition::math::Vector3d H0 = link->WorldAngularMomentum();
  ASSERT_EQ(H0, ignition::math::Vector3d(Ixx, Iyy, Izz) * w0);
  double H0mag = H0.Length();

  const double simDuration = 10.0;
  int steps = ceil(simDuration / _dt);

  // variables to compute statistics on
  ignition::math::Vector3Stats linearPositionError;
  ignition::math::Vector3Stats linearVelocityError;
  ignition::math::Vector3Stats angularPositionError;
  ignition::math::Vector3Stats angularMomentumError;
  ignition::math::SignalStats energyError;
  {
    const std::string statNames = "maxAbs";
    EXPECT_TRUE(linearPositionError.InsertStatistics(statNames));
    EXPECT_TRUE(linearVelocityError.InsertStatistics(statNames));
    EXPECT_TRUE(angularPositionError.InsertStatistics(statNames));
    EXPECT_TRUE(angularMomentumError.InsertStatistics(statNames));
    EXPECT_TRUE(energyError.InsertStatistics(statNames));
  }

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  for (int i = 0; i < steps; ++i)
  {
    world->Step(1);

    // current time
    double t = (world->SimTime() - t0).Double();

    // linear velocity error
    ignition::math::Vector3d v = link->WorldCoGLinearVel();
    linearVelocityError.InsertData(v - (v0 + g * t));

    // linear position error
    ignition::math::Vector3d p = link->WorldInertialPose().Pos();
    linearPositionError.InsertData(p - (p0 + v0 * t + 0.5 * g * t * t));

    // angular momentum error
    ignition::math::Vector3d H = link->WorldAngularMomentum();
    angularMomentumError.InsertData((H - H0) / H0mag);

    // angular position error
    if (!_complex)
    {
      ignition::math::Vector3d a = link->WorldInertialPose().Rot().Euler();
      ignition::math::Quaterniond angleTrue(w0 * t);
      angularPositionError.InsertData(a - angleTrue.Euler());
    }

    // energy error
    energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("wallTime", elapsedTime.Double());
  common::Time simTime = (world->SimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt * 1.1);
  this->Record("simTime", simTime.Double());
  this->Record("timeRatio", elapsedTime.Double() / simTime.Double());

  // Record statistics on pitch and yaw angles
  this->Record("energy0", E0);
  this->Record("energyError_", energyError);
  this->Record("angMomentum0", H0mag);
  this->Record("angMomentumErr_", angularMomentumError.Mag());
  this->Record("angPositionErr", angularPositionError);
  this->Record("linPositionErr_", linearPositionError.Mag());
  this->Record("linVelocityErr_", linearVelocityError.Mag());
}

/////////////////////////////////////////////////
TEST_P(BoxesTest, Boxes)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  double dt = std::tr1::get<1>(GetParam());
  int modelCount = std::tr1::get<2>(GetParam());
  bool collision = std::tr1::get<3>(GetParam());
  bool isComplex = std::tr1::get<4>(GetParam());
  gzdbg << physicsEngine << ", dt: " << dt << ", modelCount: " << modelCount << ", collision: " << collision
        << ", isComplex: " << isComplex << std::endl;
  RecordProperty("engine", physicsEngine);
  this->Record("dt", dt);
  RecordProperty("modelCount", modelCount);
  RecordProperty("collision", collision);
  RecordProperty("isComplex", isComplex);
  Boxes(physicsEngine, dt, modelCount, collision, isComplex);
}
