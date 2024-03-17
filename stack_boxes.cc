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
#include <cmath>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "stack_boxes.hh"

using namespace gazebo;
using namespace benchmark;

/////////////////////////////////////////////////
// Boxes:
// Spawn a single box and record accuracy for momentum and enery
// conservation
// _mass of top box
void BoxesTest::Boxes(
  const std::string & _physicsEngine,
  double _dt,
  int _modelCount,
  bool _collision,
  bool _complex,
  int _mass)
{
  // Load a emppty world (with ground plane)
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  // get gravity value
  if (!_complex) {
    world->SetGravity(ignition::math::Vector3d::Zero);
  }
  ignition::math::Vector3d g = world->Gravity();

  // Box size
  const double dx = 1;
  const double dy = 1;
  const double dz = 1;

  // mass of boxes other than top box
  const double mass = 1.0;  

  // Create box with inertia based on box of uniform density
  msgs::Model msgModel;
  msgs::AddBoxLink(msgModel, mass, ignition::math::Vector3d(dx, dy, dz));
  
  // spawn multiple boxes
  // compute error statistics only on the last box
  ASSERT_GT(_modelCount, 0);
  physics::ModelPtr model;

  ignition::math::Vector3d initialPosition(0.0, 0.0, dz / 2);

  for (int i = 0; i < _modelCount - 1; ++i) {
    // give models unique names
    msgModel.set_name(this->GetUniqueString("model"));
    std::string modelName = msgModel.name();

    // Set the position of each box incrementally higher
    msgs::Set(
      msgModel.mutable_pose()->mutable_position(),
      initialPosition + ignition::math::Vector3d(0.0, 0.0, dz * i));

    model = this->SpawnModel(msgModel);
    ASSERT_NE(model, nullptr);

  }

  const double top_box_Ixx = std::round((1.0 / 12.0) * _mass * (dy * dy + dz * dz) * 1e8) / 1e8;
  const double top_box_Iyy = std::round((1.0 / 12.0) * _mass * (dx * dx + dz * dz) * 1e8) / 1e8;
  const double top_box_Izz = std::round((1.0 / 12.0) * _mass * (dx * dx + dy * dy) * 1e8) / 1e8;

  const ignition::math::Matrix3d I0(
    top_box_Ixx, 0.0, 0.0,
    0.0, top_box_Iyy, 0.0,
    0.0, 0.0, top_box_Izz);

  msgs::Model msgModel_;

  msgs::AddBoxLink(msgModel_, _mass, ignition::math::Vector3d(dx, dy, dz));


  msgModel_.set_name(this->GetUniqueString("model"));
  std::string modelName = msgModel_.name();

  // Set the position of each box incrementally higher
  msgs::Set(
    msgModel_.mutable_pose()->mutable_position(),
    initialPosition + ignition::math::Vector3d(0.0, 0.0, dz * (_modelCount-1)));

  model = this->SpawnModel(msgModel_);
  ASSERT_NE(model, nullptr);

  physics::LinkPtr link;
  link = model->GetLink();
  ASSERT_NE(link, nullptr);


  // ASSERT_EQ(I0, link->GetInertial()->MOI());

  // initial time
  common::Time t0 = world->SimTime();

  // initial linear position in global frame
  ignition::math::Vector3d p0 = link->WorldInertialPose().Pos();

  // initial angular momentum in global frame
  ignition::math::Vector3d H0 = link->WorldAngularMomentum();
  // ASSERT_EQ(H0, ignition::math::Vector3d(Ixx, Iyy, Izz) * w0);
  double H0mag = H0.Length();

  // change step size after setting initial conditions
  // since simbody requires a time step
  physics->SetMaxStepSize(_dt);
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
  for (int i = 0; i < steps; ++i) {
    world->Step(1);

    // current time
    double t = (world->SimTime() - t0).Double();

    // linear velocity error
    ignition::math::Vector3d v = link->WorldCoGLinearVel();
    // linearVelocityError.InsertData(v - (v0 + g * t));

    // linear position error
    ignition::math::Vector3d p = link->WorldInertialPose().Pos();
    // linearPositionError.InsertData(p - (p0 + v0 * t + 0.5 * g * t * t));

    // angular momentum error
    ignition::math::Vector3d H = link->WorldAngularMomentum();
    angularMomentumError.InsertData((H - H0) / H0mag);

    // angular position error
    if (!_complex) {
      ignition::math::Vector3d a = link->WorldInertialPose().Rot().Euler();
      // ignition::math::Quaterniond angleTrue(w0 * t);
      // angularPositionError.InsertData(a - angleTrue.Euler());
    }

    // energy error
    // energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("wallTime", elapsedTime.Double());
  common::Time simTime = (world->SimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt * 1.1);
  this->Record("simTime", simTime.Double());
  this->Record("timeRatio", elapsedTime.Double() / simTime.Double());

  // Record statistics on pitch and yaw angles
  // this->Record("energy0", E0);
  // this->Record("energyError_", energyError);
  this->Record("angMomentum0", H0mag);
  this->Record("angMomentumErr_", angularMomentumError.Mag());
  // this->Record("angPositionErr", angularPositionError);
  this->Record("linPositionErr_", linearPositionError.Mag());
  this->Record("linVelocityErr_", linearVelocityError.Mag());
}

/////////////////////////////////////////////////
TEST_P(BoxesTest, Boxes)
{
  std::string physicsEngine = std::tr1::get < 0 > (GetParam());
  double dt = std::tr1::get < 1 > (GetParam());
  int modelCount = std::tr1::get < 2 > (GetParam());
  bool collision = std::tr1::get < 3 > (GetParam());
  bool isComplex = std::tr1::get < 4 > (GetParam());
  int mass = std::tr1::get < 5 > (GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << ", modelCount: " << modelCount
        << ", collision: " << collision
        << ", isComplex: " << isComplex
        << ", mass of top box: " << mass
        << std::endl;
  RecordProperty("engine", physicsEngine);
  this->Record("dt", dt);
  RecordProperty("modelCount", modelCount);
  RecordProperty("collision", collision);
  RecordProperty("isComplex", isComplex);
  RecordProperty("mass", mass);
  Boxes(
    physicsEngine,
    dt,
    modelCount,
    collision,
    isComplex,
    mass);
}
