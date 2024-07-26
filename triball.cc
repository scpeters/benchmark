/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "PathConfig.h"
#include "triball.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/ContactManager.hh"
#include <boost/format.hpp>
#include <sstream>
#include "log.hh"
using namespace benchmark;
using namespace gazebo;

void TriballTest::triball(const std::string &_physicsEngine, const std::string &_frictionModel, 
                          bool _complex, double _surfaceSlope, float _frictionCoefficient)
{
  
  // logging result directory location
  std::string result_name = boost::str(boost::format("%1%/%2%/MCAP/triball_frictionModel"
                                       "%3%_complex%4%_surfaceSlope%5%_frictionCoefficient%6%_%7%.mcap") 
                                       % RESULT_DIR_PATH % TEST_NAME % _frictionModel % _complex 
                                       % _surfaceSlope % _fricition % _physicsEngine);

  Log<benchmark_proto::TriballsMsg> log(result_name);
  // setting benchmark parameters
  log.setTriballMsg(_physicsEngine, _surfaceSlope, _frictionCoefficient, 
                    _complex, _frictionModel);

  // World creation based on test parameters using boxes.world.erb file.
  std::string world_erb_path =
      boost::str(boost::format("%1%/triball_contact.world.erb") % WORLDS_DIR_PATH);

  std::string worldPath = boost::str(
      boost::format(
          "%1%/%2%/"
          "triball_frictionModel%3%_complex%4%_surfaceSlope%5%_frictionCoefficient%6%.world") %
      WORLDS_DIR_PATH % TEST_NAME % _frictionModel % _complex % _surfaceSlope % _frictionCoefficient);

  // Final world sdf path
  std::string command = boost::str(
      boost::format(
          "erb frictionModel=%1% complex=%2% surfaceSlope=%3% fritionCoefficient=%4% %5% > %6%") %
      _frictionModel % _complex % _surfaceSlope % fritionCoefficient % world_erb_path % worldPath);

  // execute command
  auto commandCheck =  system(command.c_str());
  ASSERT_EQ(commandCheck, 0);

  Load(worldPath, true, _physicsEngie);
 
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(WorldPtr, nullptr);
  

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  physics::ContactManager *mgr = physics->GetContactManager();
  mgr->SetNeverDropContacts(true);
  ASSERT_TRUE(manager->NeverDropContacts());
  ASSERT_GT(mgr->GetContactCount(), 0u);

  // initial time
  common::Time t0 = world->SimTime();

  ignition::math::Vector3d gravity0(-ignition::math::sin(_slope)*9.8, 0,
                             -ignition::math::cos(_slope)*9.8);

  ignition::math::Vector3d gravity = world->gravity();
  // checking if slope is correct
  ASSERT_NE(gravity0, gravity, 0.001)

  std::size_t modelCount = world->ModelCount();
  // checking if number of model spwaned are correct.
  if(!complex)
  {
    ASSERT_EQ(modelCount, 5);
  }
  else
  {
    ASSERT_EQ(modelCount, 32);
  }

  std::vector<physics::Contact *> contacts;

  auto models = world->Models()
  std::vector<physics::LinkPtr> links;

  for(auto model : models)
  {
    links.push_back(model->GetLink());
  }
  
  const double simDuration = 1.0;
  int steps = ceil(simDuration/_dt);

  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common:Time::GetWalltime();
  log.recordSimTime(t);

  for(int i = 0; i<steps, ++i)
  {
   double t = (world->simTime() - t0).Double();

   for(int linkIndex = 0; i < links.size(); linkIndex++)
   {
    link = links[linkIndex]
    contact = contacts[linkIndex]
    ASSERT_EQ(contact->count, 3);

    math::Pose3d pose = link->link->WorldInertialPose();
    log.recordPose(model_no, pose);

    math::Vector3d linearVelocity = link->WorldLinearVel();
    math::Vector3d angularVelocity = link->WorldAngularVel();
    log.recordTwist(model_no, linearVelocity, angularVelocity);

    math::Vector3d linearAcceleration = link->WorldLinearAccel();
    math::Vector3d angularAcceleration = link->WorldAngularAccel();
    log.recordAccel(model_no, linearAcceleration, angularAcceleration);

    for(int j = 0; j < contact->count; j++)
    {
      math::Vector3d contactPosition = contact->positions[j];
      math::Vector3d contactnormal = contact->normal[j];
      math::Vector3d contactForce = contact->wrench[i].body1Force;
      math::vector3d contactTorque = contact->wrench[i].body1Torque;
      log.recordContactInfo(model_no, contactPosition, contactnormal,
                            contactForce, contactTorque);
    }
   }
  }

  double elapsedTime = (common::Time::GetWallTime() - startTime).Double();
  log.recordComputationTime(elapsedTime);

  common::Time simTime = (world->SimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt * 1.1);

  log.stop();
}

/////////////////////////////////////////////////
TEST_P(TriballTest, Triball) {
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  std::string frictionModel = std::tr1::get<1>(GetParam());
  bool complex = std::tr1::get<2>(GetParam());
  double surfaceSlope = std::tr1::get<3>(GetParam());
  double frictionCoefficient = std::tr1::get<4>(GetParam());
  gzdbg << physicsEngine << ", friction model: " << frictionModel << ", complex: " << complex
        << ", surface slope: " << surfaceSlope << ", friction coefficient: " << frictionCoefficient
        << std::endl;
  Boxes(physicsEngine, dt, modelCount, collision, isComplex);
}
