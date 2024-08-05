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

using namespace gazebo;
using namespace benchmark;

void TriballTest::Triball(const std::string &_physicsEngine, const std::string &_frictionModel, 
                          bool _complex, double _surfaceSlope, float _frictionCoefficient
                          , double _cogH)
{
  
  // logging result directory location
  std::string result_name = boost::str(boost::format("%1%/%2%/MCAP/triball_frictionModel"
                                       "%3%_complex%4%_surfaceSlope%5%_frictionCoefficient%6%_cogH%7%_%8%.mcap") 
                                       % RESULT_DIR_PATH % TEST_NAME % _frictionModel % _complex 
                                       % _surfaceSlope % _frictionCoefficient % _cogH % _physicsEngine);

  Log<benchmark_proto::TriballsMsg> log(result_name);
  // setting benchmark parameters
  log.setTriballMsg(_physicsEngine, _surfaceSlope, _frictionCoefficient, 
                    _complex, _frictionModel, _cogH);

  // World creation based on test parameters using boxes.world.erb file.
  std::string world_erb_path =
      boost::str(boost::format("%1%/triball_contact.world.erb") % WORLDS_DIR_PATH);

  std::string worldPath = boost::str(
      boost::format(
          "%1%/%2%/"
          "triball_frictionModel%3%_complex%4%_surfaceSlope%5%_frictionCoefficient%6%_cogH%7%.world") %
      WORLDS_DIR_PATH % TEST_NAME % _frictionModel % _complex % _surfaceSlope % _frictionCoefficient %
      _cogH);

  // Final world sdf path
  std::string command = boost::str(
      boost::format(
          "erb frictionModel=%1% complex=%2% surfaceSlope=%3% fritionCoefficient=%4% cogH=%5% %6% > %7%") %
      _frictionModel % _complex % _surfaceSlope % _frictionCoefficient % _cogH % world_erb_path % worldPath);
  
  // execute command
  auto commandCheck =  system(command.c_str());
  ASSERT_EQ(commandCheck, 0);

  Load(worldPath, true, _physicsEngine);
 
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);
  ASSERT_EQ(physics->GetType(), _physicsEngine);
  physics->SetRealTimeUpdateRate(0.0);

  physics::ContactManager *mgr = physics->GetContactManager();
  mgr->SetNeverDropContacts(true);
  world->Step(1);
  
  // std::cout << "number of contacts " << mgr->GetContactCount();
  ASSERT_TRUE(mgr->NeverDropContacts());
  ASSERT_GT(mgr->GetContactCount(), 0);

  // initial time
  common::Time t0 = world->SimTime();

  ignition::math::Vector3d gravity = world->Gravity();

  std::size_t modelCount = world->ModelCount();
  // checking if number of model spwaned are correct.
  if(!_complex)
  {
    ASSERT_EQ(modelCount, 6);
  }
  else
  {
    ASSERT_EQ(modelCount, 33);
  }

  auto contacts = mgr->GetContacts();
  auto models = world->Models();
  int contactCount = 3;
  EXPECT_EQ(mgr->GetContactCount(), contactCount*(modelCount - 1));

  physics::LinkPtr link; 
  
  double _dt = 0.001;
  const double simDuration = 1.0;
  int steps = ceil(simDuration/_dt);

  std::vector<std::string> collisionNames{"collision_a", "collision_b", "collision_c"};
  collisionNames.reserve(3);
  
  common::Time startTime = common::Time::GetWallTime();

  for(int i = 0; i<steps; ++i)
  {
   world->Step(1);
   double t = (world->SimTime() - t0).Double();
   log.recordSimTime(t);

   for(int model_no = 1; model_no < modelCount; model_no++)
   {
    auto model = models[model_no];
    link = model->GetLink();
    auto modelIdx = model_no - 1;

    ignition::math::Pose3d pose = link->WorldInertialPose();
    log.recordPose(modelIdx, pose);

    ignition::math::Vector3d linearVelocity = link->WorldLinearVel();
    ignition::math::Vector3d angularVelocity = link->WorldAngularVel();
    log.recordTwist(modelIdx, linearVelocity, angularVelocity);

    ignition::math::Vector3d linearAcceleration = link->WorldLinearAccel();
    ignition::math::Vector3d angularAcceleration = link->WorldAngularAccel();
    log.recordAccel(modelIdx, linearAcceleration, angularAcceleration);

    for(int contactIdx = 0; contactIdx < collisionNames.size(); contactIdx++)
    { 
      bool contactStatus = false;
      ignition::math::Vector3d contactPosition;
      ignition::math::Vector3d contactNormal;
      ignition::math::Vector3d contactForce;
      ignition::math::Vector3d contactTorque;

      for(auto contact : contacts)
      { 
        std::string scopedName(contact->collision1->GetScopedName());
        std::string collisionName(scopedName.substr(scopedName.rfind("::") +2));

        if(collisionName == collisionNames[contactIdx] && 
           contact->collision1->GetLink() == link)
        { 
          contactStatus = true;
          contactPosition = contact->positions[0];
          contactNormal = contact->normals[0];
          contactForce = contact->wrench[0].body1Force;
          contactTorque = contact->wrench[0].body1Torque;
        }  
      }

      log.recordContactInfo(modelIdx, contactIdx, contactStatus, contactPosition, 
                              contactNormal, contactForce, contactTorque);
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
  float frictionCoefficient = std::tr1::get<4>(GetParam());
  float cogH = std::tr1::get<5>(GetParam());

  gzdbg << physicsEngine << ", friction model: " << frictionModel << ", complex: " << complex
        << ", surface slope: " << surfaceSlope << ", friction coefficient: " << frictionCoefficient
        << "center of gravity of height" << cogH << std::endl;

  Triball(physicsEngine, frictionModel, complex, surfaceSlope, frictionCoefficient, cogH);
}
