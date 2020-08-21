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
#include <map>
#include <string>
#include <utility>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "collide_spheres.hh"

using namespace gazebo;
using namespace benchmark;

void OnContacts(ConstContactsPtr &/*_msg*/)
{
}

typedef std::map<double, std::pair<physics::ModelPtr, physics::ModelPtr>>
        mapSeparatedModels;
/////////////////////////////////////////////////
// Collide spheres:
// Load world with many pairs of spheres in contact.
// Disable physics and verify collision checking.
void CollideTest::Spheres(const std::string &_physicsEngine
                        , double _dt)
{
  // Load collide_spheres world
  Load("worlds/collide_spheres.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("collide_spheres");
  ASSERT_NE(world, nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  // Disable physics updates
  world->SetPhysicsEnabled(false);

  // Models with 1mm and 100mm radius
  auto models = world->Models();
  mapSeparatedModels mmRadius, dmRadius;
  for (const auto model : models)
  {
    const auto name = model->GetName();
    double radius;
    if (name.find("mm") == 0)
    {
      double separation = std::stod(name.substr(2, 2)) / 10;
      if (name.at(4) == 'A')
      {
        mmRadius[separation].first = model;
      }
      else if (name.at(4) == 'B')
      {
        mmRadius[separation].second = model;
      }
      else
      {
        gzerr << "Unrecognized model name: " << name << std::endl;
      }
    }
    else if (name.find("dm") == 0)
    {
      double separation = std::stod(name.substr(2, 2)) / 10;
      if (name.at(4) == 'A')
      {
        dmRadius[separation].first = model;
      }
      else if (name.at(4) == 'B')
      {
        dmRadius[separation].second = model;
      }
      else
      {
        gzerr << "Unrecognized model name: " << name << std::endl;
      }
    }
  }

  // Confirm no models are missing a partner
  for (const auto mmPair : mmRadius)
  {
    gzdbg << "Checking mm radius pair with separation "
          << mmPair.first
          << std::endl;
    ASSERT_NE(mmPair.second.first, nullptr);
    ASSERT_NE(mmPair.second.second, nullptr);
  }
  for (const auto dmPair : dmRadius)
  {
    gzdbg << "Checking dm radius pair with separation "
          << dmPair.first
          << std::endl;
    ASSERT_NE(dmPair.second.first, nullptr);
    ASSERT_NE(dmPair.second.second, nullptr);
  }

  // compute distance between object centers
  for (const auto mmPair : mmRadius)
  {
    auto modelA = mmPair.second.first;
    auto modelB = mmPair.second.second;
    auto positionDiff = modelA->WorldPose().Pos()
                      - modelB->WorldPose().Pos();
    EXPECT_DOUBLE_EQ(positionDiff.Length(), mmPair.first * 1e-3);
  }
  for (const auto dmPair : dmRadius)
  {
    auto modelA = dmPair.second.first;
    auto modelB = dmPair.second.second;
    auto positionDiff = modelA->WorldPose().Pos()
                      - modelB->WorldPose().Pos();
    EXPECT_DOUBLE_EQ(positionDiff.Length(), dmPair.first * 1e-1);
  }

  // You have to subscribe to a contacts topic in order to use
  // the C++ API, otherwise it skips it to save CPU time
  auto contactSub = this->node->Subscribe("~/physics/contacts", &OnContacts);

  world->Step(1);

  // Contact data
  auto contactManager = physics->GetContactManager();
  ASSERT_NE(contactManager, nullptr);
  unsigned int contactCount = contactManager->GetContactCount();
  EXPECT_EQ(contactCount, 16u);
  auto contacts = contactManager->GetContacts();

  for (unsigned int i = 0; i < contactCount; ++i)
  {
    const auto contact = contacts[i];
    EXPECT_EQ(contact->count, 1);
    if (contact->count != 1)
      continue;
    gzdbg << contact->collision1->GetScopedName()
          << " "
          << contact->collision2->GetScopedName()
          << " "
          << contact->normals[0] << " normal, "
          << contact->depths[0] << " depth"
          << std::endl;
  }

  // Recording data
  // // Record statistics on pitch and yaw angles
  // this->Record("energy0", E0);
  // this->Record("energyError_", energyError);
  // this->Record("angMomentum0", H0mag);
  // this->Record("angMomentumErr_", angularMomentumError.Mag());
  // this->Record("angPositionErr", angularPositionError);
  // this->Record("linPositionErr_", linearPositionError.Mag());
  // this->Record("linVelocityErr_", linearVelocityError.Mag());
}

/////////////////////////////////////////////////
TEST_P(CollideTest, Spheres)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  double dt                 = std::tr1::get<1>(GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << std::endl;
  RecordProperty("engine", physicsEngine);
  this->Record("dt", dt);
  Spheres(physicsEngine
        , dt);
}
