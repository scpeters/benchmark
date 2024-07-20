#include "triball.hh"
#include "gazebo/physics/ContactManager.hh"
using namespace benchmark;
using namespace gazebo;

void TriballTest::triball(const std::string &_physicsEngine, const std::string _frictionModel, 
                          int _Configuration, int _surfaceSlop, float _frictionCoefficient)
{
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

  ignition::math::Vector3 = world->gravity();
  std::size_t modelCount = world->ModelCount();
  ASSERT_EQ(modelCcount, _modelCount);
  std::vector<physics::Contact *> contacts;

  auto models = world->Models()
  std::vector<physics::LinkPtr> links;

  for(auto model : models)
  {
    links.push_back(model->GetLink());
  }
  const double simDuration = 10.0;
  int steps = ceil(simDuration/_dt);

  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common:Time::GetWalltime();
  for(int i = 0; i<steps, ++i)
  {
   double t = (world->simTime() - t0).Double();
   for(int linkIndex = 0; i < links.size(); linkIndex++)
   {
    link = links[linkIndex]
    contact = contacts[linkIndex]
    ASSERT_EQ(contact->count, 3);

    math::Pose3d pose = link->link->WorldInertialPose();
    math::Vector3d linearVelocity = link->WorldLinearVel();
    math::Vector3d angularVelocity = link->WorldAngularVel();
    math::Vector3d linearAcceleration = link->WorldLinearAccel();
    math::Vector3d angularAcceleration = link->WorldAngularAccel();

    for(int j = 0; j < contact->count; j++)
    {
      math::Vector3d contactPosition = contact->positions[j];
      math::Vector3d contactnormal = contact->positions[j];
      math::Vector3d contactForce = contact->wrench[i].body1Force;
      math::vector3d contactTorque = contact->wrench[i].body1Torque;
    }

   }
  }


}
