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

#ifndef BENCHMARK_GAZEBO_TRIBALL_HH_
#define BENCHMARK_GAZEBO_TRIBALL_HH_

#include <string>
#include "gazebo/test/ServerFixture.hh"

namespace gazebo
{
  namespace benchmark
  {
    // physics engine
    // dt
    // number of boxes to spawn
    // collision shape on / off
    // complex trajectory on / off
    typedef std::tr1::tuple < const char *
                            , double
                            , int
                            , bool
                            , bool
                            > char1double1int1bool2;
    class TriballTest : public ServerFixture,
                      public testing::WithParamInterface<char1double1int1bool2>
    {
      /// \brief Test accuracy of unconstrained rigid body motion.
      /// \param[in] _physicsEngine Physics engine to use.
      /// \param[in] _frictionModel Max time step size.
      /// \param[in] _Configuration Number of boxes to spawn.
      /// \param[in] _surfaceSlope Flag for collision shape on / off.
      /// \param[in] _frictionCoefficient Flag for complex trajectory on / off.
      /// \param[in] _velocityEnable Flag for complex trajectory on / off.
      public: void Triball(const std::string &_physicsEngine
                       , const std::string _frictionModel
                       , bool _fixedConfiguration
                       , int _surfaceSlope
                       , float _frictionCoefficient);
    };
  }
}
#endif