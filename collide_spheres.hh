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

#ifndef BENCHMARK_GAZEBO_COLLIDE_SPHERES_HH_
#define BENCHMARK_GAZEBO_COLLIDE_SPHERES_HH_

#include <string>
#include "gazebo/test/ServerFixture.hh"

namespace gazebo
{
  namespace benchmark
  {
    // physics engine
    // dt
    typedef std::tr1::tuple < const char *
                            , double
                            > char1double1;
    class CollideTest : public ServerFixture,
                        public testing::WithParamInterface<char1double1>
    {
      /// \brief Test collision checking between spheres.
      /// \param[in] _physicsEngine Physics engine to use.
      /// \param[in] _dt Max time step size.
      public: void Spheres(const std::string &_physicsEngine
                         , double _dt);
    };
  }
}
#endif
