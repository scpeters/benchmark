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
#include <string.h>

#include "boxes.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
using namespace benchmark;

const double g_dt_min = 1e-4;
const double g_dt_max = 1.01e-3;
const double g_dt_step = 1.0e-4;

INSTANTIATE_TEST_CASE_P(EnginesDtSimple, BoxesTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Range(g_dt_min, g_dt_max, g_dt_step)
  , ::testing::Values(1)
  , ::testing::Values(true)
  , ::testing::Values(false)));

INSTANTIATE_TEST_CASE_P(EnginesDtComplex, BoxesTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Range(g_dt_min, g_dt_max, g_dt_step)
  , ::testing::Values(1)
  , ::testing::Values(true)
  , ::testing::Values(true)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
