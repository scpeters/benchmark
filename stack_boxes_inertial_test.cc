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

#include "stack_boxes.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
using namespace benchmark;

const int top_box_mass_min = 21;
const int top_box_mass_max = 45;
const int mass_step = 20;

const double g_dt_min = 5e-4;
const double g_dt_max = 1.01e-3;
const double g_dt_step = 3.0e-4;

INSTANTIATE_TEST_CASE_P(
  OdeBoxes, BoxesTest,
  ::testing::Combine(
    ::testing::Values("ode"),
    ::testing::Range(g_dt_min, g_dt_max, g_dt_step),
    ::testing::Values(6),
    ::testing::Values(true),
    ::testing::Values(true),
    ::testing::Range(top_box_mass_min, top_box_mass_max, mass_step)));

#ifdef HAVE_BULLET
INSTANTIATE_TEST_CASE_P(
  BulletBoxes, BoxesTest,
  ::testing::Combine(
    ::testing::Values("bullet"),
    ::testing::Range(g_dt_min, g_dt_max, g_dt_step),
    ::testing::Values(6),
    ::testing::Values(true),
    ::testing::Values(true),
    ::testing::Range(top_box_mass_min, top_box_mass_max, mass_step)));
#endif

// #ifdef HAVE_SIMBODY
// INSTANTIATE_TEST_CASE_P(
//   SimbodyBoxes, BoxesTest,
//   ::testing::Combine(
//     ::testing::Values("simbody"),
//     ::testing::Values(1.0e-3),
//     ::testing::Values(6),
//     ::testing::Values(true),
//     ::testing::Values(true),
//     ::testing::Range(top_box_mass_min, g_dt_max, g_dt_step)));
// #endif

#ifdef HAVE_DART
INSTANTIATE_TEST_CASE_P(
  DartBoxes, BoxesTest,
  ::testing::Combine(
    ::testing::Values("dart"),
    ::testing::Range(g_dt_min, g_dt_max, g_dt_step),
    ::testing::Values(6),
    ::testing::Values(true),
    ::testing::Values(true),
    ::testing::Range(top_box_mass_min, top_box_mass_max, mass_step)));
#endif

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
