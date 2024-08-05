#include <string.h>

#include "triball.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include <ignition/math/Helpers.hh>


using namespace gazebo;
using namespace benchmark;


const double slope_min = 0; 
const double slope_max = IGN_PI/6;
const double slope_step = IGN_PI/24;
const float friction_coefficient = 0.9;

// cog height with respect to link frame
const double cog_h_max = 0.02;
const double cog_h_min = -0.02;
const double cog_h_step = 0.01;

INSTANTIATE_TEST_CASE_P(
    OdeTriball, TriballTest,
    ::testing::Combine(::testing::Values("ode"), ::testing::Values("pyramid"),
                       ::testing::Values(false), ::testing::Range(slope_min, slope_max, slope_step),
                       ::testing::Values(friction_coefficient), 
                       ::testing::Range(cog_h_min, cog_h_max, cog_h_step)));

// INSTANTIATE_TEST_CASE_P(
//     BulletTriball, TriballTest,
//     ::testing::Combine(::testing::Values("bullet"), ::testing::Values("pyramid"),
//                        ::testing::Values(false), ::testing::Range(slope_min, slope_max, slope_step),
//                        ::testing::Values(friction_coefficient), 
//                        ::testing::Range(cog_h_min, cog_h_max, cog_h_step)));
/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}