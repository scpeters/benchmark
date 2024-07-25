#include <string.h>

#include "boxes.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include <gz/math/Helpers.hh>

using namespace gazebo;
using namespace benchmark;

const std::string friction_model = "pyramid";

const double slope_min = 0; 
const double slope_max = GZ_PI/6;
const double slope_step = GZ_PI/12;


INSTANTIATE_TEST_CASE_P(
    OdeBoxes, BoxesTest,
    ::testing::Combine(::testing::Values("ode"), ::testing::Values(friction_model),
                       ::testing::Values(false), ::testing::Range(slope_min, slope_max,slope_step),
                       , ::testing::Values(0.9)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}