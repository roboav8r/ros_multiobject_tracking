#include <ros/ros.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "dynamics_models.h"
#include "gaussian_datatypes.h"
#include "multiobject_tracking_node.h"
#include "sensor_callbacks.h"
#include "sensor_models.h"
#include "trackers.h"

// #include "foo/foo.h"

TEST(TestSuite, dummy_test)
{
  ASSERT_TRUE(true);
}

// DYNAMICS MODELS test suite
// TEST(DynamicsModelSuite, defConstructorTest)
// {
//   Eigen::Matrix<>
  
// }
// Test constructor 2





// GAUSSIAN DATA TYPES test suite
// Gaussian Model
// Test clear()

// Gaussian Mixture
// Test sort function
// Test select best gaussian function
// Test select close gaussian function
// Test merge function
// Test prune function
// Test mahalanobid distance function
// Test normalize functions


// BIRTH test suite

// SPAWN test suite

// STATE predict test suite

// GMPHD test suite



// // Declare a test
// TEST(TestSuite, testCase1)
// {
// <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// // Declare another test
// TEST(TestSuite, testCase2)
// {
// <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}