#include <ros/ros.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "dynamics_models.h"
#include "gaussian_datatypes.h"
#include "multiobject_tracking_node.h"
#include "sensor_callbacks.h"
#include "sensor_models.h"
#include "trackers.h"

/*
DUMMY test suite
*/ 
TEST(DummyTestSuite, dummyTest)
{
  ASSERT_TRUE(true);
}

/*
DYNAMICS MODEL test suite
*/ 
TEST(DynamicsModelSuite, defConstructorTest)
// Test the default constructor, updateTransMatrix/updateCovMatrix members, and transition/covariance matrix accessors
{
  Eigen::Matrix<float,4,4> expTransMatrix = (Eigen::Matrix4f() << 1, 0, .1, 0, 0, 1, 0, .1, 0, 0, 1, 0, 0, 0, 0, 1).finished();
  Eigen::Matrix<float,4,4> expCovMatrix = (Eigen::Matrix4f() << 2.5e-7, 0, 5e-6, 0, 0, 2.5e-7, 0, 5e-6, 5e-6, 0, 1e-4, 0, 0, 5e-6, 0, 1e-4).finished();

  DynamicsModels::LinearDynamics2D defaultDynModel;

  ASSERT_TRUE(defaultDynModel.TransMatrix().isApprox(expTransMatrix)) << "Expected trans matrix : \n" << expTransMatrix << ",\n got: \n" << defaultDynModel.TransMatrix();
  ASSERT_TRUE(defaultDynModel.CovMatrix().isApprox(expCovMatrix))<< "Expected cov matrix : \n" << expCovMatrix << ",\n got: \n" << defaultDynModel.CovMatrix();
}

TEST(DynamicsModelSuite, secondConstructorTest)
// Test the secondary constructor, updateTransMatrix/updateCovMatrix members, and transition/covariance matrix accessors
{
  Eigen::Matrix<float,4,4> expTransMatrix = (Eigen::Matrix4f() << 1, 0, 1.5, 0, 0, 1, 0, 1.5, 0, 0, 1, 0, 0, 0, 0, 1).finished();
  Eigen::Matrix<float,4,4> expCovMatrix = (Eigen::Matrix4f() << 5.0625, 0, 6.75, 0, 0, 5.0625, 0, 6.75, 6.75, 0, 9, 0, 0, 6.75, 0, 9).finished();

  DynamicsModels::LinearDynamics2D dynModel(1.5,2.0);

  ASSERT_TRUE(dynModel.TransMatrix().isApprox(expTransMatrix)) << "Expected trans matrix : \n" << expTransMatrix << ",\n got: \n" << dynModel.TransMatrix();
  ASSERT_TRUE(dynModel.CovMatrix().isApprox(expCovMatrix))<< "Expected cov matrix : \n" << expCovMatrix << ",\n got: \n" << dynModel.CovMatrix();
}

TEST(DynamicsModelSuite, mutatorTest)
// Test the TransMatrix(dt) and CovMatrix(dt) mutators
{
  Eigen::Matrix<float,4,4> expTransMatrix = (Eigen::Matrix4f() << 1, 0, 1.5, 0, 0, 1, 0, 1.5, 0, 0, 1, 0, 0, 0, 0, 1).finished();
  Eigen::Matrix<float,4,4> expCovMatrix = (Eigen::Matrix4f() << 5.0625, 0, 6.75, 0, 0, 5.0625, 0, 6.75, 6.75, 0, 9, 0, 0, 6.75, 0, 9).finished();

  DynamicsModels::LinearDynamics2D dynModel(0.1,2.0);

  ASSERT_TRUE(dynModel.TransMatrix(1.5).isApprox(expTransMatrix)) << "Expected trans matrix : \n" << expTransMatrix << ",\n got: \n" << dynModel.TransMatrix();
  ASSERT_TRUE(dynModel.CovMatrix(1.5).isApprox(expCovMatrix))<< "Expected cov matrix : \n" << expCovMatrix << ",\n got: \n" << dynModel.CovMatrix();
}



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