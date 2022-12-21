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
// DUMMY test suite
*/ 
TEST(DummyTestSuite, dummyTest)
{
  ASSERT_TRUE(true);
}

/*
// DYNAMICS MODEL test suite
*/ 
TEST(DynamicsModelSuite, dynConstructorTest)
// Test the default constructor, updateTransMatrix/updateCovMatrix members, and transition/covariance matrix accessors
{
  Eigen::Matrix<float,4,4> expTransMatrix = (Eigen::Matrix4f() << 1, 0, .1, 0, 0, 1, 0, .1, 0, 0, 1, 0, 0, 0, 0, 1).finished();
  Eigen::Matrix<float,4,4> expCovMatrix = (Eigen::Matrix4f() << 2.5e-7, 0, 5e-6, 0, 0, 2.5e-7, 0, 5e-6, 5e-6, 0, 1e-4, 0, 0, 5e-6, 0, 1e-4).finished();

  DynamicsModels::LinearDynamics2D defaultDynModel;

  ASSERT_TRUE(defaultDynModel.TransMatrix().isApprox(expTransMatrix)) << "Expected trans matrix : \n" << expTransMatrix << ",\n got: \n" << defaultDynModel.TransMatrix();
  ASSERT_TRUE(defaultDynModel.CovMatrix().isApprox(expCovMatrix))<< "Expected cov matrix : \n" << expCovMatrix << ",\n got: \n" << defaultDynModel.CovMatrix();
}

TEST(DynamicsModelSuite, dynConstructorTest2)
// Test the secondary constructor, updateTransMatrix/updateCovMatrix members, and transition/covariance matrix accessors
{
  Eigen::Matrix<float,4,4> expTransMatrix = (Eigen::Matrix4f() << 1, 0, 1.5, 0, 0, 1, 0, 1.5, 0, 0, 1, 0, 0, 0, 0, 1).finished();
  Eigen::Matrix<float,4,4> expCovMatrix = (Eigen::Matrix4f() << 5.0625, 0, 6.75, 0, 0, 5.0625, 0, 6.75, 6.75, 0, 9, 0, 0, 6.75, 0, 9).finished();

  DynamicsModels::LinearDynamics2D dynModel(1.5,2.0);

  ASSERT_TRUE(dynModel.TransMatrix().isApprox(expTransMatrix)) << "Expected trans matrix : \n" << expTransMatrix << ",\n got: \n" << dynModel.TransMatrix();
  ASSERT_TRUE(dynModel.CovMatrix().isApprox(expCovMatrix))<< "Expected cov matrix : \n" << expCovMatrix << ",\n got: \n" << dynModel.CovMatrix();
}

TEST(DynamicsModelSuite, dynMutatorTest)
// Test the TransMatrix(dt) and CovMatrix(dt) mutators
{
  Eigen::Matrix<float,4,4> expTransMatrix = (Eigen::Matrix4f() << 1, 0, 1.5, 0, 0, 1, 0, 1.5, 0, 0, 1, 0, 0, 0, 0, 1).finished();
  Eigen::Matrix<float,4,4> expCovMatrix = (Eigen::Matrix4f() << 5.0625, 0, 6.75, 0, 0, 5.0625, 0, 6.75, 6.75, 0, 9, 0, 0, 6.75, 0, 9).finished();

  DynamicsModels::LinearDynamics2D dynModel(0.1,2.0);

  ASSERT_TRUE(dynModel.TransMatrix(1.5).isApprox(expTransMatrix)) << "Expected trans matrix : \n" << expTransMatrix << ",\n got: \n" << dynModel.TransMatrix();
  ASSERT_TRUE(dynModel.CovMatrix(1.5).isApprox(expCovMatrix))<< "Expected cov matrix : \n" << expCovMatrix << ",\n got: \n" << dynModel.CovMatrix();
}

// TODO test void ProcNoise()


/*
// GAUSSIAN DATA TYPES test suite
*/

// Gaussian Model tests

TEST(GaussianDataSuite, gaussModelConsTest)
{
  // Test the GaussianModel constructor & clear method
  Eigen::Matrix<float, 4, 1> expMean = Eigen::MatrixXf::Zero(4,1);
  Eigen::Matrix<float, 4, 4> expCov = Eigen::MatrixXf::Identity(4,4);

  GaussianDataTypes::GaussianModel<4> gm;
  ASSERT_EQ(gm.Weight,0.0)<< "Expected gaussian weight : \n" << 0.0 << ",\n got: \n" << gm.Weight;
  ASSERT_EQ(gm.Mean, expMean)<< "Expected gaussian mean : \n" << expMean << ",\n got: \n" << gm.Mean;
  ASSERT_EQ(gm.Cov, expCov)<< "Expected gaussian covariance : \n" << expCov << ",\n got: \n" << gm.Cov;
}

TEST(GaussianDataSuite, gaussModelAsgnmntTest)
{
    // Test the GaussianModel assignment operator
    Eigen::Matrix<float, 4, 1> expMean;
    Eigen::Matrix<float, 4, 4> expCov;
    float expWeight{1.0};
    expMean << 2.0, 3.0, 4.0, 5.0;
    expCov << 6.0, 0.0, 0.0, 0.0,
              0.0, 7.0, 0.0, 0.0,
              0.0, 0.0, 8.0, 0.0,
              0.0, 0.0, 0.0, 9.0;

    // Form source gaussian model
    GaussianDataTypes::GaussianModel<4> srcGm;
    srcGm.Weight = expWeight;
    srcGm.Mean = expMean;
    srcGm.Cov = expCov;

    // Assign values to target gaussian model
    GaussianDataTypes::GaussianModel<4> targetGm;
    targetGm = srcGm;

    ASSERT_EQ(targetGm.Weight, expWeight)<< "Expected gaussian weight : \n" << expWeight << ",\n got: \n" << targetGm.Weight;
    ASSERT_EQ(targetGm.Mean, expMean)<< "Expected gaussian mean : \n" << expMean << ",\n got: \n" << targetGm.Mean;
    ASSERT_EQ(targetGm.Cov, expCov)<< "Expected gaussian covariance : \n" << expCov << ",\n got: \n" << targetGm.Cov;
}

// Gaussian Mixture tests

TEST(GaussianDataSuite, gaussMixConstTest1)
{
  // Test the default Gaussian Mixture constructor
  GaussianDataTypes::GaussianMixture<4> testGm;
  ASSERT_EQ(testGm.Gaussians.size(), 0)<< "Gaussian mixture has nonzero size after constructed.";

}

TEST(GaussianDataSuite, gaussMixConstTest2)
{
  // Test the second Gaussian Mixture constructor
    
  // Form gaussian mixture components
  GaussianDataTypes::GaussianModel<4> srcGm1, srcGm2, srcGm3, srcGm4, srcGm5;
  srcGm1.Weight = 0.1;
  srcGm1.Mean << 2.0, 3.0, 4.0, 5.0;
  srcGm1.Cov << 6.0, 0.0, 0.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 9.0;
  srcGm2 = srcGm1;
  srcGm2.Weight = 0.2;
  srcGm2.Mean *=1.2;
  srcGm2.Cov *=1.2;
  srcGm3 = srcGm1;
  srcGm3.Weight = 0.3;
  srcGm3.Mean *=1.3;
  srcGm3.Cov *=1.3;
  srcGm4 = srcGm1;
  srcGm4.Weight = 0.4;
  srcGm4.Mean *=1.4;
  srcGm4.Cov *=1.4;
  srcGm5 = srcGm1;
  srcGm5.Weight = 0.5;
  srcGm5.Mean *=1.5;
  srcGm5.Cov *=1.5;

  // Put gaussian models into a mixture
  GaussianDataTypes::GaussianMixture<4> gm1;
  gm1.Gaussians.push_back(srcGm1);
  gm1.Gaussians.push_back(srcGm2);
  gm1.Gaussians.push_back(srcGm3);
  gm1.Gaussians.push_back(srcGm4);
  gm1.Gaussians.push_back(srcGm5);

  // Form mixture by copy constructor 
  GaussianDataTypes::GaussianMixture<4> gm2(gm1);

  for (int ii ; ii < gm2.Gaussians.size(); ii++)
  {
    ASSERT_EQ(gm1.Gaussians[ii].Weight, gm2.Gaussians[ii].Weight)<< "Expected Gaussian weight : \n" << gm1.Gaussians[ii].Weight << ",\n copied Gaussian Mixture has: \n" << gm2.Gaussians[ii].Weight;
    ASSERT_EQ(gm1.Gaussians[ii].Mean, gm2.Gaussians[ii].Mean)<< "Expected Gaussian mean : \n" << gm1.Gaussians[ii].Mean << ",\n copied Gaussian Mixture has: \n" << gm2.Gaussians[ii].Mean;
    ASSERT_EQ(gm1.Gaussians[ii].Cov, gm2.Gaussians[ii].Cov)<< "Expected Gaussian covariance : \n" << gm1.Gaussians[ii].Cov << ",\n copied Gaussian Mixture has: \n" << gm2.Gaussians[ii].Cov;
  }

}

TEST(GaussianDataSuite, gaussMixConstTest3)
{
  // Test the second and third Gaussian Mixture constructors
    
  // Form gaussian mixture components
  GaussianDataTypes::GaussianModel<4> srcGm1, srcGm2, srcGm3, srcGm4, srcGm5;
  srcGm1.Weight = 0.1;
  srcGm1.Mean << 2.0, 3.0, 4.0, 5.0;
  srcGm1.Cov << 6.0, 0.0, 0.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 9.0;
  srcGm2 = srcGm1;
  srcGm2.Weight = 0.2;
  srcGm2.Mean *=1.2;
  srcGm2.Cov *=1.2;
  srcGm3 = srcGm1;
  srcGm3.Weight = 0.3;
  srcGm3.Mean *=1.3;
  srcGm3.Cov *=1.3;
  srcGm4 = srcGm1;
  srcGm4.Weight = 0.4;
  srcGm4.Mean *=1.4;
  srcGm4.Cov *=1.4;
  srcGm5 = srcGm1;
  srcGm5.Weight = 0.5;
  srcGm5.Mean *=1.5;
  srcGm5.Cov *=1.5;

  // Put gaussian models into a vector
  std::vector<GaussianDataTypes::GaussianModel<4>> gmVector;
  gmVector.push_back(srcGm1);
  gmVector.push_back(srcGm2);
  gmVector.push_back(srcGm3);
  gmVector.push_back(srcGm4);
  gmVector.push_back(srcGm5);

  // Put gaussian models into a mixture
  GaussianDataTypes::GaussianMixture<4> gm1;
  gm1.Gaussians.push_back(srcGm1);
  gm1.Gaussians.push_back(srcGm2);
  gm1.Gaussians.push_back(srcGm3);
  gm1.Gaussians.push_back(srcGm4);
  gm1.Gaussians.push_back(srcGm5);

  // Form mixture by copy constructor 
  GaussianDataTypes::GaussianMixture<4> gm2(gmVector);

  for (int ii ; ii < gm2.Gaussians.size(); ii++)
  {
    ASSERT_EQ(gm1.Gaussians[ii].Weight, gm2.Gaussians[ii].Weight)<< "Expected Gaussian weight : \n" << gm1.Gaussians[ii].Weight << ",\n constructed Gaussian Mixture has: \n" << gm2.Gaussians[ii].Weight;
    ASSERT_EQ(gm1.Gaussians[ii].Mean, gm2.Gaussians[ii].Mean)<< "Expected Gaussian mean : \n" << gm1.Gaussians[ii].Mean << ",\n constructed Gaussian Mixture has: \n" << gm2.Gaussians[ii].Mean;
    ASSERT_EQ(gm1.Gaussians[ii].Cov, gm2.Gaussians[ii].Cov)<< "Expected Gaussian covariance : \n" << gm1.Gaussians[ii].Cov << ",\n constructed Gaussian Mixture has: \n" << gm2.Gaussians[ii].Cov;
  }

}

TEST(GaussianDataSuite, gaussMixSortTest)
{
    // Test the GaussianMixture.sort() function by comparing an unsorted and sorted mixture, then sorting the unsorted mixture and comparing

    // Form gaussian mixture components
    GaussianDataTypes::GaussianModel<4> srcGm1, srcGm2, srcGm3, srcGm4, srcGm5;
    srcGm1.Weight = 0.1;
    srcGm1.Mean << 2.0, 3.0, 4.0, 5.0;
    srcGm1.Cov << 6.0, 0.0, 0.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 9.0;
    srcGm2 = srcGm1;
    srcGm2.Weight = 0.2;
    srcGm2.Mean *=1.2;
    srcGm2.Cov *=1.2;
    srcGm3 = srcGm1;
    srcGm3.Weight = 0.3;
    srcGm3.Mean *=1.3;
    srcGm3.Cov *=1.3;
    srcGm4 = srcGm1;
    srcGm4.Weight = 0.4;
    srcGm4.Mean *=1.4;
    srcGm4.Cov *=1.4;
    srcGm5 = srcGm1;
    srcGm5.Weight = 0.5;
    srcGm5.Mean *=1.5;
    srcGm5.Cov *=1.5;

    // Put gaussian models into a mixture, unsorted
    GaussianDataTypes::GaussianMixture<4> unsortedGm;
    unsortedGm.Gaussians.push_back(srcGm2);
    unsortedGm.Gaussians.push_back(srcGm5);
    unsortedGm.Gaussians.push_back(srcGm1);
    unsortedGm.Gaussians.push_back(srcGm4);
    unsortedGm.Gaussians.push_back(srcGm3);

    // Put gaussian models into a mixture, sorted
    GaussianDataTypes::GaussianMixture<4> sortedGm;
    sortedGm.Gaussians.push_back(srcGm5);
    sortedGm.Gaussians.push_back(srcGm4);
    sortedGm.Gaussians.push_back(srcGm3);
    sortedGm.Gaussians.push_back(srcGm2);
    sortedGm.Gaussians.push_back(srcGm1);

    for (int ii ; ii < unsortedGm.Gaussians.size(); ii++)
    {
      ASSERT_NE(unsortedGm.Gaussians[ii].Weight, sortedGm.Gaussians[ii].Weight)<< "Gaussian weight : \n" << sortedGm.Gaussians[ii].Weight << ",\n sorted Gaussian Mixture has: \n" << unsortedGm.Gaussians[ii].Weight;
      ASSERT_NE(unsortedGm.Gaussians[ii].Mean, sortedGm.Gaussians[ii].Mean)<< "Gaussian mean : \n" << sortedGm.Gaussians[ii].Mean << ",\n sorted Gaussian Mixture has: \n" << unsortedGm.Gaussians[ii].Mean;
      ASSERT_NE(unsortedGm.Gaussians[ii].Cov, sortedGm.Gaussians[ii].Cov)<< "Expected Gaussian covariance : \n" << sortedGm.Gaussians[ii].Cov << ",\n sorted Gaussian Mixture has: \n" << unsortedGm.Gaussians[ii].Cov;
    }

    unsortedGm.sort();

    for (int ii ; ii < unsortedGm.Gaussians.size(); ii++)
    {
      ASSERT_EQ(unsortedGm.Gaussians[ii].Weight, sortedGm.Gaussians[ii].Weight)<< "Expected Gaussian weight : \n" << sortedGm.Gaussians[ii].Weight << ",\n sorted Gaussian Mixture has: \n" << unsortedGm.Gaussians[ii].Weight;
      ASSERT_EQ(unsortedGm.Gaussians[ii].Mean, sortedGm.Gaussians[ii].Mean)<< "Expected Gaussian mean : \n" << sortedGm.Gaussians[ii].Mean << ",\n sorted Gaussian Mixture has: \n" << unsortedGm.Gaussians[ii].Mean;
      ASSERT_EQ(unsortedGm.Gaussians[ii].Cov, sortedGm.Gaussians[ii].Cov)<< "Expected Gaussian covariance : \n" << sortedGm.Gaussians[ii].Cov << ",\n sorted Gaussian Mixture has: \n" << unsortedGm.Gaussians[ii].Cov;
    }

}

TEST(GaussianDataSuite, gaussMixSelectBestTest)
{
    // Test the GaussianMixture.selectBestGaussian() function

    // Form gaussian mixture components
    GaussianDataTypes::GaussianModel<4> srcGm1, srcGm2, srcGm3, srcGm4, srcGm5;
    srcGm1.Weight = 0.1;
    srcGm1.Mean << 2.0, 3.0, 4.0, 5.0;
    srcGm1.Cov << 6.0, 0.0, 0.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 9.0;
    srcGm2 = srcGm1;
    srcGm2.Weight = 0.2;
    srcGm2.Mean *=1.2;
    srcGm2.Cov *=1.2;
    srcGm3 = srcGm1;
    srcGm3.Weight = 0.3;
    srcGm3.Mean *=1.3;
    srcGm3.Cov *=1.3;
    srcGm4 = srcGm1;
    srcGm4.Weight = 0.4;
    srcGm4.Mean *=1.4;
    srcGm4.Cov *=1.4;
    srcGm5 = srcGm1;
    srcGm5.Weight = 0.5;
    srcGm5.Mean *=1.5;
    srcGm5.Cov *=1.5;

    // Put gaussian models into a mixture, unsorted
    GaussianDataTypes::GaussianMixture<4> gaussMix1;
    gaussMix1.Gaussians.push_back(srcGm2);
    gaussMix1.Gaussians.push_back(srcGm5);
    gaussMix1.Gaussians.push_back(srcGm1);
    gaussMix1.Gaussians.push_back(srcGm4);
    gaussMix1.Gaussians.push_back(srcGm3);

    // Put gaussian models into a mixture, sorted
    GaussianDataTypes::GaussianMixture<4> gaussMix2;
    gaussMix2.Gaussians.push_back(srcGm5);
    gaussMix2.Gaussians.push_back(srcGm4);
    gaussMix2.Gaussians.push_back(srcGm3);
    gaussMix2.Gaussians.push_back(srcGm2);
    gaussMix2.Gaussians.push_back(srcGm1);

    GaussianDataTypes::GaussianMixture<4> gaussMixEmpty;

    ASSERT_EQ(gaussMix1.selectBestGaussian(), 1);
    ASSERT_EQ(gaussMix2.selectBestGaussian(), 0);
    ASSERT_EQ(gaussMixEmpty.selectBestGaussian(), -1);

}
// TODO Test select close gaussian function
// TODO Test merge function
// TODO Test prune function
// TODO Test mahalanobis distance function
// TODO Test normalize functions


// BIRTH test suite

// SPAWN test suite

// STATE predict test suite

// GMPHD test suite




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}