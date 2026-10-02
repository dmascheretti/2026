#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "cf_ekf/ekf.hpp"

namespace {

const std::string kParamsFile = CF_PARAMS_FILE;      // set by CMake
const std::string kEkfConfigFile = CF_EKF_CONFIG;    // set by CMake

class EkfTest : public ::testing::Test {
 protected:
  cf_model::Params params_ = cf_model::load_params(kParamsFile);
  cf_ekf::EkfConfig config_ = cf_ekf::load_ekf_config(kEkfConfigFile);
  cf_ekf::Ekf ekf_{params_, config_};

  cf_ekf::ImuSample hover_imu() {
    cf_ekf::ImuSample imu;
    imu.accel = Eigen::Vector3d(0.0, 0.0, params_.gravity);
    return imu;
  }
};

TEST_F(EkfTest, HoverImuKeepsStateConstant) {
  cf_ekf::StateVector x0 = cf_ekf::StateVector::Zero();
  x0(cf_ekf::PZ) = 1.0;
  const cf_ekf::StateVector x1 = ekf_.propagate(x0, hover_imu(), 0.01);
  EXPECT_LT((x1 - x0).norm(), 1e-12);
}

TEST_F(EkfTest, ZeroSpecificForceIsFreeFall) {
  const cf_ekf::StateVector x0 = cf_ekf::StateVector::Zero();
  const cf_ekf::StateVector x1 = ekf_.propagate(x0, cf_ekf::ImuSample{}, 0.1);
  EXPECT_NEAR(x1(cf_ekf::VZ), -params_.gravity * 0.1, 1e-12);
  EXPECT_NEAR(x1(cf_ekf::PZ), -0.5 * params_.gravity * 0.01, 1e-12);
}

TEST_F(EkfTest, GyroRollRateIntegrates) {
  cf_ekf::ImuSample imu = hover_imu();
  imu.gyro = Eigen::Vector3d(0.5, 0.0, 0.0);
  const cf_ekf::StateVector x1 = ekf_.propagate(cf_ekf::StateVector::Zero(), imu, 0.01);
  EXPECT_NEAR(x1(cf_ekf::ROLL), 0.005, 1e-12);
}

TEST_F(EkfTest, PropagateDerivativesMatchHandDerivedAtHover) {
  // At hover: d(vx)/d(pitch) = g*dt, d(vy)/d(roll) = -g*dt.
  // Checked by finite differences of propagate().
  const double dt = 0.01;
  const double h = 1e-6;
  cf_ekf::StateVector x = cf_ekf::StateVector::Zero();
  cf_ekf::StateVector xp = x;
  xp(cf_ekf::PITCH) = h;
  const double dvx_dpitch =
      (ekf_.propagate(xp, hover_imu(), dt)(cf_ekf::VX) - ekf_.propagate(x, hover_imu(), dt)(cf_ekf::VX)) / h;
  EXPECT_NEAR(dvx_dpitch, params_.gravity * dt, 1e-6);
  xp = x;
  xp(cf_ekf::ROLL) = h;
  const double dvy_droll =
      (ekf_.propagate(xp, hover_imu(), dt)(cf_ekf::VY) - ekf_.propagate(x, hover_imu(), dt)(cf_ekf::VY)) / h;
  EXPECT_NEAR(dvy_droll, -params_.gravity * dt, 1e-6);
}

TEST_F(EkfTest, RangeModelAccountsForTilt) {
  cf_ekf::StateVector x = cf_ekf::StateVector::Zero();
  x(cf_ekf::PZ) = 1.0;
  EXPECT_NEAR(ekf_.predict_range(x), 1.0, 1e-12);
  x(cf_ekf::ROLL) = 0.2;
  EXPECT_NEAR(ekf_.predict_range(x), 1.0 / std::cos(0.2), 1e-12);
}

TEST_F(EkfTest, FlowModelForwardMotion) {
  cf_ekf::StateVector x = cf_ekf::StateVector::Zero();
  x(cf_ekf::PZ) = 1.0;
  x(cf_ekf::VX) = 1.0;
  cf_ekf::FlowSample flow;
  flow.dt = 0.01;
  const Eigen::Vector2d n = ekf_.predict_flow(x, flow);
  EXPECT_NEAR(n.x(), 0.01 * params_.flow_npix / params_.flow_thetapix, 1e-12);
  EXPECT_NEAR(n.y(), 0.0, 1e-12);
}

TEST_F(EkfTest, OutlierRangeIsRejected) {
  cf_ekf::StateVector x0 = cf_ekf::StateVector::Zero();
  x0(cf_ekf::PZ) = 1.0;
  ekf_.reset(x0);
  EXPECT_TRUE(ekf_.update_range(1.01).accepted);
  EXPECT_FALSE(ekf_.update_range(3.0).accepted);
  EXPECT_FALSE(ekf_.update_range(10.0).accepted);  // above range_max
}

// Drone drifts at constant velocity at 1 m height. The filter starts with
// the wrong height and zero velocity and must converge.
void run_constant_velocity_flight(cf_ekf::Ekf& ekf, const cf_model::Params& params,
                                  double noise_scale, double& rms_velocity_error,
                                  double& final_height);

TEST_F(EkfTest, ConvergesExactlyWithoutNoise) {
  double rms_velocity_error = 0.0;
  double final_height = 0.0;
  run_constant_velocity_flight(ekf_, params_, 0.0, rms_velocity_error, final_height);
  EXPECT_LT(rms_velocity_error, 0.01);
  EXPECT_NEAR(final_height, 1.0, 1e-3);
}

TEST_F(EkfTest, ConvergesWithNoisySensors) {
  double rms_velocity_error = 0.0;
  double final_height = 0.0;
  run_constant_velocity_flight(ekf_, params_, 1.0, rms_velocity_error, final_height);
  EXPECT_LT(rms_velocity_error, 0.1);
  EXPECT_NEAR(final_height, 1.0, 0.02);

  // Covariance stays symmetric positive definite.
  const cf_ekf::StateMatrix& P = ekf_.covariance();
  EXPECT_LT((P - P.transpose()).norm(), 1e-12);
  Eigen::SelfAdjointEigenSolver<cf_ekf::StateMatrix> eig(P);
  EXPECT_GT(eig.eigenvalues().minCoeff(), 0.0);
}

// noise_scale = 1 gives: accel 0.05 m/s^2, gyro 0.002 rad/s, range 5 mm,
// flow 0.3 pixel per sample. RMS velocity error is over the last 5 s.
void run_constant_velocity_flight(cf_ekf::Ekf& ekf, const cf_model::Params& params,
                                  double noise_scale, double& rms_velocity_error,
                                  double& final_height) {
  std::mt19937 rng(42);
  std::normal_distribution<double> unit(0.0, 1.0);

  const Eigen::Vector3d true_velocity(0.3, -0.2, 0.0);
  const double true_height = 1.0;

  cf_ekf::StateVector x0 = cf_ekf::StateVector::Zero();
  x0(cf_ekf::PZ) = 0.7;  // wrong on purpose
  ekf.reset(x0);

  double sum_sq_error = 0.0;
  int error_samples = 0;
  const double imu_dt = 0.002;     // 500 Hz
  const int steps_per_flow = 5;    // 100 Hz
  const int steps_per_range = 12;  // ~40 Hz
  for (int step = 0; step < 5000; ++step) {  // 10 s
    cf_ekf::ImuSample imu;
    imu.accel = Eigen::Vector3d(0.0, 0.0, params.gravity);
    for (int i = 0; i < 3; ++i) {
      imu.accel(i) += noise_scale * 0.05 * unit(rng);
      imu.gyro(i) += noise_scale * 0.002 * unit(rng);
    }
    ekf.predict(imu, imu_dt);

    if (step % steps_per_range == 0) {
      ekf.update_range(true_height + noise_scale * 0.005 * unit(rng));
    }
    if (step % steps_per_flow == 0) {
      cf_ekf::FlowSample flow;
      flow.dt = imu_dt * steps_per_flow;
      cf_ekf::StateVector truth = cf_ekf::StateVector::Zero();
      truth(cf_ekf::PZ) = true_height;
      truth.segment<3>(cf_ekf::VX) = true_velocity;
      const Eigen::Vector2d n = ekf.predict_flow(truth, flow);
      flow.dpixel_x = n.x() + noise_scale * 0.3 * unit(rng);
      flow.dpixel_y = n.y() + noise_scale * 0.3 * unit(rng);
      ekf.update_flow(flow);
    }
    if (step >= 2500) {
      sum_sq_error += (ekf.state().segment<3>(cf_ekf::VX) - true_velocity).squaredNorm();
      ++error_samples;
    }
  }
  rms_velocity_error = std::sqrt(sum_sq_error / error_samples);
  final_height = ekf.state()(cf_ekf::PZ);
}

}  // namespace
