// Closed-loop simulation: simulated Crazyflie + EKF + MPC + safety supervisor.
//
// Usage: closed_loop_sim <scenario.yaml> <output.csv> [sim.yaml]
//
// Timing: physics at physics_rate; IMU, flow and range at their own rates
// drive the EKF; MPC and supervisor run at control_rate; their command
// reaches the drone after command_latency (radio).
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "cf_bringup/safety_supervisor.hpp"
#include "cf_bringup/scenario.hpp"
#include "cf_ekf/ekf.hpp"
#include "cf_model/params.hpp"
#include "cf_mpc/mpc_controller.hpp"
#include "plant.hpp"

namespace {

// Number of physics steps between two events at the given rate.
int steps_per_event(double physics_rate, double rate) {
  const double ratio = physics_rate / rate;
  const int steps = static_cast<int>(std::lround(ratio));
  if (std::abs(ratio - steps) > 1e-9) {
    throw std::runtime_error("all rates must divide physics_rate");
  }
  return steps;
}

struct PendingSetpoint {
  double release_time = 0.0;
  sim::Setpoint setpoint;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: closed_loop_sim <scenario.yaml> <output.csv> [sim.yaml]\n";
    return 1;
  }
  const std::string scenario_file = argv[1];
  const std::string output_file = argv[2];
  const std::string sim_file = argc > 3 ? argv[3] : CF_SIM_CONFIG;

  const cf_model::Params params = cf_model::load_params(CF_PARAMS_FILE);
  const cf_bringup::Scenario scenario = cf_bringup::load_scenario(scenario_file);
  sim::SimConfig sim_config = sim::load_sim_config(sim_file);
  if (scenario.mass_factor > 0.0) sim_config.mass_factor = scenario.mass_factor;
  if (scenario.thrust_factor > 0.0) sim_config.thrust_factor = scenario.thrust_factor;
  const cf_ekf::EkfConfig ekf_config = cf_ekf::load_ekf_config(CF_EKF_CONFIG);
  const cf_bringup::SafetyConfig safety_config = cf_bringup::load_safety_config(CF_SAFETY_CONFIG);

  sim::Quadrotor quad(params, sim_config);
  quad.set_position(scenario.initial_position);

  cf_ekf::Ekf ekf(params, ekf_config);
  cf_ekf::StateVector x0 = cf_ekf::StateVector::Zero();
  x0.segment<3>(cf_ekf::PX) = scenario.initial_position;
  ekf.reset(x0);

  cf_mpc::MpcController mpc(params);
  mpc.set_disturbance_observer(scenario.disturbance_observer);
  cf_bringup::SafetySupervisor supervisor(safety_config);

  if (std::abs(1.0 / sim_config.control_rate - mpc.sample_time()) > 1e-9) {
    throw std::runtime_error("control_rate does not match the MPC sample time");
  }

  const double dt = 1.0 / sim_config.physics_rate;
  const int imu_every = steps_per_event(sim_config.physics_rate, sim_config.imu_rate);
  const int flow_every = steps_per_event(sim_config.physics_rate, sim_config.flow_rate);
  const int range_every = steps_per_event(sim_config.physics_rate, sim_config.range_rate);
  const int control_every = steps_per_event(sim_config.physics_rate, sim_config.control_rate);
  const int total_steps = static_cast<int>(std::lround(scenario.duration / dt));
  const int horizon = mpc.horizon_steps();

  std::ofstream csv(output_file);
  if (!csv) {
    throw std::runtime_error("cannot open " + output_file);
  }
  csv << "t,true_px,true_py,true_pz,true_vx,true_vy,true_vz,true_roll,true_pitch,true_yaw,"
         "est_px,est_py,est_pz,est_vx,est_vy,est_vz,est_roll,est_pitch,est_yaw,"
         "ref_px,ref_py,ref_pz,ref_yaw,"
         "cmd_roll,cmd_pitch,cmd_yaw_rate,cmd_thrust_cmd,"
         "stopped,solver_ok,solve_time\n";

  std::deque<PendingSetpoint> radio;
  Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();
  double max_solve_time = 0.0;
  std::string stop_reason;

  for (int step = 0; step <= total_steps; ++step) {
    const double t = step * dt;

    // ---- sensors -> EKF ----
    if (step % imu_every == 0) {
      cf_ekf::ImuSample imu;
      imu.gyro = quad.measure_gyro();
      imu.accel = quad.measure_accel();
      last_gyro = imu.gyro;
      ekf.predict(imu, imu_every * dt);
    }
    if (step % range_every == 0) {
      ekf.update_range(quad.measure_range());
    }
    if (step % flow_every == 0) {
      cf_ekf::FlowSample flow;
      flow.dt = flow_every * dt;
      const Eigen::Vector2d pixels = quad.measure_flow(flow.dt);
      flow.dpixel_x = pixels.x();
      flow.dpixel_y = pixels.y();
      flow.gyro = last_gyro;
      ekf.update_flow(flow);
    }

    // ---- MPC + supervisor ----
    if (step % control_every == 0) {
      const sim::TrueState& truth = quad.state();
      const Eigen::Vector3d true_euler = cf_model::euler_zyx(truth.attitude.toRotationMatrix());

      cf_mpc::VehicleState estimate;
      if (scenario.use_ekf) {
        const cf_ekf::StateVector& x = ekf.state();
        estimate.position = x.segment<3>(cf_ekf::PX);
        estimate.velocity = x.segment<3>(cf_ekf::VX);
        estimate.roll = x(cf_ekf::ROLL);
        estimate.pitch = x(cf_ekf::PITCH);
        estimate.yaw = x(cf_ekf::YAW);
      } else {
        estimate.position = truth.position;
        estimate.velocity = truth.velocity;
        estimate.roll = true_euler.x();
        estimate.pitch = true_euler.y();
        estimate.yaw = true_euler.z();
      }

      cf_mpc::Reference reference;
      for (int k = 0; k <= horizon; ++k) {
        const cf_bringup::Waypoint& wp = cf_bringup::waypoint_at(scenario, t + k * mpc.sample_time());
        reference.positions.push_back(wp.position);
        reference.velocities.push_back(Eigen::Vector3d::Zero());
      }
      const cf_bringup::Waypoint& current_wp = cf_bringup::waypoint_at(scenario, t);
      reference.yaw = current_wp.yaw;

      const cf_mpc::MpcOutput mpc_out = mpc.compute(estimate, reference);
      max_solve_time = std::max(max_solve_time, mpc_out.solve_time);

      cf_bringup::SupervisorInput sup_in;
      sup_in.now = t;
      sup_in.position = estimate.position;
      sup_in.estimate_stamp = t;
      sup_in.command.roll = mpc_out.command.roll;
      sup_in.command.pitch = mpc_out.command.pitch;
      sup_in.command.yaw_rate = mpc_out.command.yaw_rate;
      sup_in.command.thrust_cmd = mpc_out.command.thrust_cmd;
      sup_in.command_stamp = t;
      sup_in.solver_ok = mpc_out.ok;
      sup_in.kill_switch = scenario.kill_time >= 0.0 && t >= scenario.kill_time;
      const cf_bringup::SupervisorOutput sup_out = supervisor.step(sup_in);
      if (stop_reason.empty() && !sup_out.stop_reason.empty()) {
        stop_reason = sup_out.stop_reason;
        std::printf("t = %.2f s: STOP (%s)\n", t, stop_reason.c_str());
      }

      PendingSetpoint pending;
      pending.release_time = t + sim_config.command_latency;
      pending.setpoint.roll = sup_out.command.roll;
      pending.setpoint.pitch = sup_out.command.pitch;
      pending.setpoint.yaw_rate = sup_out.command.yaw_rate;
      pending.setpoint.thrust_cmd = sup_out.command.thrust_cmd;
      radio.push_back(pending);

      csv << t << ','
          << truth.position.x() << ',' << truth.position.y() << ',' << truth.position.z() << ','
          << truth.velocity.x() << ',' << truth.velocity.y() << ',' << truth.velocity.z() << ','
          << true_euler.x() << ',' << true_euler.y() << ',' << true_euler.z() << ','
          << estimate.position.x() << ',' << estimate.position.y() << ',' << estimate.position.z() << ','
          << estimate.velocity.x() << ',' << estimate.velocity.y() << ',' << estimate.velocity.z() << ','
          << estimate.roll << ',' << estimate.pitch << ',' << estimate.yaw << ','
          << current_wp.position.x() << ',' << current_wp.position.y() << ','
          << current_wp.position.z() << ',' << current_wp.yaw << ','
          << sup_out.command.roll << ',' << sup_out.command.pitch << ','
          << sup_out.command.yaw_rate << ',' << sup_out.command.thrust_cmd << ','
          << (sup_out.mode == cf_bringup::Mode::kStopped ? 1 : 0) << ','
          << (mpc_out.ok ? 1 : 0) << ',' << mpc_out.solve_time << '\n';
    }

    // ---- radio -> drone ----
    while (!radio.empty() && radio.front().release_time <= t + 1e-12) {
      quad.set_setpoint(radio.front().setpoint);
      radio.pop_front();
    }

    quad.step(dt);
  }

  std::printf("wrote %s (max MPC solve time %.3f ms)\n", output_file.c_str(),
              1000.0 * max_solve_time);
  return 0;
}
