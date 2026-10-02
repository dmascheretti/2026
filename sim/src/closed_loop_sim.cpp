// Batch closed-loop simulation of one scenario, written to a CSV log.
//
// Usage: closed_loop_sim <scenario.yaml> <output.csv> [sim.yaml]
//
// The system itself (drone, sensors, EKF, MPC, supervisor, radio latency)
// is sim::Simulation; this file only loads the scenario, runs it and logs.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "cf_bringup/safety_supervisor.hpp"
#include "cf_bringup/scenario.hpp"
#include "cf_ekf/ekf.hpp"
#include "cf_model/params.hpp"
#include "simulation.hpp"

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

  sim::Simulation simulation(params, sim_config, cf_ekf::load_ekf_config(CF_EKF_CONFIG),
                             cf_bringup::load_safety_config(CF_SAFETY_CONFIG),
                             scenario.initial_position);
  simulation.set_use_ekf(scenario.use_ekf);
  simulation.set_disturbance_observer(scenario.disturbance_observer);
  simulation.set_reference([&scenario](double t) { return cf_bringup::waypoint_at(scenario, t); });
  simulation.arm();  // armed from t = 0

  std::ofstream csv(output_file);
  if (!csv) {
    throw std::runtime_error("cannot open " + output_file);
  }
  csv << "t,true_px,true_py,true_pz,true_vx,true_vy,true_vz,true_roll,true_pitch,true_yaw,"
         "est_px,est_py,est_pz,est_vx,est_vy,est_vz,est_roll,est_pitch,est_yaw,"
         "ref_px,ref_py,ref_pz,ref_yaw,"
         "cmd_roll,cmd_pitch,cmd_yaw_rate,cmd_thrust_cmd,"
         "stopped,solver_ok,solve_time\n";

  const long total_steps = std::lround(scenario.duration / simulation.physics_dt());
  double max_solve_time = 0.0;
  std::string stop_reason;

  for (long step = 0; step <= total_steps; ++step) {
    if (scenario.kill_time >= 0.0 && simulation.time() >= scenario.kill_time) {
      simulation.kill();
    }
    if (!simulation.step()) {
      continue;  // no control step this time: nothing to log
    }
    const sim::ControlSample& s = simulation.last_sample();
    max_solve_time = std::max(max_solve_time, s.solve_time);
    if (stop_reason.empty() && !s.stop_reason.empty()) {
      stop_reason = s.stop_reason;
      std::printf("t = %.2f s: STOP (%s)\n", s.t, stop_reason.c_str());
    }
    csv << s.t << ','
        << s.true_position.x() << ',' << s.true_position.y() << ',' << s.true_position.z() << ','
        << s.true_velocity.x() << ',' << s.true_velocity.y() << ',' << s.true_velocity.z() << ','
        << s.true_euler.x() << ',' << s.true_euler.y() << ',' << s.true_euler.z() << ','
        << s.estimate.position.x() << ',' << s.estimate.position.y() << ','
        << s.estimate.position.z() << ',' << s.estimate.velocity.x() << ','
        << s.estimate.velocity.y() << ',' << s.estimate.velocity.z() << ','
        << s.estimate.roll << ',' << s.estimate.pitch << ',' << s.estimate.yaw << ','
        << s.reference.position.x() << ',' << s.reference.position.y() << ','
        << s.reference.position.z() << ',' << s.reference.yaw << ','
        << s.sent.roll << ',' << s.sent.pitch << ',' << s.sent.yaw_rate << ','
        << s.sent.thrust_cmd << ',' << (s.stopped ? 1 : 0) << ',' << (s.solver_ok ? 1 : 0)
        << ',' << s.solve_time << '\n';
  }

  std::printf("wrote %s (max MPC solve time %.3f ms)\n", output_file.c_str(),
              1000.0 * max_solve_time);
  return 0;
}
