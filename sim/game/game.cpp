// Interactive Crazyflie simulator ("the game").
//
// Everything is simulated, no hardware: the drone, its sensors, the EKF,
// the MPC, the safety supervisor and the radio delay are exactly the code of
// sim::Simulation (the same code the batch simulator and the ROS nodes use).
// The player only moves the target point the MPC flies to, and can disturb
// the drone (wind gust, payload) or press the kill switch.
//
// Usage: cf_sim_game                 interactive window
//        cf_sim_game --demo <dir>    scripted run that saves screenshots, then exits
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>

#include <raylib.h>
#include <rlgl.h>

#include "cf_bringup/safety_supervisor.hpp"
#include "cf_bringup/scenario.hpp"
#include "cf_ekf/ekf.hpp"
#include "cf_model/params.hpp"
#include "draw.hpp"
#include "game_state.hpp"
#include "simulation.hpp"

namespace {

constexpr double kTargetSpeed = 0.6;       // m/s, horizontal target motion
constexpr double kTargetClimbRate = 0.4;   // m/s
constexpr double kTargetYawRate = 1.2;     // rad/s
constexpr double kTakeoffHeight = 0.5;     // m
constexpr double kFenceMargin = 0.15;      // m, target stays this far inside the geofence
constexpr double kGustForce = 0.04;        // N, about 14 % of the drone's weight
constexpr double kGustDuration = 1.5;      // s
constexpr double kPayloadMassFactor = 1.15;
constexpr double kRingPassRadius = 0.15;   // m
constexpr int kRingCount = 6;
constexpr double kHistoryLength = 10.0;    // s

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

struct World {
  cf_model::Params params;
  sim::SimConfig sim_config;
  cf_ekf::EkfConfig ekf_config;
  cf_bringup::SafetyConfig safety;
  std::unique_ptr<sim::Simulation> simulation;
  game::GameState state;
  std::mt19937 rng{7};
};

void show_message(game::GameState& state, const std::string& text, double wall_time) {
  state.message = text;
  state.message_until = wall_time + 2.5;
}

// Target is kept inside the geofence (minus a margin), above the floor.
void clamp_target(World& world) {
  Eigen::Vector3d& t = world.state.target;
  for (int i = 0; i < 2; ++i) {
    t(i) = std::clamp(t(i), world.safety.geofence_min(i) + kFenceMargin,
                      world.safety.geofence_max(i) - kFenceMargin);
  }
  t.z() = std::clamp(t.z(), 0.0, world.safety.geofence_max.z() - kFenceMargin);
}

// Creates a fresh simulation (drone on the floor at the origin, disarmed).
void reset_world(World& world) {
  world.simulation = std::make_unique<sim::Simulation>(
      world.params, world.sim_config, world.ekf_config, world.safety, Eigen::Vector3d::Zero());
  game::GameState& st = world.state;
  world.simulation->set_use_ekf(st.use_ekf);
  world.simulation->set_disturbance_observer(st.observer);
  world.simulation->drone().set_mass_factor(st.payload ? kPayloadMassFactor : 1.0);
  st.target = Eigen::Vector3d::Zero();
  st.target_yaw = 0.0;
  st.landing = false;
  st.gust_force.setZero();
  st.gust_end = -1.0;
  st.history.clear();
  st.mode = game::Mode::kManual;

  // The MPC asks for the reference over its whole horizon: give it the
  // mission waypoints (with preview) or the player's target.
  World* w = &world;
  world.simulation->set_reference([w](double t) {
    if (w->state.mode == game::Mode::kMission) {
      return cf_bringup::waypoint_at(w->state.mission, t - w->state.mission_start);
    }
    cf_bringup::Waypoint wp;
    wp.position = w->state.target;
    wp.yaw = w->state.target_yaw;
    return wp;
  });
  // One step so that last_sample() is filled before the first frame.
  world.simulation->step();
}

void arm(World& world, double wall_time) {
  sim::Simulation& sim = *world.simulation;
  game::GameState& st = world.state;
  const sim::TrueState& truth = sim.drone().state();
  if (truth.on_ground) {
    sim.reset_estimator();  // like the ekf/reset service before take-off
  }
  const cf_mpc::VehicleState& est = sim.last_sample().estimate;
  st.target = Eigen::Vector3d(est.position.x(), est.position.y(),
                              std::max(est.position.z(), kTakeoffHeight));
  st.target_yaw = est.yaw;
  st.landing = false;
  if (st.mode == game::Mode::kMission) {
    st.mission_start = sim.time();
  }
  clamp_target(world);
  sim.arm();
  show_message(st, "armed - taking off", wall_time);
}

void start_race(World& world, double wall_time) {
  game::GameState& st = world.state;
  std::uniform_real_distribution<double> ux(world.safety.geofence_min.x() + 0.3,
                                            world.safety.geofence_max.x() - 0.3);
  std::uniform_real_distribution<double> uy(world.safety.geofence_min.y() + 0.3,
                                            world.safety.geofence_max.y() - 0.3);
  std::uniform_real_distribution<double> uz(0.35, 1.1);
  st.rings.clear();
  Eigen::Vector3d previous = world.simulation->last_sample().true_position;
  for (int i = 0; i < kRingCount; ++i) {
    game::Ring ring;
    ring.center = Eigen::Vector3d(ux(world.rng), uy(world.rng), uz(world.rng));
    ring.heading = std::atan2(ring.center.y() - previous.y(), ring.center.x() - previous.x());
    st.rings.push_back(ring);
    previous = ring.center;
  }
  st.next_ring = 0;
  st.race_start = world.simulation->time();
  st.race_time = -1.0;
  st.mode = game::Mode::kRace;
  show_message(st, "ring race: steer the target through the yellow ring", wall_time);
}

// Direction the player calls "forward" (depends on the camera).
double control_heading(const game::GameState& st) {
  switch (st.camera) {
    case game::CameraMode::kChase: return st.chase_yaw;
    case game::CameraMode::kOrbit: return st.orbit_yaw + M_PI;
    case game::CameraMode::kTop: return 0.0;
  }
  return 0.0;
}

void handle_input(World& world, double frame_dt, double wall_time) {
  game::GameState& st = world.state;
  sim::Simulation& sim = *world.simulation;

  if (IsKeyPressed(KEY_SPACE)) {
    sim.kill();
    show_message(st, "KILL SWITCH", wall_time);
  }
  if (IsKeyPressed(KEY_ENTER)) arm(world, wall_time);
  if (IsKeyPressed(KEY_L) && sim.armed()) {
    st.landing = true;
    if (st.mode == game::Mode::kMission) st.mode = game::Mode::kManual;
    st.target = sim.last_sample().estimate.position;
    st.target.z() = 0.0;
    show_message(st, "landing", wall_time);
  }
  if (IsKeyPressed(KEY_ONE)) {
    st.mode = game::Mode::kManual;
    st.target = sim.last_sample().reference.position;
    show_message(st, "manual: move the target with W A S D, R F, Q E", wall_time);
  }
  if (IsKeyPressed(KEY_TWO)) {
    st.mode = game::Mode::kMission;
    st.mission_start = sim.time();
    if (!sim.armed()) arm(world, wall_time);
    show_message(st, "mission: flying sim/scenarios/steps.yaml", wall_time);
  }
  if (IsKeyPressed(KEY_THREE)) start_race(world, wall_time);
  if (IsKeyPressed(KEY_G)) {
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);
    const double a = angle(world.rng);
    st.gust_force = kGustForce * Eigen::Vector3d(std::cos(a), std::sin(a), 0.0);
    st.gust_end = sim.time() + kGustDuration;
    sim.drone().set_external_force(st.gust_force);
    show_message(st, "wind gust", wall_time);
  }
  if (IsKeyPressed(KEY_M)) {
    st.payload = !st.payload;
    sim.drone().set_mass_factor(st.payload ? kPayloadMassFactor : 1.0);
    show_message(st, st.payload ? "payload added: drone 15 % heavier" : "payload removed", wall_time);
  }
  if (IsKeyPressed(KEY_O)) {
    st.observer = !st.observer;
    sim.set_disturbance_observer(st.observer);
  }
  if (IsKeyPressed(KEY_T)) {
    st.use_ekf = !st.use_ekf;
    sim.set_use_ekf(st.use_ekf);
  }
  if (IsKeyPressed(KEY_C)) {
    st.camera = static_cast<game::CameraMode>((static_cast<int>(st.camera) + 1) % 3);
  }
  if (IsKeyPressed(KEY_P)) st.paused = !st.paused;
  if (IsKeyPressed(KEY_H)) st.show_help = !st.show_help;
  if (IsKeyPressed(KEY_X)) {
    reset_world(world);
    show_message(st, "reset", wall_time);
  }

  // Mouse: right drag orbits, wheel zooms.
  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    const Vector2 delta = GetMouseDelta();
    st.camera = game::CameraMode::kOrbit;
    st.orbit_yaw -= 0.005 * delta.x;
    st.orbit_pitch = std::clamp(st.orbit_pitch + 0.005 * delta.y, 0.05, 1.5);
  }
  st.orbit_distance = std::clamp(st.orbit_distance - 0.3 * GetMouseWheelMove(), 0.8, 8.0);

  // Target motion (manual and race modes).
  if (st.mode != game::Mode::kMission && !st.landing) {
    const double h = control_heading(st);
    const Eigen::Vector3d forward(std::cos(h), std::sin(h), 0.0);
    const Eigen::Vector3d left(-std::sin(h), std::cos(h), 0.0);
    Eigen::Vector3d move = Eigen::Vector3d::Zero();
    if (IsKeyDown(KEY_W)) move += forward;
    if (IsKeyDown(KEY_S)) move -= forward;
    if (IsKeyDown(KEY_A)) move += left;
    if (IsKeyDown(KEY_D)) move -= left;
    st.target += kTargetSpeed * frame_dt * move;
    if (IsKeyDown(KEY_R)) st.target.z() += kTargetClimbRate * frame_dt;
    if (IsKeyDown(KEY_F)) st.target.z() -= kTargetClimbRate * frame_dt;
    if (IsKeyDown(KEY_Q)) st.target_yaw = wrap(st.target_yaw + kTargetYawRate * frame_dt);
    if (IsKeyDown(KEY_E)) st.target_yaw = wrap(st.target_yaw - kTargetYawRate * frame_dt);
    clamp_target(world);
  }
}

// Things that follow from the simulation state: end of gust, landing,
// race progress, history for trails and plot, propeller animation.
void update_game(World& world, double frame_dt, double wall_time) {
  game::GameState& st = world.state;
  sim::Simulation& sim = *world.simulation;
  const sim::ControlSample& s = sim.last_sample();
  const sim::TrueState& truth = sim.drone().state();

  if (st.gust_end >= 0.0 && sim.time() > st.gust_end) {
    st.gust_force.setZero();
    st.gust_end = -1.0;
    sim.drone().set_external_force(Eigen::Vector3d::Zero());
  }
  if (st.landing && sim.armed() && truth.on_ground) {
    sim.disarm();
    st.landing = false;
    show_message(st, "landed - motors off", wall_time);
  }
  if (st.mode == game::Mode::kRace && st.race_time < 0.0 &&
      st.next_ring < static_cast<int>(st.rings.size())) {
    if ((truth.position - st.rings[st.next_ring].center).norm() < kRingPassRadius) {
      ++st.next_ring;
      if (st.next_ring == static_cast<int>(st.rings.size())) {
        st.race_time = sim.time() - st.race_start;
        if (st.best_race_time < 0.0 || st.race_time < st.best_race_time) {
          st.best_race_time = st.race_time;
        }
        show_message(st, TextFormat("race finished: %.1f s", st.race_time), wall_time);
      }
    }
  }

  // Chase camera follows the drone's heading, smoothed.
  st.chase_yaw = wrap(st.chase_yaw + std::min(1.0, 2.0 * frame_dt) * wrap(s.true_euler.z() - st.chase_yaw));

  // Propellers: spin speed grows with thrust (visual only), alternating direction.
  const double max_motor = cf_model::thrust_per_motor_from_cmd(world.params, world.params.thrust_cmd_max);
  for (int i = 0; i < 4; ++i) {
    const double load = std::clamp(truth.motor_thrust(i) / max_motor, 0.0, 1.0);
    const double direction = (i % 2 == 0) ? -1.0 : 1.0;
    st.prop_angle[i] += direction * 60.0 * std::sqrt(load) * frame_dt;
  }
}

void record_history(World& world) {
  const sim::ControlSample& s = world.simulation->last_sample();
  game::HistoryPoint point;
  point.t = s.t;
  point.true_position = s.true_position;
  point.est_position = s.estimate.position;
  point.ref_z = s.reference.position.z();
  world.state.history.push_back(point);
  while (!world.state.history.empty() && world.state.history.front().t < s.t - kHistoryLength) {
    world.state.history.pop_front();
  }
}

// Advances the simulation by frame_dt of simulated time (fixed 1 ms steps).
void advance_simulation(World& world, double frame_dt, double& accumulator) {
  accumulator += frame_dt;
  const double dt = world.simulation->physics_dt();
  while (accumulator >= dt) {
    if (world.simulation->step()) {
      record_history(world);
    }
    accumulator -= dt;
  }
}

// Scripted run for screenshots and as a smoke test: what a player would do.
struct DemoEvent {
  double time;      // s, simulation time
  const char* what;
};

void run_demo_event(World& world, const std::string& what, double wall_time) {
  game::GameState& st = world.state;
  if (what == "arm") arm(world, wall_time);
  if (what == "mission") {
    st.mode = game::Mode::kMission;
    st.mission_start = world.simulation->time();
  }
  if (what == "orbit") st.camera = game::CameraMode::kOrbit;
  if (what == "chase") st.camera = game::CameraMode::kChase;
  if (what == "top") st.camera = game::CameraMode::kTop;
  if (what == "gust") {
    st.gust_force = kGustForce * Eigen::Vector3d(0.0, -1.0, 0.0);
    st.gust_end = world.simulation->time() + kGustDuration;
    world.simulation->drone().set_external_force(st.gust_force);
    show_message(st, "wind gust", wall_time);
  }
  if (what == "race") {
    st.mode = game::Mode::kManual;
    st.target = world.simulation->last_sample().reference.position;
    start_race(world, wall_time);
  }
  if (what == "kill") {
    world.simulation->kill();
    show_message(st, "KILL SWITCH", wall_time);
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string demo_dir;
  if (argc == 3 && std::string(argv[1]) == "--demo") {
    demo_dir = argv[2];
  } else if (argc != 1) {
    std::fprintf(stderr, "usage: cf_sim_game [--demo <screenshot dir>]\n");
    return 1;
  }
  const bool demo = !demo_dir.empty();

  World world;
  world.params = cf_model::load_params(CF_PARAMS_FILE);
  world.sim_config = sim::load_sim_config(CF_SIM_CONFIG);
  world.ekf_config = cf_ekf::load_ekf_config(CF_EKF_CONFIG);
  world.safety = cf_bringup::load_safety_config(CF_SAFETY_CONFIG);
  world.state.mission = cf_bringup::load_scenario(CF_MISSION_FILE);
  reset_world(world);

  SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
  InitWindow(1280, 720, "Crazyflie MPC + EKF simulator");
  SetTargetFPS(60);
  game::SceneAssets assets = game::load_assets();
  show_message(world.state, "press ENTER to take off", 0.0);

  // Demo script (simulation time).
  const DemoEvent demo_events[] = {
      {0.5, "arm"}, {3.0, "mission"}, {8.0, "orbit"}, {14.0, "gust"},
      {14.0, "chase"}, {17.0, "race"}, {17.0, "top"}, {24.0, "chase"}, {24.5, "kill"},
  };
  const DemoEvent demo_shots[] = {
      {2.5, "game_takeoff.png"}, {12.5, "game_mission_orbit.png"}, {14.6, "game_gust.png"},
      {22.0, "game_race_top.png"}, {25.8, "game_kill_switch.png"},
  };
  size_t next_event = 0;
  size_t next_shot = 0;
  const double demo_end = 26.0;

  double accumulator = 0.0;
  double wall_time = 0.0;
  while (!WindowShouldClose()) {
    // Real time when playing; a fixed 1/60 s per frame in the demo (reproducible).
    const double frame_dt = demo ? 1.0 / 60.0 : std::min<double>(GetFrameTime(), 0.05);
    wall_time += frame_dt;
    sim::Simulation& sim = *world.simulation;

    if (demo) {
      while (next_event < std::size(demo_events) && sim.time() >= demo_events[next_event].time) {
        run_demo_event(world, demo_events[next_event].what, wall_time);
        ++next_event;
      }
      // Demo "pilot" for the race: put the target on the next ring.
      game::GameState& st = world.state;
      if (st.mode == game::Mode::kRace && st.next_ring < static_cast<int>(st.rings.size())) {
        const Eigen::Vector3d to_ring = st.rings[st.next_ring].center - st.target;
        const double step = kTargetSpeed * 1.5 * frame_dt;
        st.target += to_ring.norm() > step ? (step * to_ring.normalized()).eval() : to_ring;
      }
    } else {
      handle_input(world, frame_dt, wall_time);
    }

    if (!world.state.paused) {
      advance_simulation(world, frame_dt, accumulator);
      update_game(world, frame_dt, wall_time);
    }

    BeginDrawing();
    game::draw_scene(assets, world.state, *world.simulation, world.params, world.safety);
    game::draw_hud(world.state, *world.simulation, world.params, wall_time);
    if (demo && next_shot < std::size(demo_shots) &&
        world.simulation->time() >= demo_shots[next_shot].time) {
      rlDrawRenderBatchActive();  // flush pending draws so the HUD is in the image
      Image shot = LoadImageFromScreen();
      const std::string path = demo_dir + "/" + demo_shots[next_shot].what;
      ExportImage(shot, path.c_str());
      UnloadImage(shot);
      ++next_shot;
    }
    EndDrawing();

    if (demo && world.simulation->time() >= demo_end) break;
  }

  game::unload_assets(assets);
  CloseWindow();
  return 0;
}
