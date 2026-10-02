// State of the interactive simulator ("game") that is not part of the
// simulated system itself: what the player controls and what is shown.
#pragma once

#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "cf_bringup/scenario.hpp"

namespace game {

enum class Mode { kManual, kMission, kRace };
enum class CameraMode { kChase, kOrbit, kTop };

// One point of the history shown in trails and the altitude plot (50 Hz).
struct HistoryPoint {
  double t = 0.0;
  Eigen::Vector3d true_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d est_position = Eigen::Vector3d::Zero();
  double ref_z = 0.0;
};

struct Ring {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // m, world
  double heading = 0.0;                              // rad, direction to fly through
};

struct GameState {
  Mode mode = Mode::kManual;
  CameraMode camera = CameraMode::kChase;
  bool paused = false;
  bool show_help = true;
  bool landing = false;

  // Player-controlled target (manual and race modes).
  Eigen::Vector3d target = Eigen::Vector3d(0.0, 0.0, 0.0);  // m
  double target_yaw = 0.0;                                  // rad

  // Mission mode: scenario played from mission_start.
  cf_bringup::Scenario mission;
  double mission_start = 0.0;  // s, simulation time

  // Toggles shown in the HUD.
  bool use_ekf = true;
  bool observer = true;
  bool payload = false;          // +15 % mass

  // Wind gust.
  Eigen::Vector3d gust_force = Eigen::Vector3d::Zero();  // N, world
  double gust_end = -1.0;                                // s, simulation time

  // Race.
  std::vector<Ring> rings;
  int next_ring = 0;
  double race_start = 0.0;
  double race_time = -1.0;       // s, < 0 while running
  double best_race_time = -1.0;  // s

  // Camera (orbit mode angles in rad, distance in m).
  double orbit_yaw = -2.4;
  double orbit_pitch = 0.45;
  double orbit_distance = 2.4;
  double chase_yaw = 0.0;        // smoothed drone yaw for the chase camera

  // Visuals.
  std::deque<HistoryPoint> history;  // last 10 s
  double prop_angle[4] = {0.0, 0.0, 0.0, 0.0};
  std::string message;               // short message at the top of the screen
  double message_until = 0.0;        // s, wall time
};

}  // namespace game
