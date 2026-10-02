#include "draw.hpp"

#include <algorithm>
#include <cmath>

#include <rlgl.h>

namespace game {

namespace {

// Dark theme, same palette as the analysis plots.
const Color kBackground = {26, 26, 25, 255};
const Color kPanel = {20, 20, 19, 215};
const Color kText = {255, 255, 255, 255};
const Color kTextMuted = {195, 194, 183, 255};
const Color kTrue = {57, 135, 229, 255};       // blue: true state
const Color kEstimate = {217, 89, 38, 255};    // orange: EKF estimate
const Color kCommand = {25, 158, 112, 255};    // aqua: commands, reference
const Color kDanger = {230, 103, 103, 255};    // red: stop, geofence
const Color kWarning = {201, 133, 0, 255};     // yellow: warnings

// The real Crazyflie is 9 cm across: drawn this many times larger.
constexpr double kDroneVisualScale = 3.0;

Color with_alpha(Color c, unsigned char alpha) {
  c.a = alpha;
  return c;
}

// Rotation body -> world, in raylib coordinates: P * R * P^T.
Eigen::Matrix3d to_raylib_rotation(const Eigen::Matrix3d& R) {
  Eigen::Matrix3d P;
  P << 1, 0, 0,
       0, 0, 1,
       0, -1, 0;
  return P * R * P.transpose();
}

// Pushes a model transform so that the following draw calls use body
// coordinates (already mapped with to_raylib).
void push_body_transform(const Eigen::Vector3d& position, const Eigen::Matrix3d& R) {
  const Eigen::Matrix3d Rr = to_raylib_rotation(R);
  const Vector3 p = to_raylib(position);
  // 4x4 homogeneous transform, column-major as OpenGL expects.
  const float m[16] = {
      (float)Rr(0, 0), (float)Rr(1, 0), (float)Rr(2, 0), 0.0f,   // column 0
      (float)Rr(0, 1), (float)Rr(1, 1), (float)Rr(2, 1), 0.0f,   // column 1
      (float)Rr(0, 2), (float)Rr(1, 2), (float)Rr(2, 2), 0.0f,   // column 2
      p.x, p.y, p.z, 1.0f};                                      // translation
  rlPushMatrix();
  rlMultMatrixf(m);
}

// Motor positions in the body frame (same order as the simulator's mixer:
// front-right, back-right, back-left, front-left).
Eigen::Vector3d motor_position(const cf_model::Params& params, int i) {
  const double d = params.motor_to_motor_diagonal / 2.0 / std::sqrt(2.0);
  const double x[4] = {d, -d, -d, d};
  const double y[4] = {-d, -d, d, d};
  return Eigen::Vector3d(x[i], y[i], 0.0) * kDroneVisualScale;
}

void draw_drone_solid(const cf_model::Params& params, const double prop_angle[4],
                      const Eigen::Vector4d& motor_thrust, double max_motor_thrust) {
  const double s = kDroneVisualScale;
  // Body.
  DrawCube(Vector3{0, 0, 0}, 0.03f * s, 0.008f * s, 0.03f * s, Color{60, 60, 62, 255});
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d motor = motor_position(params, i);
    const bool front = motor.x() > 0.0;
    DrawCylinderEx(Vector3{0, 0, 0}, to_raylib(motor), 0.0025f * s, 0.0025f * s, 6,
                   front ? kDanger : Color{90, 90, 92, 255});
    // Motor.
    const Vector3 m = to_raylib(motor);
    DrawCylinder(Vector3{m.x, m.y - 0.004f * (float)s, m.z}, 0.004f * s, 0.004f * s,
                 0.009f * s, 8, Color{40, 40, 40, 255});
    // Propeller disc (brighter = more thrust) and two blades at the spin angle.
    const double load = std::clamp(motor_thrust(i) / max_motor_thrust, 0.0, 1.0);
    const Vector3 hub = {m.x, m.y + 0.0055f * (float)s, m.z};
    DrawCylinder(hub, 0.0225f * s, 0.0225f * s, 0.0005f * s, 24,
                 with_alpha(kTrue, static_cast<unsigned char>(30 + 90 * load)));
    for (int blade = 0; blade < 2; ++blade) {
      const double a = prop_angle[i] + blade * M_PI;
      const Vector3 tip = {hub.x + (float)(0.022 * s * std::cos(a)), hub.y,
                           hub.z + (float)(0.022 * s * std::sin(a))};
      DrawCylinderEx(hub, tip, 0.0012f * s, 0.0008f * s, 4, Color{220, 220, 220, 255});
    }
  }
}

void draw_drone_wire(const cf_model::Params& params, Color color) {
  const double s = kDroneVisualScale;
  DrawCubeWires(Vector3{0, 0, 0}, 0.03f * s, 0.008f * s, 0.03f * s, color);
  for (int i = 0; i < 4; ++i) {
    const Vector3 m = to_raylib(motor_position(params, i));
    DrawLine3D(Vector3{0, 0, 0}, m, color);
    DrawCircle3D(m, 0.0225f * s, Vector3{1, 0, 0}, 90.0f, color);
  }
}

void draw_trail(const std::deque<HistoryPoint>& history, bool estimate, Color color) {
  for (size_t i = 1; i < history.size(); ++i) {
    const Eigen::Vector3d& a = estimate ? history[i - 1].est_position : history[i - 1].true_position;
    const Eigen::Vector3d& b = estimate ? history[i].est_position : history[i].true_position;
    const float fade = static_cast<float>(i) / history.size();
    DrawLine3D(to_raylib(a), to_raylib(b), with_alpha(color, static_cast<unsigned char>(40 + 200 * fade)));
  }
}

void draw_ring(const Ring& ring, Color color) {
  // Circle normal along the heading (horizontal).
  const float angle_deg = static_cast<float>(90.0 + ring.heading * 180.0 / M_PI);
  for (int k = -2; k <= 2; ++k) {
    DrawCircle3D(to_raylib(ring.center), 0.18f + 0.004f * k, Vector3{0, 1, 0}, angle_deg, color);
  }
}

// Bar from 0 to value/max with a label.
void draw_bar(int x, int y, int w, int h, double fraction, Color color, const char* label) {
  fraction = std::clamp(fraction, 0.0, 1.0);
  DrawRectangle(x, y, w, h, Color{50, 50, 48, 255});
  DrawRectangle(x, y, static_cast<int>(w * fraction), h, color);
  DrawText(label, x + w + 8, y - 1, 14, kTextMuted);
}

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::kManual: return "MANUAL";
    case Mode::kMission: return "MISSION (steps.yaml)";
    case Mode::kRace: return "RING RACE";
  }
  return "";
}

const char* camera_name(CameraMode camera) {
  switch (camera) {
    case CameraMode::kChase: return "chase";
    case CameraMode::kOrbit: return "orbit";
    case CameraMode::kTop: return "top";
  }
  return "";
}

}  // namespace

Vector3 to_raylib(const Eigen::Vector3d& world) {
  return Vector3{static_cast<float>(world.x()), static_cast<float>(world.z()),
                 static_cast<float>(-world.y())};
}

Camera3D make_camera(const GameState& state, const sim::Simulation& simulation) {
  const Eigen::Vector3d drone = simulation.last_sample().true_position;
  Camera3D camera = {};
  camera.fovy = 50.0f;
  camera.projection = CAMERA_PERSPECTIVE;
  camera.up = Vector3{0, 1, 0};

  Eigen::Vector3d target;
  Eigen::Vector3d eye;
  if (state.camera == CameraMode::kChase) {
    target = drone + Eigen::Vector3d(0, 0, 0.05);
    const double distance = 0.9;
    const double pitch = 0.35;
    eye = target + distance * Eigen::Vector3d(-std::cos(state.chase_yaw) * std::cos(pitch),
                                              -std::sin(state.chase_yaw) * std::cos(pitch),
                                              std::sin(pitch));
  } else if (state.camera == CameraMode::kOrbit) {
    target = Eigen::Vector3d(drone.x() * 0.5, drone.y() * 0.5, 0.5);
    eye = target + state.orbit_distance *
                       Eigen::Vector3d(std::cos(state.orbit_yaw) * std::cos(state.orbit_pitch),
                                       std::sin(state.orbit_yaw) * std::cos(state.orbit_pitch),
                                       std::sin(state.orbit_pitch));
  } else {  // top: world +x points up on the screen
    target = Eigen::Vector3d(0, 0, 0);
    eye = Eigen::Vector3d(0, 0, 4.2);
    camera.up = to_raylib(Eigen::Vector3d(1, 0, 0));
  }
  camera.position = to_raylib(eye);
  camera.target = to_raylib(target);
  return camera;
}

SceneAssets load_assets() {
  SceneAssets assets;
  // Textured floor: the optical-flow sensor needs texture, so does the eye.
  Image checker = GenImageChecked(512, 512, 32, 32, Color{48, 48, 46, 255}, Color{36, 36, 35, 255});
  Texture2D texture = LoadTextureFromImage(checker);
  UnloadImage(checker);
  assets.floor = LoadModelFromMesh(GenMeshPlane(8.0f, 8.0f, 1, 1));
  assets.floor.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
  return assets;
}

void unload_assets(SceneAssets& assets) {
  UnloadTexture(assets.floor.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture);
  UnloadModel(assets.floor);
}

void draw_scene(const SceneAssets& assets, const GameState& state, sim::Simulation& simulation,
                const cf_model::Params& params, const cf_bringup::SafetyConfig& safety) {
  const sim::ControlSample& s = simulation.last_sample();
  const sim::TrueState& truth = simulation.drone().state();

  ClearBackground(kBackground);
  BeginMode3D(make_camera(state, simulation));

  DrawModel(assets.floor, Vector3{0, 0, 0}, 1.0f, WHITE);
  DrawGrid(8, 1.0f);

  // Geofence box (red when the estimate is outside).
  const Eigen::Vector3d fence_center = 0.5 * (safety.geofence_min + safety.geofence_max);
  const Eigen::Vector3d fence_size = safety.geofence_max - safety.geofence_min;
  const bool outside = (s.estimate.position.array() < safety.geofence_min.array()).any() ||
                       (s.estimate.position.array() > safety.geofence_max.array()).any();
  DrawCubeWiresV(to_raylib(fence_center),
                 Vector3{(float)fence_size.x(), (float)fence_size.z(), (float)fence_size.y()},
                 outside ? kDanger : with_alpha(kTextMuted, 110));

  // Reference: where the MPC is told to go.
  const Eigen::Vector3d ref = s.reference.position;
  DrawSphere(to_raylib(ref), 0.03f, with_alpha(kCommand, 200));
  DrawLine3D(to_raylib(ref), to_raylib(Eigen::Vector3d(ref.x(), ref.y(), 0.0)), with_alpha(kCommand, 120));
  const Eigen::Vector3d heading(std::cos(s.reference.yaw), std::sin(s.reference.yaw), 0.0);
  DrawLine3D(to_raylib(ref), to_raylib(ref + 0.12 * heading), kCommand);

  // Rings.
  if (state.mode == Mode::kRace) {
    for (size_t i = 0; i < state.rings.size(); ++i) {
      if (static_cast<int>(i) < state.next_ring) continue;
      const bool next = static_cast<int>(i) == state.next_ring;
      draw_ring(state.rings[i], next ? kWarning : with_alpha(kTextMuted, 120));
    }
  }

  // Trails.
  draw_trail(state.history, false, kTrue);
  if (state.use_ekf) draw_trail(state.history, true, kEstimate);

  // Shadow.
  DrawCylinder(to_raylib(Eigen::Vector3d(truth.position.x(), truth.position.y(), 0.001)),
               0.07f, 0.07f, 0.001f, 24, Color{0, 0, 0, 90});

  // Wind gust arrow.
  if (state.gust_force.norm() > 0.0) {
    const Eigen::Vector3d from = truth.position - state.gust_force.normalized() * 0.45;
    DrawCylinderEx(to_raylib(from), to_raylib(truth.position - state.gust_force.normalized() * 0.12),
                   0.006f, 0.0f, 8, kWarning);
  }

  // EKF estimate as a wireframe ghost.
  if (state.use_ekf) {
    push_body_transform(s.estimate.position,
                        cf_model::rotation_zyx(s.estimate.roll, s.estimate.pitch, s.estimate.yaw));
    draw_drone_wire(params, kEstimate);
    rlPopMatrix();
  }

  // The true drone.
  push_body_transform(truth.position, truth.attitude.toRotationMatrix());
  draw_drone_solid(params, state.prop_angle, truth.motor_thrust,
                   cf_model::thrust_per_motor_from_cmd(params, params.thrust_cmd_max));
  rlPopMatrix();

  EndMode3D();
}

void draw_hud(const GameState& state, sim::Simulation& simulation, const cf_model::Params& params,
              double wall_time) {
  const sim::ControlSample& s = simulation.last_sample();
  const sim::TrueState& truth = simulation.drone().state();
  const int screen_w = GetScreenWidth();
  const int screen_h = GetScreenHeight();
  const double deg = 180.0 / M_PI;

  // ---- status panel (top left) ----
  DrawRectangle(10, 10, 330, 182, kPanel);
  const char* status = "DISARMED";
  Color status_color = kTextMuted;
  if (s.armed && !s.stopped) {
    status = "ARMED - MPC in control";
    status_color = kCommand;
  } else if (s.armed && s.stopped) {
    status = "MOTORS OFF (safety stop)";
    status_color = kDanger;
  }
  DrawText(status, 22, 20, 20, status_color);
  DrawText(TextFormat("mode: %s", mode_name(state.mode)), 22, 48, 16, kText);
  DrawText(TextFormat("t = %.1f s   camera: %s%s", simulation.time(), camera_name(state.camera),
                      state.paused ? "   PAUSED" : ""), 22, 70, 16, kTextMuted);
  DrawText(TextFormat("[T] estimator: %s", state.use_ekf ? "EKF (flow + ToF + IMU)" : "ground truth"),
           22, 96, 16, state.use_ekf ? kEstimate : kTextMuted);
  DrawText(TextFormat("[O] disturbance observer: %s", state.observer ? "on" : "off"), 22, 118, 16, kText);
  DrawText(TextFormat("[M] payload: %s", state.payload ? "+15 % mass" : "none"), 22, 140, 16,
           state.payload ? kWarning : kText);
  DrawText(TextFormat("[G] wind gust: %s", state.gust_force.norm() > 0 ? "BLOWING" : "-"), 22, 162,
           16, state.gust_force.norm() > 0 ? kWarning : kText);

  // ---- telemetry panel (top right) ----
  const int px = screen_w - 330;
  DrawRectangle(px, 10, 320, 300, kPanel);
  int y = 20;
  DrawText("TELEMETRY", px + 12, y, 18, kText);
  y += 28;
  DrawText(TextFormat("altitude   true %.2f m   est %.2f m", truth.position.z(), s.estimate.position.z()),
           px + 12, y, 15, kText);
  y += 20;
  DrawText(TextFormat("speed      %.2f m/s", truth.velocity.norm()), px + 12, y, 15, kText);
  y += 20;
  DrawText(TextFormat("roll %+5.1f  pitch %+5.1f  yaw %+6.1f deg", s.true_euler.x() * deg,
                      s.true_euler.y() * deg, s.true_euler.z() * deg), px + 12, y, 15, kText);
  y += 20;
  const double est_error = (s.estimate.position - s.true_position).norm();
  DrawText(TextFormat("EKF position error  %.1f cm", 100.0 * est_error), px + 12, y, 15, kEstimate);
  y += 20;
  DrawText(TextFormat("MPC solve %.2f ms   disturbance %+.3f N", 1000.0 * s.solve_time, s.disturbance),
           px + 12, y, 15, kTextMuted);
  y += 28;
  draw_bar(px + 12, y, 180, 12, s.sent.thrust_cmd / params.thrust_cmd_max, kCommand,
           TextFormat("thrust cmd %.0f", s.sent.thrust_cmd));
  y += 22;
  const double max_motor = cf_model::thrust_per_motor_from_cmd(params, params.thrust_cmd_max);
  const char* motor_names[4] = {"FR", "BR", "BL", "FL"};
  for (int i = 0; i < 4; ++i) {
    draw_bar(px + 12, y, 180, 10, truth.motor_thrust(i) / max_motor, kTrue,
             TextFormat("motor %s %.0f mN", motor_names[i], 1000.0 * truth.motor_thrust(i)));
    y += 18;
  }
  y += 6;
  DrawText(TextFormat("cmd  roll %+5.1f  pitch %+5.1f deg", s.sent.roll * deg, s.sent.pitch * deg),
           px + 12, y, 15, kCommand);

  // ---- altitude plot (bottom right), last 10 s ----
  const int gw = 320;
  const int gh = 170;
  const int gx = screen_w - gw - 10;
  const int gy = screen_h - gh - 10;
  DrawRectangle(gx, gy, gw, gh, kPanel);
  DrawText("altitude [m], last 10 s", gx + 8, gy + 6, 14, kTextMuted);
  const double z_max = 1.2;
  const int plot_top = gy + 28;
  const int plot_bottom = gy + gh - 10;
  auto plot_y = [&](double z) {
    return plot_bottom - static_cast<int>((plot_bottom - plot_top) * std::clamp(z / z_max, 0.0, 1.0));
  };
  for (double z : {0.0, 0.5, 1.0}) {  // reference lines with labels
    DrawLine(gx + 34, plot_y(z), gx + gw - 8, plot_y(z), Color{60, 60, 58, 255});
    DrawText(TextFormat("%.1f", z), gx + 8, plot_y(z) - 6, 12, kTextMuted);
  }
  if (!state.history.empty()) {
    const double t_end = state.history.back().t;
    auto plot_x = [&](double t) { return gx + 34 + static_cast<int>((gw - 42) * (1.0 - (t_end - t) / 10.0)); };
    for (size_t i = 1; i < state.history.size(); ++i) {
      const HistoryPoint& a = state.history[i - 1];
      const HistoryPoint& b = state.history[i];
      DrawLine(plot_x(a.t), plot_y(a.ref_z), plot_x(b.t), plot_y(b.ref_z), kTextMuted);
      DrawLine(plot_x(a.t), plot_y(a.est_position.z()), plot_x(b.t), plot_y(b.est_position.z()), kEstimate);
      DrawLine(plot_x(a.t), plot_y(a.true_position.z()), plot_x(b.t), plot_y(b.true_position.z()), kTrue);
    }
  }
  DrawText("true", gx + gw - 140, gy + 6, 14, kTrue);
  DrawText("EKF", gx + gw - 95, gy + 6, 14, kEstimate);
  DrawText("ref", gx + gw - 55, gy + 6, 14, kTextMuted);

  // ---- race panel ----
  if (state.mode == Mode::kRace) {
    DrawRectangle(10, 200, 330, 70, kPanel);
    if (state.race_time >= 0.0) {
      DrawText(TextFormat("FINISHED in %.1f s", state.race_time), 22, 210, 20, kCommand);
    } else {
      DrawText(TextFormat("ring %d / %d   time %.1f s", state.next_ring + 1,
                          static_cast<int>(state.rings.size()), simulation.time() - state.race_start),
               22, 210, 20, kWarning);
    }
    if (state.best_race_time >= 0.0) {
      DrawText(TextFormat("best %.1f s", state.best_race_time), 22, 240, 16, kTextMuted);
    }
  }

  // ---- stop banner ----
  if (s.armed && s.stopped) {
    const char* line1 = TextFormat("SAFETY STOP: %s", s.stop_reason.c_str());
    const char* line2 = "ENTER = re-arm    X = reset everything";
    const int w1 = MeasureText(line1, 30);
    DrawRectangle(screen_w / 2 - w1 / 2 - 20, screen_h / 2 - 50, w1 + 40, 90, Color{60, 20, 20, 220});
    DrawText(line1, screen_w / 2 - w1 / 2, screen_h / 2 - 38, 30, kDanger);
    DrawText(line2, screen_w / 2 - MeasureText(line2, 18) / 2, screen_h / 2 + 5, 18, kText);
  }

  // ---- message ----
  if (wall_time < state.message_until && !state.message.empty()) {
    const int w = MeasureText(state.message.c_str(), 22);
    DrawRectangle(screen_w / 2 - w / 2 - 14, 14, w + 28, 36, kPanel);
    DrawText(state.message.c_str(), screen_w / 2 - w / 2, 21, 22, kText);
  }

  // ---- help (bottom left) ----
  if (state.show_help) {
    const char* lines[] = {
        "ENTER arm / take off      SPACE kill switch      L land",
        "W A S D move target       R F up / down          Q E turn",
        "1 manual   2 mission   3 ring race               C camera",
        "G wind gust   M payload   O observer   T EKF/truth",
        "P pause   X reset   H hide help   right mouse: orbit   wheel: zoom",
    };
    const int n = 5;
    DrawRectangle(10, screen_h - 18 * n - 24, 560, 18 * n + 14, kPanel);
    for (int i = 0; i < n; ++i) {
      DrawText(lines[i], 22, screen_h - 18 * n - 16 + 18 * i, 15, kTextMuted);
    }
  }
}

}  // namespace game
