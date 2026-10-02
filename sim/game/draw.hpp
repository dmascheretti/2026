// Drawing of the 3D scene and the HUD (raylib). Reads the simulation and
// the game state, changes nothing.
#pragma once

#include <raylib.h>

#include "cf_bringup/safety_supervisor.hpp"
#include "cf_model/params.hpp"
#include "game_state.hpp"
#include "simulation.hpp"

namespace game {

// World frame (x forward, y left, z up) -> raylib frame (y up).
Vector3 to_raylib(const Eigen::Vector3d& world);

// Camera for the current camera mode.
Camera3D make_camera(const GameState& state, const sim::Simulation& simulation);

struct SceneAssets {
  Model floor;
};

SceneAssets load_assets();
void unload_assets(SceneAssets& assets);

void draw_scene(const SceneAssets& assets, const GameState& state, sim::Simulation& simulation,
                const cf_model::Params& params, const cf_bringup::SafetyConfig& safety);

void draw_hud(const GameState& state, sim::Simulation& simulation, const cf_model::Params& params,
              double wall_time);

}  // namespace game
