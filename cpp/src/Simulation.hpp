#pragma once

#include "Terrain.hpp"
#include "Robot.hpp"
#include "Controller.hpp"
#include "mujoco.h"


struct simulation_state_t {
    robot_state_t X;
    controller_state_t cstate;
};


simulation_state_t robot_sim(simulation_state_t ss,
                             const controller_params_t& cparams,
                             const terrain_t& terrain,
                             double time,
                             mjModel* model,
                             mjData* mjdata);
