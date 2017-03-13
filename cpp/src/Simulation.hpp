#pragma once

#include "Terrain.hpp"
#include "Controller.hpp"
#include "mujoco.h"


enum {
    BODY_X = 0,
    BODY_Y,
    BODY_THETA,
    RIGHT_L,
    RIGHT_L_EQ,
    RIGHT_THETA,
    RIGHT_THETA_EQ,
    LEFT_L,
    LEFT_L_EQ,
    LEFT_THETA,
    LEFT_THETA_EQ,

    BODY_DX = 0,
    BODY_DY,
    BODY_DTHETA,
    RIGHT_DL,
    RIGHT_DL_EQ,
    RIGHT_DTHETA,
    RIGHT_DTHETA_EQ,
    LEFT_DL,
    LEFT_DL_EQ,
    LEFT_DTHETA,
    LEFT_DTHETA_EQ
};


struct robot_state_t {
    mjtNum time;
    mjtNum qpos[11];
    mjtNum qvel[11];
    mjtNum act;
};


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
