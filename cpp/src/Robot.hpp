#pragma once

#include "mujoco.h"


enum {
    BODY_X = 0,
    BODY_Y,
    BODY_THETA,
    RIGHT_THETA_EQ,
    RIGHT_THETA,
    RIGHT_L_EQ,
    RIGHT_L,
    LEFT_THETA_EQ,
    LEFT_THETA,
    LEFT_L_EQ,
    LEFT_L,
    POS_COUNT,

    RIGHT_LEG    = RIGHT_THETA_EQ,
    LEFT_LEG     = LEFT_THETA_EQ,
    LEG_THETA_EQ = RIGHT_THETA_EQ - RIGHT_LEG,
    LEG_THETA    = RIGHT_THETA - RIGHT_LEG,
    LEG_L_EQ     = RIGHT_L_EQ - RIGHT_LEG,
    LEG_L        = RIGHT_L - RIGHT_LEG,

    BODY_DX = 0,
    BODY_DY,
    BODY_DTHETA,
    RIGHT_DTHETA_EQ,
    RIGHT_DTHETA,
    RIGHT_DL_EQ,
    RIGHT_DL,
    LEFT_DTHETA_EQ,
    LEFT_DTHETA,
    LEFT_DL_EQ,
    LEFT_DL,
    VEL_COUNT,

    LEG_DTHETA_EQ = RIGHT_DTHETA_EQ - RIGHT_LEG,
    LEG_DTHETA    = RIGHT_DTHETA - RIGHT_LEG,
    LEG_DL_EQ     = RIGHT_DL_EQ - RIGHT_LEG,
    LEG_DL        = RIGHT_DL - RIGHT_LEG,
    
    RIGHT_THETA_POS = 0,
    RIGHT_THETA_VEL,
    RIGHT_L_POS,
    RIGHT_L_VEL,
    LEFT_THETA_POS,
    LEFT_THETA_VEL,
    LEFT_L_POS,
    LEFT_L_VEL
};


struct robot_state_t {
    mjtNum time;
    mjtNum qpos[POS_COUNT];
    mjtNum qvel[VEL_COUNT];
    mjtNum act;
};
