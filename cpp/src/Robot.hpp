#pragma once

#include "mujoco.h"


enum {
    BODY_X = 0,
    BODY_Y,
    BODY_ANGLE,
    RIGHT_ANGLE,
    RIGHT_ANGLE_SPRING,
    RIGHT_LENGTH,
    RIGHT_LENGTH_SPRING,
    LEFT_ANGLE,
    LEFT_ANGLE_SPRING,
    LEFT_LENGTH,
    LEFT_LENGTH_SPRING,
    POS_COUNT,

    RIGHT_LEG         = RIGHT_ANGLE,
    LEFT_LEG          = LEFT_ANGLE,
    LEG_ANGLE         = RIGHT_ANGLE - RIGHT_LEG,
    LEG_ANGLE_SPRING  = RIGHT_ANGLE_SPRING - RIGHT_LEG,
    LEG_LENGTH        = RIGHT_LENGTH - RIGHT_LEG,
    LEG_LENGTH_SPRING = RIGHT_LENGTH_SPRING - RIGHT_LEG,

    BODY_DX = 0,
    BODY_DY,
    BODY_DANGLE,
    RIGHT_DANGLE,
    RIGHT_DANGLE_SPRING,
    RIGHT_DLENGTH,
    RIGHT_DLENGTH_SPRING,
    LEFT_DANGLE,
    LEFT_DANGLE_SPRING,
    LEFT_DLENGTH,
    LEFT_DLENGTH_SPRING,
    VEL_COUNT,

    LEG_DANGLE         = RIGHT_DANGLE - RIGHT_LEG,
    LEG_DANGLE_SPRING  = RIGHT_DANGLE_SPRING - RIGHT_LEG,
    LEG_DLENGTH        = RIGHT_DLENGTH - RIGHT_LEG,
    LEG_DLENGTH_SPRING = RIGHT_DLENGTH_SPRING - RIGHT_LEG,
};


struct robot_state_t {
    mjtNum time;
    mjtNum qpos[POS_COUNT];
    mjtNum qvel[VEL_COUNT];
    // mjtNum act;
};


robot_state_t get_robot_state(const mjData* d);
void set_robot_state(const robot_state_t& X, mjData* d);
