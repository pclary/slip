#pragma once

#include "Simulation.hpp"
#include "Terrain.hpp"
#include "Goal.hpp"
#include "mujoco.h"


class StateEvaluator {
public:
    double value(robot_state_t X, const terrain_t& terrain, goal_t goal);
    double stability(robot_state_t X, const terrain_t& terrain);
    double goal_value(robot_state_t X, goal_t goal);
    double combine_value(double stability, double goal_value);

private:
    static const mjtNum b0[];
    static const mjtNum b1[];
    static const mjtNum b2[];
    static const mjtNum w0[];
    static const mjtNum w1[];
    static const mjtNum w2[];

};
