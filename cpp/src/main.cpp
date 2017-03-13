#include <iostream>
#include <cstddef>
#include "Planner.hpp"
#include "Simulation.hpp"


int main()
{
    mjModel model;
    mjData mjdata;
    terrain_t terrain;

    Planner planner;
    planner.Ts_tree = 0.5;
    planner.rollout_depth = 4;

    simulation_state_t ss = {};

    for (size_t i = 0; i < 100; ++i) {
        auto cparams = planner.reset_tree(ss);
        ss = robot_sim(ss, cparams, terrain, planner.Ts_tree, &model, &mjdata);
        for (size_t j = 0; j < 1000; ++j)
            planner.expand_tree();
    }

    std::cout << ss.X.qpos[0] << std::endl;

    return 0;
}
