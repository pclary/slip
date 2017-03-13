#pragma once

#include "Tree.hpp"
#include "StateEvaluator.hpp"
#include "Simulation.hpp"
#include "Controller.hpp"
#include "Goal.hpp"
#include <queue>
#include <cstddef>
#include <limits>
#include <random>


#define TRANSITION_SAMPLES 4


struct tree_data_t
{
    simulation_state_t  ss[TRANSITION_SAMPLES];
    controller_params_t cparams;
    size_t trials     = 0;
    double stability  = 0;
    double goal_value = 0;
    double path_value = -std::numeric_limits<double>::infinity();
};


class Planner {
public:
    goal_t goal = {};
    size_t rollout_depth = 0;
    double Ts_tree = 0;
    mjModel* model = nullptr;
    mjData* mjdata = nullptr;
    ground_data_t ground_data = {};

    Tree<tree_data_t>* best_child(const Tree<tree_data_t>* n);
    controller_params_t reset_tree(const simulation_state_t& ss);
    void expand_tree();

private:

    Tree<tree_data_t> tree;
    Tree<tree_data_t>* rollout_node = &tree;
    StateEvaluator state_evaluator;
    std::queue<controller_params_t> action_queue;

    std::default_random_engine rng;

    controller_params_t generate_params(tree_data_t& td,
                                        const terrain_t& terrain);

    tree_data_t simulate_transition(const simulation_state_t* ss,
                                    size_t ss_len,
                                    controller_params_t cparams,
                                    goal_t goal,
                                    double time);

    Tree<tree_data_t>* rand_tree_node();
};
