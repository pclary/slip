#include "Planner.hpp"
#include <cmath>


Tree<tree_data_t>* Planner::best_child(const Tree<tree_data_t>* n)
{
    auto v_max = -std::numeric_limits<double>::infinity();
    Tree<tree_data_t>* n_max = nullptr;
    for (auto c : n->children) {
        if (c->data.path_value > v_max) {
            v_max = c->data.path_value;
            n_max = c;
        }
    }
    return n_max;
}


controller_params_t Planner::reset_tree(const simulation_state_t& ss)
{
    // Store the highest value path in the action stack
    action_queue = std::queue<controller_params_t>();
    auto n = &tree;
    while (!n->children.empty()) {
        n = best_child(n);
        if (!n) break;
        action_queue.push(n->data.cparams);
    }

    // Pop the paremeters to execute
    controller_params_t cparams;
    if (!action_queue.empty()) {
        cparams = action_queue.front();
        action_queue.pop();
    }

    // Simulate the upcoming tree timestep
    const auto td = simulate_transition(&ss, 1, cparams, goal, Ts_tree);

    // Reset tree with predicted state as root
    tree.clear();
    tree.data = td;
    rollout_node = &tree;

    return cparams;
}


void Planner::expand_tree()
{
    // Check whether max depth on current rollout has been reached
    if (rollout_node->depth >= rollout_depth) {
        // Evaluate leaf node
        auto path_value =
            state_evaluator.combine_value(rollout_node->data.stability,
                                          rollout_node->data.goal_value);

        // Propogate value to parents
        auto n = rollout_node->parent;
        while (n) {
            // Combine potential new path value
            // path_value retained from child
            const auto new_path_value = path_value +
                state_evaluator.combine_value(n->data.stability,
                                              n->data.goal_value);

            // Set node value if greater than previous value
            if (n->data.path_value < new_path_value) {
                path_value = new_path_value;
                n->data.path_value = path_value;
            } else {
                // Otherwise, stop backprop
                break;
            }

            // Move to parent
            n = n->parent;
        }

        // Start new rollout
        rollout_node = rand_tree_node();
    }

    // Expand on current rollout node

    // Generate a set of parameters to try
    const auto terrain =
        get_local_terrain(ground_data,
                          rollout_node->data.ss[0].X.qpos[BODY_X]);
    const auto cparams_gen = generate_params(rollout_node->data, terrain);

    // Simulate the transition multiple times to estimate stochasticity
    const auto td =
        simulate_transition(rollout_node->data.ss, TRANSITION_SAMPLES,
                            cparams_gen, goal, Ts_tree);

    // If the stability is reasonably high, add it as a child and
    // continue the rollout, otherwise start a new rollout
    if (td.stability > 0.3)
        rollout_node = rollout_node->add_child(td);
    else
        rollout_node = rand_tree_node();
}


controller_params_t Planner::generate_params(tree_data_t& td,
                                             const terrain_t& terrain)
{
    std::uniform_real_distribution<double> uniform(0, 1);

    // If action queue has anything, use that
    if (!action_queue.empty()) {
            auto cparams = action_queue.front();
            action_queue.pop();
            return cparams;
    }

    // Otherwise, generate a new set of controller parameters
    controller_params_t cparams;
    cparams.target_dx = goal.dx;

    switch (td.trials) {
    case 0: // Same as last step
        cparams = td.cparams;
        break;
    case 1: // Stop
        cparams.target_dx = 0;
        break;
    case 2: // Short step
        cparams.step_offset = -0.1;
        break;
    case 3: // Shorter step
        cparams.step_offset = -0.2;
        break;
    case 4: // Long step
        cparams.step_offset = 0.1;
        break;
    case 5: // Longer step
        cparams.step_offset = 0.2;
        break;
    case 6: // Jump
        cparams.energy_injection = 400;
        break;
    case 7: // Big jump
        cparams.energy_injection = 800;
        break;
    case 8: // High step
        cparams.step_height += 0.1;
        break;
    case 9: // Higher step
        cparams.step_height += 0.2;
        break;
    default: // Random
        cparams.step_offset = (uniform(rng) - 0.5) * 0.5;
        cparams.step_height = uniform(rng) * 0.2 + 0.1;
        cparams.energy_injection = (uniform(rng) - 0.5) * 1000;
        cparams.phase_rate = uniform(rng) + 1;
        cparams.target_dx = goal.dx * (uniform(rng) + 0.5);
        break;
    }

    // Increment number of trials
    ++td.trials;

    return cparams;
}


tree_data_t Planner::simulate_transition(const simulation_state_t* ss,
                                         size_t ss_len,
                                         controller_params_t cparams,
                                         goal_t goal,
                                         double time)
{
    // Initialize tree data
    tree_data_t td;
    td.cparams = cparams;
    double st[TRANSITION_SAMPLES];

    // Random distributions
    std::uniform_int_distribution<size_t> sample(0, ss_len - 1);
    std::uniform_real_distribution<mjtNum> pm1(-1, 1);

    // Run several simulations
    for (size_t i = 0; i < TRANSITION_SAMPLES; ++i) {
        // Pick a random starting state
        auto ss0 = ss[sample(rng)];

        // Get local terrain
        const auto terrain = get_local_terrain(ground_data, ss0.X.qpos[BODY_X]);

        // Perturb initial conditions
        if (i > 0) {
            ss0.X.qpos[BODY_X]  += 1e-2 * pm1(rng);
            ss0.X.qpos[BODY_Y]  += 0e-3 * pm1(rng);
            ss0.X.qvel[BODY_DX] += 1e-1 * pm1(rng);
            ss0.X.qvel[BODY_DY] += 3e-2 * pm1(rng);
        }

        // Simulate a step
        td.ss[i] = robot_sim(ss0, cparams, terrain, time, model, mjdata);

        // Evaluate the result
        const auto terrainp =
            get_local_terrain(ground_data, td.ss[i].X.qpos[BODY_X]);
        st[i] = state_evaluator.stability(ss[i].X, terrainp);
        td.goal_value += state_evaluator.goal_value(ss[i].X, goal);
    }

    // Average the goal value and take the softmin of the stability score
    td.goal_value /= TRANSITION_SAMPLES;
    double stexpsum = 0;
    for (size_t i = 0; i < TRANSITION_SAMPLES; ++i) {
        const double stexp = 1.0 - std::exp(st[i]);
        td.stability += st[i] * stexp;
        stexpsum += stexp;
    }
    td.stability /= stexpsum;

    return td;
}


Tree<tree_data_t>* Planner::rand_tree_node()
{
    std::uniform_real_distribution<double> uniform(0, 1);

    // Start at tree root
    auto n = &tree;

    // Descend until the level above the max depth
    for (size_t d = 0; d < rollout_depth - 1; ++d) {
        double weight = 1 / (rollout_depth - d + 1);
        if (uniform(rng) < weight || n->children.empty()) {
            // If rand > weight or no children, stop here
            break;
        } else {
            // Pick a random child
            std::uniform_int_distribution<size_t>
                sample(0, n->children.size() - 1);
            n = n->children[sample(rng)];
        }
    }

    return n;
}
