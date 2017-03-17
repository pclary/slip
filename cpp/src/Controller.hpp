#pragma once

#include "Robot.hpp"


struct controller_params_t {
    double target_dx        = 0;
    double step_offset      = 0;
    double energy_injection = 0;
    double step_height      = 0.1;
    double phase_rate       = 1;

    static constexpr double robot_weight      = 30*9.81;
    static constexpr double contact_threshold = 30*9.81/2/1e4;
    static constexpr double l_max             = 0.8;
    static constexpr double ddx_filter        = 0.1;
    static constexpr double step_lock_phase   = 0.7;
    static constexpr double max_stride        = 0.4;
    static constexpr double phase_stretch     = 0;

    static constexpr double x_pd[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.5, 1.0, 1.0, 1.0, 1.0, 1.0};
    static constexpr double x_pd_kp = 1e3;
    static constexpr double x_pd_kv = 1e2;

    static constexpr double y_pd[] = {0.0, 0.0, 0.0, 0.5, 1.0,
                                      1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
    static constexpr double y_pd_kp = 4e3;
    static constexpr double y_pd_kv = 1e2;

    static constexpr double body_angle_pd = 0;
    static constexpr double body_angle_pd_en[] = {1.0, 1.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    static constexpr double body_angle_pd_kp = 2e3;
    static constexpr double body_angle_pd_kv = 1e2;

    static constexpr double weight_ff[] = {1.0, 1.0, 1.0, 0.5, 0.0,
                                           0.0, 0.0, 0.5, 1.0, 1.0, 1.0};
    static constexpr double energy_ff[] = {0.0, 0.5, 1.0, 1.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};


struct leg_controller_state_t {
    double phase;
    double foot_x_last;
    double foot_x_target;
};


struct controller_state_t {
    leg_controller_state_t right = {0, 0, 0};
    leg_controller_state_t left = {0.5, 0, 0};
    double body_ddx = 0;
    double body_dx_last = 0;
};


struct leg_control_t {
    double angle;
    double length;
};


struct control_t {
    leg_control_t right;
    leg_control_t left;
};


control_t step(const robot_state_t& X,
               controller_state_t& cstate,
               const controller_params_t& cparams,
               double Ts);
