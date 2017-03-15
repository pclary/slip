#pragma once


struct controller_params_t {
    double target_dx = 0;
    double step_offset = 0;
    double energy_injection = 0;
    double step_height = 0.1;
    double phase_rate = 1;
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



control_t step(robot_state_t X, controller_params_t& cparams, double Ts);
