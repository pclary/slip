#include "Controller.hpp"
#include <cmath>
#include <cstddef>
#include <complex>
#include <iostream>


#define eval_traj(t, i, p) (t[i] + p * (t[i + 1] - t[i]))


// Static controller parameter arrays
constexpr double controller_params_t::x_pd[];
constexpr double controller_params_t::y_pd[];
constexpr double controller_params_t::body_angle_pd_en[];
constexpr double controller_params_t::weight_ff[];
constexpr double controller_params_t::energy_ff[];


static double clamp(double x, double lower, double upper)
{
    // Clamp x to the lower and upper bounds
    return std::fmin(std::fmax(x, lower), upper);
}


static leg_control_t leg_step(const robot_state_t& X,
                              size_t leg,
                              leg_controller_state_t& cstate,
                              mjtNum body_ddx,
                              const controller_params_t& cparams,
                              mjtNum Ts)
{
    // Get singleturn body angle to allow for flips
    auto X_body_theta = std::fmod(X.qpos[BODY_THETA] + M_PI, 2 * M_PI) - M_PI;

    // Above some speed, increase step rate instead of increasing stride length
    const auto fbdx2 = std::fabs(X.qvel[BODY_DX]) / 2;
    auto phase_rate_eq = cparams.phase_rate;
    if (fbdx2 / cparams.phase_rate > cparams.max_stride)
        phase_rate_eq = fbdx2 / cparams.max_stride;

    const auto dx_diff = clamp(X.qvel[BODY_DX] - cparams.target_dx, -0.5, 0.5);

    // Add speed-dependent term to energy injection
    auto energy_injection = cparams.energy_injection +
        500 * std::fmax(std::fabs(X.qvel[BODY_DX]) - 1, 0);

    // Energy injection for speed control
    energy_injection -= std::copysign(dx_diff * 1000, X.qvel[BODY_DX]);

    // Increase leg phases with time
    cstate.phase += Ts * cparams.phase_rate;

    // Capture foot positions at phase rollover
    if (cstate.phase >= 1)
        cstate.foot_x_last = X.qpos[BODY_X] +
            X.qpos[leg + LEG_L] * sin(X_body_theta +
                                      X.qpos[leg + LEG_THETA_EQ] +
                                      X.qpos[leg + LEG_THETA]);

    // Limit phases to [0, 1)
    cstate.phase = cstate.phase - std::floor(cstate.phase);

    // Modify timing and trajectory shapes by stretching the phase
    // phase = stretch_phase(cstate.phase, cparams.phase_stretch);
    const auto phase = cstate.phase;

    // Detect ground contact
    const auto gc = clamp(-X.qpos[leg + LEG_L] /
                          cparams.contact_threshold, 0, 1);

    // Update leg targets
    // const auto stride_eq =
    //     (0.92 / std::pow(phase_rate_eq, 0.3) +
    //      0.1*std::fmax(std::fabs(X.qvel[BODY_DX]) - 1, 0) +
         // 0.2*std::fmax(std::fabs(X.qvel[BODY_DX]) - 1.8, 0)) * X.qvel[BODY_DX];
    const auto stride_eq = 1 * X.qvel[BODY_DX];
    if (phase < cparams.step_lock_phase) {
        const auto foot_extension = (1 - phase) * stride_eq +
            0.1 * clamp(X.qvel[BODY_DX] - cparams.target_dx, -0.5, 0.5) +
            0*(0.02 + 0.01 * std::fmax(std::fabs(X.qvel[BODY_DX]) - 1, 0)) *
            body_ddx;
        cstate.foot_x_target = X.qpos[BODY_X] + foot_extension;
    }

    // Decompose phase into trajectory point index and diff proportion
    const double n_pd_pts = 10;
    double id;
    const double p = std::modf(phase * n_pd_pts, &id);
    const size_t i = id;

    // Get PD controllers for x and y foot position (leg angle and length)
    const auto x_pd_diff = cparams.x_pd[i + 1] - cparams.x_pd[i];
    const auto x_setpoint = cparams.x_pd[i] + p * x_pd_diff;
    const auto x_dsetpoint = n_pd_pts * x_pd_diff;
    const auto y_pd_diff = cparams.y_pd[i + 1] - cparams.y_pd[i];
    const auto y_setpoint = cparams.y_pd[i] + p * y_pd_diff;
    const auto y_dsetpoint = n_pd_pts * y_pd_diff;

    // Get scaled x and y targets
    const auto x = x_setpoint * (cstate.foot_x_target - cstate.foot_x_last) +
        cstate.foot_x_last - X.qpos[BODY_X];
    const auto dx = x_dsetpoint * (cstate.foot_x_target - cstate.foot_x_last) -
        X.qvel[BODY_DX];
    const auto y = cparams.l_max - y_setpoint * cparams.step_height;
    const auto dy = -y_dsetpoint * cparams.step_height;

    // Transform to polar
    leg_control_t u;
    u.length_pos = std::sqrt(x*x + y*y);
    u.length_vel = (x*dx + y*dy) / u.length_pos;
    const auto xlsin = x / u.length_pos;
    u.angle_pos = (std::fabs(xlsin) < 1 ?
                   std::asin(xlsin) :
                   std::copysign(M_PI_2, xlsin)) - X_body_theta;
    u.angle_vel = (y*dx - x*dy) / u.length_pos*u.length_pos -
        X.qvel[BODY_DTHETA];

    // Set gains
    u.angle_kp = cparams.x_pd_kp;
    u.angle_kv = cparams.x_pd_kv;
    u.length_kp = cparams.y_pd_kp;
    u.length_kv = cparams.y_pd_kv;

    // Modulate angle target control with ground contact
    u.angle_kp *= (1 - gc);
    u.angle_kv *= (1 - gc);

    // Add feedforward terms for weight compensation and energy injection
    const auto ff_length =
        eval_traj(cparams.weight_ff, i, p) * cparams.robot_weight +
        eval_traj(cparams.energy_ff, i, p) * energy_injection;
    u.length_pos += ff_length / u.length_kp;

    // // Add body angle control, modulated with ground contact
    // const auto angle_en = gc * eval_traj(cparams.body_angle_pd_en, i, p);
    // const auto new_kp = u.angle_kp - angle_en * cparams.body_angle_pd_kp;
    // u.angle_pos = (u.angle_kp * u.angle_pos) / (new_kp + 1e-3);
    // u.angle_kp = new_kp;
    // const auto new_kv = u.angle_kv - angle_en * cparams.body_angle_pd_kv;
    // u.angle_pos = (u.angle_kv * u.angle_vel) / (new_kp + 1e-3);
    // u.angle_kv = new_kv;

    // Return PD setpoints and gains
    return u;
}


control_t step(const robot_state_t& X,
               controller_state_t& cstate,
               const controller_params_t& cparams,
               mjtNum Ts)
{
    // Estimate body acceleration
    auto body_ddx_est = (X.qvel[BODY_DX] - cstate.body_dx_last) / Ts;
    cstate.body_ddx += cparams.ddx_filter * (body_ddx_est - cstate.body_ddx);
    cstate.body_dx_last = X.qvel[BODY_DX];
    std::cout << cstate.body_ddx << std::endl;

    // Controller step for each leg
    return {leg_step(X, RIGHT_LEG, cstate.right, cstate.body_ddx, cparams, Ts),
            leg_step(X, LEFT_LEG,  cstate.left,  cstate.body_ddx, cparams, Ts)};
}
