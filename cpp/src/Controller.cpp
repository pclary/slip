#include "Controller.hpp"
#include <cmath>
#include <cstddef>


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
    auto X_body_theta = std::fmod(X.qpos[BODY_THETA] + M_PI, M_2_PI) - M_PI;

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
            X.qpos[leg + LEG_L] * sin(X.qpos[BODY_THETA] +
                                      X.qpos[leg + LEG_THETA]);

    // Limit phases to [0, 1)
    cstate.phase = cstate.phase - std::floor(cstate.phase);

    // Modify timing and trajectory shapes by stretching the phase
    // phase = stretch_phase(cstate.phase, cparams.phase_stretch);
    const auto phase = cstate.phase;

    // Detect ground contact
    const auto gc = clamp((X.qpos[leg + LEG_L_EQ] - X.qpos[leg + LEG_L]) /
                          cparams.contact_threshold, 0, 1);

    // Update leg targets
    const auto stride_eq =
        (0.92 / std::pow(phase_rate_eq, 0.3) +
         0.1*std::fmax(std::fabs(X.qvel[BODY_DX]) - 1, 0) +
         0.2*std::fmax(std::fabs(X.qvel[BODY_DX]) - 1.8, 0)) * X.qvel[BODY_DX];
    if (phase < cparams.step_lock_phase) {
        const auto foot_extension = (1 - phase) * stride_eq +
            0.15 * clamp(X.qvel[BODY_DX] - cparams.target_dx, -0.5, 0.5) +
            (0.02 + 0.01 * std::fmax(std::fabs(X.qvel[BODY_DX]) - 1, 0)) *
            body_ddx;
        cstate.foot_x_target = X.qpos[BODY_X] + foot_extension;
    }

    // Get PD controllers for x and y foot position (leg angle and length)
    leg_pd.y = get_pd(cparams.y_pd, phase);
    leg_pd.x = get_pd(cparams.x_pd, phase);

    // Get transformed leg PD output
    u = eval_leg_pd(leg_pd, body, leg, cparams, cstate.foot_x_last, cstate.foot_x_target);

    // Modulate angle target control with ground contact
    u.theta_eq = (1 - gc) * u.theta_eq;

    // Add feedforward terms for weight compensation and energy injection
    u.l_eq = u.l_eq + ...
               eval_ff(cparams.weight_ff, phase) * cparams.robot_weight + ...
               eval_ff(cparams.energy_ff, phase) * energy_injection;

    // Add body angle control, modulated with ground contact
    u.theta_eq = u.theta_eq - ...
               gc * eval_pd(cparams.body_angle_pd, phase, body.theta, body.dtheta, 0, 0);
}


control_t step(const robot_state_t& X,
               controller_state_t& cstate,
               const controller_params_t& cparams,
               mjtNum Ts)
{
    // Estimate body acceleration
    auto body_ddx_est = (X.qvel[BODY_DX] - cstate.body_dx_last) / Ts;
    cstate.body_ddx += cparams.ddx_filter * (body_ddx_est - cstate.body_ddx);
    cstate.body_dx_last = X.qpos[BODY_DX];

    // Controller step for each leg
    return {leg_step(X, RIGHT_LEG, cstate.right, cstate.body_ddx, cparams, Ts),
            leg_step(X, LEFT_LEG,  cstate.left,  cstate.body_ddx, cparams, Ts)};
}
