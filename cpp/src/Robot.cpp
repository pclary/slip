#include "Robot.hpp"


robot_state_t get_robot_state(const mjData* d)
{
    robot_state_t X;
    X.time = d->time;
    mju_copy(X.qpos, d->qpos, sizeof X.qpos);
    mju_copy(X.qvel, d->qvel, sizeof X.qvel);
    // mju_copy(X.act,  d->act,  sizeof X.act);

    return X;
}


void set_robot_state(const robot_state_t& X, mjData* d)
{
    d->time = X.time;
    mju_copy(d->qpos, X.qpos, sizeof X.qpos);
    mju_copy(d->qvel, X.qvel, sizeof X.qvel);
}
