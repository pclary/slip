#include "Simulator.hpp"
#include "Visualization.hpp"
#include "System.hpp"
#include "mujoco.h"
#include <cstring>


int main(void)
{
    Simulator sim;
    Visualization vis(sim.m);
    System system;

    double qpos_init[] =
        {0, 0, 1.01, 1, 0, 0, 0,
         -0.0305, 0, 0.4973, -1.1997, 0, 1.4267, -1.5968,
         -1.5244, 0.6472, 0, 0.9785, -0.0164, 0.01787, -0.2049,
         -0.0305, 0, 0.4973, -1.1997, 0, 1.4267, -1.5968,
         -1.5244, 0.6472, 0, 0.9786, 0.00386, -0.01524, -0.2051};

    mju_copy(sim.d->qpos, qpos_init, 35);

    do {
        mjtNum tstart = sim.d->time;
        while (sim.d->time - tstart < 1 / 60.0) {
            mj_step1(sim.m, sim.d);
            system.step(sim.m, sim.d);
            mj_step2(sim.m, sim.d);
        }
    } while (vis.update(sim.d));

    return 0;
}
