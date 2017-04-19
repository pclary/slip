#include "Simulator.hpp"
#include "Visualization.hpp"
#include "System.hpp"
#include "mujoco.h"
#include <cmath>


int main(void)
{
    Simulator sim;
    Visualization vis(sim.m);
    System system;

    system.ethercat.pelvisMedulla.inputs.radioSignalGood = true;
    system.ethercat.pelvisMedulla.inputs.radioChannel[8] = 1;
    system.ethercat.pelvisMedulla.inputs.radioChannel[9] = -1;

    do {
        mjtNum tstart = sim.d->time;
        while (sim.d->time - tstart < 1 / 60.0) {
            mj_step1(sim.m, sim.d);
            system.step(sim.m, sim.d);
            mj_step2(sim.m, sim.d);
        }
        system.ethercat.pelvisMedulla.inputs.radioChannel[1] =
            std::sin(sim.d->time);
    } while (vis.update(sim.d));

    return 0;
}
