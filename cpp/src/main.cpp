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
    system.ethercat.pelvisMedulla.inputs.radioChannel[8] = 819;
    system.ethercat.pelvisMedulla.inputs.radioChannel[9] = -820;

    do {
        mjtNum tstart = sim.d->time;
        while (sim.d->time - tstart < 1 / 60.0) {
            mj_step1(sim.m, sim.d);
            system.step(sim.m, sim.d);
            mj_step2(sim.m, sim.d);
        }
        system.ethercat.pelvisMedulla.inputs.radioChannel[0] =
            std::floor(std::sin(sim.d->time) * 819);
        system.ethercat.pelvisMedulla.inputs.radioChannel[1] =
            std::floor(std::sin(sim.d->time + M_PI/3) * 819);
        system.ethercat.pelvisMedulla.inputs.radioChannel[6] =
            std::floor(std::sin(sim.d->time + 2*M_PI/3) * 819);
        system.ethercat.pelvisMedulla.inputs.radioChannel[2] =
            std::floor(std::sin(sim.d->time + M_PI) * 819);
        system.ethercat.pelvisMedulla.inputs.radioChannel[3] =
            std::floor(std::sin(sim.d->time + 4*M_PI/3) * 819);
        system.ethercat.pelvisMedulla.inputs.radioChannel[7] =
            std::floor(std::sin(sim.d->time + 5*M_PI/3) * 819);
    } while (vis.update(sim.d));

    return 0;
}
