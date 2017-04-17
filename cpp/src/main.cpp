#include "Simulator.hpp"
#include "Visualization.hpp"
#include "System.hpp"
#include "cassie_system_types.h"
#include "mujoco.h"


int main(void)
{
    Simulator sim;
    Visualization vis(sim.m);
    System system;
    ethercat_data_t ethercat;

    do {
        mj_step(sim.m, sim.d);
    } while (vis.update(sim.d));

    return 0;
}
