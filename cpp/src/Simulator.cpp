#include "Simulator.hpp"


size_t Simulator::instances = 0;
mjModel* Simulator::m = nullptr;


Simulator::Simulator()
{
    // Activate mujoco and load the model if this is the first instance
    if (!instances) {
        mj_activate("mjkey.txt");
        char error[1000] = "Could not load binary model";
        m = mj_loadXML("cassie-planar.xml", 0, error, 1000);
        if (!m)
            mju_error_s("Load model error: %s", error);
    }

    // Initialize mjData
    d = mj_makeData(m);

    // Take a half-update to prepare for stepping through simulation
    mj_step1(m, d);

    // Count instances for mujoco initialization/termination
    ++instances;
}


Simulator::~Simulator()
{
    // Count instances for mujoco initialization/termination
    --instances;

    // Free mjData
    mj_deleteData(d);

    // Deactivate mujoco if this is the last instance
    if (!instances) {
        mj_deleteModel(m);
        mj_deactivate();
    }
}
