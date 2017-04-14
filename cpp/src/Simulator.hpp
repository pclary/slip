#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <cstddef>
#include "mujoco.h"
#include "cassie_system_types.h"
#include "Visualization.hpp"


class Simulator
{
public:
    Simulator();
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    Simulator(Simulator&&) = default;
    Simulator& operator=(Simulator&&) = default;
    ~Simulator();

    void step(ethercat_data_t* ethercat);

private:
    static size_t instances;
    static mjModel* m;

    mjData* d;
    Visualization* vis;

};


#endif // SIMULATOR_HPP
