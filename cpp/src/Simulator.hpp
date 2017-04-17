#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <cstddef>
#include "mujoco.h"
#include "cassie_system_types.h"


class Simulator
{
public:
    Simulator();
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    Simulator(Simulator&&) = default;
    Simulator& operator=(Simulator&&) = default;
    ~Simulator();

    static mjModel* m;
    mjData* d;

private:
    static size_t instances;

};


#endif // SIMULATOR_HPP
