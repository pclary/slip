#include "Simulator.hpp"
#include "System.hpp"



int main(void)
{
    Simulator simulator;
    System system;
    ethercat_data_t ethercat;

    for (int i = 0; i < 600; ++i)
        simulator.step(&ethercat);

    return 0;
}
