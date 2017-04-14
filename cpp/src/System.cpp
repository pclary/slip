#include "System.hpp"
#include "cassie_system.h"


System::System()
{
    system = cassie_system_alloc();
    cassie_system_init(system);
}


System::~System()
{
    cassie_system_free(system);
}


void System::step(ethercat_data_t* ethercat)
{
    cassie_system_step(system, ethercat);
}
