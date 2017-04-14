#ifndef CASSIE_SYSTEM_H
#define CASSIE_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif


#include "cassie_system_types.h"


cassie_system_t* cassie_system_alloc(void);
void cassie_system_free(cassie_system_t *system);
void cassie_system_init(const cassie_system_t *system);
void cassie_system_step(const cassie_system_t *system,
                        ethercat_data_t *ethercat);


#ifdef __cplusplus
}
#endif

#endif // CASSIE_SYSTEM_H
