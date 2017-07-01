#ifndef __PROFILE_H_INCLUDED__
#define __PROFILE_H_INCLUDED__

#include <stdint.h>

typedef enum {
    PROFILE_UNKNOWN,
    PROFILE_LEAD,
    PROFILE_LEAD_FREE
} profile_select_t;

#ifdef __cplusplus
extern "C" {
#endif

const char * profile_phase(uint32_t seconds);
void profile_select(profile_select_t profile);
uint32_t profile_set(uint32_t seconds);


#ifdef __cplusplus
}
#endif


#endif //  __PROFILE_H_INCLUDED__
