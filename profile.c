#include <stdio.h>

#include "profile.h"
#include "assert.h"

#include "utils/uartstdio.h"


// temp-curve.h|c
// Lead (Sn63 Pb37)
// Preheat: to 150C in around 60s (~3C/s)
// Soak: 150-165C in 120s
// Reflow: peak 225-235C hold for 20 S
// Cooling: -4C/s to room temp.
//
// Lead-Free (SAC305)
// Preheat: to 150C in around 60s (~3C/s)
// Soak: 150-180C in 120s
// Reflow: peak 245-255C hold for 15 S
// Cooling: -4C/s to room temp.

typedef struct {
    char * phase;
    int max_slew;
    int start_temp;
    int stop_temp;
    int seconds;
} profile_t;

#define NA 0

#if 1

profile_t program_lead[] = {
    {"PREHEAT", 3,  24, 150,  60}, 
    {"SOAK",   NA, 150, 165, 120}, 
    {"HEAT",   NA, 165, 230,  20}, 
    {"REFLOW", NA, 230, 230,  20}, 
    {"COOL",   -4, 230,  24, 100},
    {NULL,     NA,  NA,  NA,  NA}
};

#else
profile_t program_lead[] = {
    {"PREHEAT", 3,  24, 150,  5}, 
    {"SOAK",   NA, 150, 165, 10}, 
    {"HEAT",   NA, 165, 230,  5}, 
    {"REFLOW", NA, 230, 230,  5}, 
    {"COOL",   -4, 230,  24, 20},
    {NULL,     NA,  NA,  NA,  NA}
};

#endif

profile_t program_leadFree[] = {
    {"PREHEAT", 3,  24, 150,  60}, 
    {"SOAK",   NA, 150, 180, 120}, 
    {"HEAT",   NA, 180, 250,  20}, 
    {"REFLOW", NA, 250, 250,  20}, 
    {"COOL",   -4, 250,  24, 100},
    {NULL,     NA,  NA,  NA,  NA}
};

profile_t * current_profile = NULL;


void profile_select(profile_select_t profile)
{
    if (PROFILE_LEAD == profile)
    {
        current_profile = &program_lead[0];
        UARTprintf("PROFILE_LEAD\n");
    }
    else if (PROFILE_LEAD_FREE == profile)
    {
        current_profile = &program_lead[0];
        UARTprintf("PROFILE_LEAD_FREE\n");
    }
    else
    {
        assert(0);
    }
}

static profile_t * prior_phase = NULL;
static int       phase_change_start = 0;

static profile_t * _get_profile(uint32_t seconds)
{
    uint32_t phaseSeconds = 0;
    profile_t * walker = current_profile;

    while(walker->phase != NULL)
    {
        if ((phaseSeconds + walker->seconds) >= seconds)
        {
            // set the phase_change seconds if necessary
            if (walker != prior_phase)
            {
                prior_phase = walker;
                phase_change_start = seconds;
            }
            return walker;
        }

        phaseSeconds += walker->seconds;
        walker++;
    }
    return NULL;
}

const char * profile_phase(uint32_t seconds)
{
    profile_t * p =  _get_profile(seconds);
    
    return (p) ? p->phase : "----";
}

uint32_t profile_set(uint32_t seconds)
{
    profile_t * p =  _get_profile(seconds);
    
    if (p)
    {
        double start = p->start_temp;
        double stop = p->stop_temp;
        double slope = (stop-start) / p->seconds;
        double value = p->start_temp + (slope*(seconds-phase_change_start));
        // need to extrapolate current temp for phase 
        return (uint32_t) value;
    }
    else
        return 0;
}



