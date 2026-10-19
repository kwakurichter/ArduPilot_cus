/* SPDX-License-Identifier: GPL-3.0-or-later */
// Offline harness: exercise the checked-out production multilateration solver.
#ifdef RANGE_SOLVER_DIAGNOSTICS
static unsigned solver_failure_line;
#endif
#include "AP_Beacon_Ranging_Solver.h"
#include <cstdio>

int main()
{
    unsigned count;
    float noise;
    while (scanf("%u %f", &count, &noise) == 2) {
        if (count < 4 || count > 8) {
            return 2;
        }
        AP_Ranging_Multilateration::Observation obs[8] {};
        for (unsigned i = 0; i < count; i++) {
            if (scanf("%f %f %f %f", &obs[i].position[0], &obs[i].position[1],
                      &obs[i].position[2], &obs[i].range) != 4) {
                return 3;
            }
        }
        float position[3] {};
        float accuracy = 0;
#ifdef RANGE_SOLVER_DIAGNOSTICS
        solver_failure_line = 0;
#endif
        const bool ok = AP_Ranging_Multilateration::solve(obs, count, noise, position, accuracy);
#ifdef RANGE_SOLVER_DIAGNOSTICS
        printf("%u ", solver_failure_line);
#endif
        printf("%u %.9g %.9g %.9g %.9g\n", unsigned(ok), double(position[0]), double(position[1]),
               double(position[2]), double(accuracy));
    }
    return 0;
}
