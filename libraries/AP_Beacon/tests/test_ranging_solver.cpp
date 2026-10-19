/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "../AP_Beacon_Ranging_Solver.h"
#include <assert.h>
#include <stdio.h>
using namespace AP_Ranging_Multilateration;

static void fixture(Observation *obs, const float p[3], unsigned count)
{
    for (unsigned i = 0; i < count; i++) {
        obs[i].position[0] = (i & 1) ? 6 : 0;
        obs[i].position[1] = (i & 2) ? 5 : 0;
        obs[i].position[2] = (i & 4) ? -3 : 0;
        float square = 0;
        for (unsigned k = 0; k < 3; k++) {
            const float delta = p[k] - obs[i].position[k];
            square += delta * delta;
        }
        obs[i].range = sqrtf(square);
    }
}

int main()
{
    Observation obs[8];
    const float truth[3] = {2, 3, -1};
    float p[3], error;
    fixture(obs,truth,8);
    assert(solve(obs,8,0.3,p,error));
    for (unsigned k=0;k<3;k++) { assert(fabsf(p[k]-truth[k])<1.0e-5); }
    assert(error>0.3 && error<1.0);
    assert(!solve(obs,3,0.3,p,error));
    assert(!solve(obs,4,0.3,p,error)); // same-height anchors: no unambiguous 3D seed
    Observation sparse[4] = {obs[0],obs[1],obs[2],obs[7]};
    assert(solve(sparse,4,0.3,p,error));
    for (unsigned k=0;k<3;k++) { assert(fabsf(p[k]-truth[k])<1.0e-5); }
    for (unsigned i=0;i<8;i++) { obs[i].range += (i%3-1.0)*0.03; }
    assert(solve(obs,8,0.3,p,error));
    for (unsigned k=0;k<3;k++) { assert(fabsf(p[k]-truth[k])<0.1); }
    obs[7].range += 4;
    assert(!solve(obs,8,0.3,p,error));
    fixture(obs,truth,8); obs[0].range=NAN;
    assert(!solve(obs,8,0.3,p,error));
    fixture(obs,truth,8); obs[2].position[0]=INFINITY;
    assert(!solve(obs,8,0.3,p,error));
    puts("Multilateration exact, sparse, noisy, degenerate and invalid-input tests passed");
}
