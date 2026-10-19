/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <math.h>
#include <stdint.h>

namespace AP_Ranging_Multilateration {

struct Observation {
    float position[3];
    float range;
};

// Small bounded solve with scaled pivot rejection; input is copied deliberately.
inline bool solve3(const float input[3][3], const float rhs[3], float result[3])
{
    float a[3][4];
    float scale = 0;
    for (unsigned i = 0; i < 3; i++) {
        for (unsigned j = 0; j < 3; j++) {
            a[i][j] = input[i][j];
            scale = fmaxf(scale, fabsf(a[i][j]));
        }
        a[i][3] = rhs[i];
    }
    if (!isfinite(scale) || scale <= 1.0e-12) {
        return false;
    }
    for (unsigned col = 0; col < 3; col++) {
        unsigned pivot = col;
        for (unsigned row = col + 1; row < 3; row++) {
            if (fabsf(a[row][col]) > fabsf(a[pivot][col])) {
                pivot = row;
            }
        }
        if (fabsf(a[pivot][col]) < scale * 1.0e-4) {
            return false;
        }
        for (unsigned j = col; j < 4; j++) {
            const float tmp = a[col][j];
            a[col][j] = a[pivot][j];
            a[pivot][j] = tmp;
        }
        const float divisor = a[col][col];
        for (unsigned j = col; j < 4; j++) {
            a[col][j] /= divisor;
        }
        for (unsigned row = 0; row < 3; row++) {
            if (row == col) {
                continue;
            }
            const float factor = a[row][col];
            for (unsigned j = col; j < 4; j++) {
                a[row][j] -= factor * a[col][j];
            }
        }
    }
    for (unsigned i = 0; i < 3; i++) {
        result[i] = a[i][3];
        if (!isfinite(result[i])) {
            return false;
        }
    }
    return true;
}

inline bool solve(const Observation *obs, uint8_t count, float noise,
                  float position[3], float &error)
{
    if (count < 4 || count > 8 || !isfinite(noise) || noise < 0.05 || noise > 5) {
        return false;
    }
    float normal[3][3] {};
    float rhs[3] {};
    for (uint8_t i = 0; i < count; i++) {
        if (!isfinite(obs[i].range) || obs[i].range <= 0 || obs[i].range >= 1000) {
            return false;
        }
        for (unsigned k = 0; k < 3; k++) {
            if (!isfinite(obs[i].position[k])) {
                return false;
            }
        }
        if (i == 0) {
            continue;
        }
        float row[3];
        float value = obs[0].range * obs[0].range - obs[i].range * obs[i].range;
        for (unsigned k = 0; k < 3; k++) {
            const float delta = obs[i].position[k] - obs[0].position[k];
            row[k] = 2 * delta;
            value += delta * delta;
        }
        for (unsigned j = 0; j < 3; j++) {
            rhs[j] += row[j] * value;
            for (unsigned k = 0; k < 3; k++) {
                normal[j][k] += row[j] * row[k];
            }
        }
    }
    float p[3];
    if (!solve3(normal, rhs, p)) {
        return false;
    }
    for (unsigned k = 0; k < 3; k++) {
        p[k] += obs[0].position[k];
    }
    // Reject stale/moving sets in the caller; this solves one static observation set.
    for (unsigned iteration = 0; iteration < 10; iteration++) {
        float h[3][3] {};
        float gradient[3] {};
        float sum_sq = 0;
        float max_residual = 0;
        for (uint8_t i = 0; i < count; i++) {
            float unit[3];
            float squared = 0;
            for (unsigned k = 0; k < 3; k++) {
                unit[k] = p[k] - obs[i].position[k];
                squared += unit[k] * unit[k];
            }
            const float predicted = sqrtf(squared);
            if (!isfinite(predicted) || predicted < 0.05) {
                return false;
            }
            const float residual = obs[i].range - predicted;
            sum_sq += residual * residual;
            max_residual = fmaxf(max_residual, fabsf(residual));
            for (unsigned j = 0; j < 3; j++) {
                unit[j] /= predicted;
            }
            for (unsigned j = 0; j < 3; j++) {
                gradient[j] += unit[j] * residual;
                for (unsigned k = 0; k < 3; k++) {
                    h[j][k] += unit[j] * unit[k];
                }
            }
        }
        float step[3];
        if (!solve3(h, gradient, step)) {
            return false;
        }
        const float step_sq = step[0]*step[0] + step[1]*step[1] + step[2]*step[2];
        if (step_sq < 1.0e-8) {
            if (max_residual > 3 * noise) {
                return false;
            }
            float trace = 0;
            for (unsigned k = 0; k < 3; k++) {
                float unit[3] {};
                float column[3];
                unit[k] = 1;
                if (!solve3(h, unit, column) || column[k] <= 0) {
                    return false;
                }
                trace += column[k];
            }
            // Never reduce the configured sensor noise based on a small residual.
            const float variance = fmaxf(noise * noise, sum_sq / count);
            error = sqrtf(trace * variance);
            if (!isfinite(error) || error >= 1.0) {
                return false;
            }
            for (unsigned k = 0; k < 3; k++) {
                position[k] = p[k];
            }
            return true;
        }
        if (step_sq > 10000) {
            return false;
        }
        for (unsigned k = 0; k < 3; k++) {
            p[k] += step[k];
        }
    }
    return false;
}

} // namespace AP_Ranging_Multilateration
