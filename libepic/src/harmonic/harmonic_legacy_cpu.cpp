/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2016 Kyle Hollins Wray, University of Massachusetts
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the "Software"), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include <epic/harmonic/harmonic_legacy_cpu.h>
#include <epic/constants.h>
#include <epic/error_codes.h>

#include <iostream>
#include <cmath>

namespace epic {

#define MIN_ITERATIONS 10000

int harmonic_legacy_sor_2d_float_cpu(unsigned int w, unsigned int h, float epsilon,
    float omega, unsigned int *locked, float *u, unsigned int &iter)
{
    float delta = epsilon + 1.0f;
    iter = 0;

    while (delta >= epsilon || iter < MIN_ITERATIONS) {
        delta = 0.0f;

        for (unsigned int y = 1; y < h - 1; y++) {
            for (unsigned int x = 1; x < w - 1; x++) {
                if (locked[y * w + x] == 1) {
                    continue;
                }

                float uPrev = u[y * w + x];

                u[y * w + x] = (1.0f - omega) * u[y * w + x] +
                                omega / 4.0f * (u[(y - 1) * w + x] +
                                                u[(y + 1) * w + x] +
                                                u[y * w + (x - 1)] + 
                                                u[y * w + (x + 1)]);

                delta = std::fmax(delta, std::fabs(u[y * w + x] - uPrev));
            }
        }

        //printf("Iteration: %i and Delta = %.3f vs Epsilon = %.3f\n", iter, delta, epsilon);

        iter++;
    }

    return EPIC_SUCCESS;
}


int harmonic_legacy_sor_2d_double_cpu(unsigned int w, unsigned int h, double epsilon,
    double omega, unsigned int *locked, double *u, unsigned int &iter)
{
    double delta = epsilon + 1.0;
    iter = 0;

    while (delta >= epsilon || iter < MIN_ITERATIONS) {
        delta = 0.0;

        for (unsigned int y = 1; y < h - 1; y++) {
            for (unsigned int x = 1; x < w - 1; x++) {
                if (locked[y * w + x] == 1) {
                    continue;
                }

                double uPrev = u[y * w + x];

                u[y * w + x] = (1.0 - omega) * u[y * w + x] +
                                omega / 4.0 * (u[(y - 1) * w + x] +
                                                u[(y + 1) * w + x] +
                                                u[y * w + (x - 1)] + 
                                                u[y * w + (x + 1)]);

                delta = std::fmax(delta, std::fabs(u[y * w + x] - uPrev));
            }
        }

        //printf("Iteration: %i and Delta = %.3f vs Epsilon = %.3f\n", iter, delta, epsilon);

        iter++;
    }

    return EPIC_SUCCESS;
}


int harmonic_legacy_sor_2d_long_double_cpu(unsigned int w, unsigned int h, long double epsilon,
    long double omega, unsigned int *locked, long double *u, unsigned int &iter)
{
    long double delta = epsilon + 1.0;
    iter = 0;

    while (delta >= epsilon || iter < MIN_ITERATIONS) {
        delta = 0.0;

        for (unsigned int y = 1; y < h - 1; y++) {
            for (unsigned int x = 1; x < w - 1; x++) {
                if (locked[y * w + x] == 1) {
                    continue;
                }

                long double uPrev = u[y * w + x];

                u[y * w + x] = (1.0 - omega) * u[y * w + x] +
                                omega / 4.0 * (u[(y - 1) * w + x] +
                                                u[(y + 1) * w + x] +
                                                u[y * w + (x - 1)] + 
                                                u[y * w + (x + 1)]);

                delta = std::fmax(delta, std::fabs(u[y * w + x] - uPrev));
            }
        }

        //printf("Iteration: %i and Delta = %.3f vs Epsilon = %.3f\n", iter, delta, epsilon);

        iter++;
    }

    return EPIC_SUCCESS;
}

}; // namespace epic

