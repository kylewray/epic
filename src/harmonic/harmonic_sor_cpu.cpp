/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Kyle Hollins Wray, University of Massachusetts
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


#include "harmonic.h"
#include "harmonic_sor_cpu.h"
#include "error_codes.h"
#include "constants.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>

/*
int working_cpu_harmonic_sor_2d(const unsigned int *m, double *u, unsigned int *locked, int max_iterations, double EPSILON, double OMEGA) {
  // Ensure that valid data was passed.
  if (m == nullptr || u == nullptr || EPSILON <= 0.0f) {
    fprintf(stderr, "Error[cpu_harmonic_sor_2d]: Invalid data.\n");
    return 0;
  }
  unsigned int iterations = 0;
  double delta = EPSILON + 1.0f;

  while (delta > EPSILON) {
    delta = 0.0f;
    for (unsigned int i = 0; i < m[0]; i++) {
      for (unsigned int j = 0; j < m[1]; j++) {
        // If this is assigned a value, then skip it.
        if (locked[i * m[1] + j]) {
          continue;
        }
        // Compute the offsets, and ensure it does not go out of bounds.
        unsigned int ip = std::min(m[0] - 1, i + 1);
        unsigned int im = std::max(0, (int) i - 1);
        unsigned int jp = std::min(m[1] - 1, j + 1);
        unsigned int jm = std::max(0, (int) j - 1);

        double old = u[i * m[1] + j];

        // Note: By construction of the for-loop, im and jm are actually the next iteration.
        u[i * m[1] + j] += OMEGA * 0.25f
            * (u[ip * m[1] + j] + u[im * m[1] + j] + u[i * m[1] + jp] + u[i * m[1] + jm] - 4.0f * u[i * m[1] + j]);

        // Compute delta, the difference between this iteration and the previous iteration.
        delta = std::max(delta, (double) fabs(u[i * m[1] + j] - old));
      }
    }

    iterations++;
    //printf("iterations (%d) u %f \n", iterations , u[42* m[1] + 42] );

    printf("Iteration %i --- %e\n", iterations, delta);

    if (iterations == max_iterations) {
      break;
    }
  }
  if (iterations != 1) {
    printf("iterations: %d, delta: %20.19f\n", iterations, delta);
  }

  return iterations;
}

int harmonic_sor_2d_cpu(Harmonic *harmonic)
{
    working_cpu_harmonic_sor_2d(harmonic->m, harmonic->u, harmonic->locked, 100000, harmonic->epsilon, harmonic->omega);
return 0;
}
*/

int harmonic_sor_2d_cpu(Harmonic *harmonic)
{
    // Ensure data is valid before we begin.
    if (harmonic == nullptr || harmonic->m == nullptr || harmonic->u == nullptr ||
            harmonic->locked == nullptr || harmonic->epsilon <= 0.0 ||
            harmonic->omega < 1.0 || harmonic->omega >= 2.0) {
        fprintf(stderr, "Error[harmonic_sor_2d_cpu]: %s\n", "Invalid data.");
        return INERTIA_ERROR_INVALID_DATA;
    }

    // Make sure 'information' can at least be propagated throughout the entire grid.
    unsigned int mMax = 0;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        mMax = std::max(mMax, harmonic->m[i]);
    }

    harmonic->currentIteration = 0;

    long double delta = harmonic->epsilon + 1.0;
    while (delta > harmonic->epsilon || harmonic->currentIteration < mMax) {
        delta = 0.0;

        printf("Starting... ");
        fflush(stdout);

        // Iterate over all non-boundary cells and update its value based on a red-black ordering.
        // Thus, for all rows, we either skip by evens or odds in 2-dimensions.
        for (unsigned int x0 = 1; x0 < harmonic->m[0] - 1; x0++) {
            // Determine if this rows starts with a red (even row) or black (odd row) cell, and
            // update the opposite depending on how many iterations there have been.
            unsigned int offset = (unsigned int)((harmonic->currentIteration % 2) != (x0 % 2));

            for (unsigned int x1 = 1 + offset; x1 < harmonic->m[1] - 1; x1 += 2) {
                // If this is locked, then skip it.
                if (harmonic->locked[x0 * harmonic->m[1] + x1]) {
                    continue;
                }

                long double uPrevious = harmonic->u[x0 * harmonic->m[1] + x1];

                // Update the value at this location with the log-sum-exp trick.
                /* Version #1
                harmonic->u[x0 * harmonic->m[1] + x1] = (1.0 - harmonic->omega) *
                                                            harmonic->u[x0 * harmonic->m[1] + x1] +
                                                        (harmonic->omega / 4.0) *
                                                            (harmonic->u[(x0 - 1) * harmonic->m[1] + x1] +
                                                            harmonic->u[(x0 + 1) * harmonic->m[1] + x1] +
                                                            harmonic->u[x0 * harmonic->m[1] + (x1 - 1)] +
                                                            harmonic->u[x0 * harmonic->m[1] + (x1 + 1)]);
                //*/

                //* Version #2
                long double maxVal = FLT_MIN;
                maxVal = std::max(harmonic->u[(x0 - 1) * harmonic->m[1] + x1], harmonic->u[(x0 + 1) * harmonic->m[1] + x1]);
                maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] + (x1 - 1)]);
                maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] + (x1 + 1)]);

                harmonic->u[x0 * harmonic->m[1] + x1] =  maxVal + std::log(
                                                            std::exp(harmonic->u[(x0 - 1) * harmonic->m[1] + x1] - maxVal) +
                                                            std::exp(harmonic->u[(x0 + 1) * harmonic->m[1] + x1] - maxVal) +
                                                            std::exp(harmonic->u[x0 * harmonic->m[1] + (x1 - 1)] - maxVal) +
                                                            std::exp(harmonic->u[x0 * harmonic->m[1] + (x1 + 1)] - maxVal)) -
                                                        std::log(2.0 * harmonic->n);
                //*/

                // Compute the updated delta.
                delta = std::max(delta, (long double)fabs(uPrevious - harmonic->u[x0 * harmonic->m[1] + x1]));
            }
        }

        printf("Iteration %i --- %Le\n", harmonic->currentIteration, delta);
        fflush(stdout);

        harmonic->currentIteration++;
    }

    printf("eplsion = %Le\n", harmonic->epsilon);

    return INERTIA_SUCCESS;
}

//int harmonic_sor_3d_cpu(Harmonic *harmonic);

//int harmonic_sor_4d_cpu(Harmonic *harmonic);


