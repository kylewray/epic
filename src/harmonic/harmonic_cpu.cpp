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
#include "harmonic_cpu.h"
#include "error_codes.h"
#include "constants.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>


void harmonic_update_2d_cpu(Harmonic *harmonic)
{
    harmonic->delta = 0.0;

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

            float uPrevious = harmonic->u[x0 * harmonic->m[1] + x1];

            // Update the value at this location with the log-sum-exp trick.
            float maxVal = FLT_MIN;
            maxVal = std::max(harmonic->u[(x0 - 1) * harmonic->m[1] + x1], harmonic->u[(x0 + 1) * harmonic->m[1] + x1]);
            maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] + (x1 - 1)]);
            maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] + (x1 + 1)]);

            harmonic->u[x0 * harmonic->m[1] + x1] =  maxVal + std::log(
                                                        std::exp(harmonic->u[(x0 - 1) * harmonic->m[1] + x1] - maxVal) +
                                                        std::exp(harmonic->u[(x0 + 1) * harmonic->m[1] + x1] - maxVal) +
                                                        std::exp(harmonic->u[x0 * harmonic->m[1] + (x1 - 1)] - maxVal) +
                                                        std::exp(harmonic->u[x0 * harmonic->m[1] + (x1 + 1)] - maxVal)) -
                                                    std::log(2.0 * harmonic->n);

            // Compute the updated delta.
            harmonic->delta = std::max(harmonic->delta, (float)fabs(uPrevious - harmonic->u[x0 * harmonic->m[1] + x1]));
        }
    }
}


void harmonic_update_3d_cpu(Harmonic *harmonic)
{
    harmonic->delta = 0.0;

    // Iterate over all non-boundary cells and update its value based on a red-black ordering.
    // Thus, for all rows, we either skip by evens or odds in 2-dimensions.
    for (unsigned int x0 = 1; x0 < harmonic->m[0] - 1; x0++) {
        for (unsigned int x1 = 1; x1 < harmonic->m[1] - 1; x1++) {
            // Determine if this rows starts with a red (even row) or black (odd row) cell, and
            // update the opposite depending on how many iterations there have been.
            unsigned int offset = (unsigned int)((harmonic->currentIteration % 2) != (x0 % 2));

            // We also negate the offset based on this dimension.
            if (x1 % 2 == 0) {
                offset = (unsigned int)(!(bool)offset);
            }

            for (unsigned int x2 = 1 + offset; x2 < harmonic->m[2] - 1; x2 += 2) {
                // If this is locked, then skip it.
                if (harmonic->locked[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2]) {
                    continue;
                }

                float uPrevious = harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2];

                // Update the value at this location with the log-sum-exp trick.
                float maxVal = FLT_MIN;
                maxVal = std::max(harmonic->u[(x0 - 1) * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2],
                                harmonic->u[(x0 + 1) * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2]);
                maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + (x1 - 1) * harmonic->m[2] + x2]);
                maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + (x1 + 1) * harmonic->m[2] + x2]);
                maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + (x2 - 1)]);
                maxVal = std::max(maxVal, harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + (x2 + 1)]);

                harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2] =  maxVal + std::log(
                                                            std::exp(harmonic->u[(x0 - 1) * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2] - maxVal) +
                                                            std::exp(harmonic->u[(x0 + 1) * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2] - maxVal) +
                                                            std::exp(harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + (x1 - 1) * harmonic->m[2] + x2] - maxVal) +
                                                            std::exp(harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + (x1 + 1) * harmonic->m[2] + x2] - maxVal) +
                                                            std::exp(harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + (x2 - 1)] - maxVal) +
                                                            std::exp(harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + (x2 + 1)] - maxVal)) -
                                                        std::log(2.0 * harmonic->n);

                // Compute the updated delta.
                harmonic->delta = std::max(harmonic->delta, (float)fabs(uPrevious - harmonic->u[x0 * harmonic->m[1] * harmonic->m[2] + x1 * harmonic->m[2] + x2]));
            }
        }
    }
}


int harmonic_complete_cpu(Harmonic *harmonic)
{
    int result;

    // Ensure data is valid before we begin.
    if (harmonic == nullptr || harmonic->m == nullptr || harmonic->u == nullptr ||
            harmonic->locked == nullptr || harmonic->epsilon <= 0.0) {
        fprintf(stderr, "Error[harmonic_complete_cpu]: %s\n", "Invalid data.");
        return EPIC_ERROR_INVALID_DATA;
    }

    // Make sure 'information' can at least be propagated throughout the entire grid.
    unsigned int mMax = 0;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        mMax = std::max(mMax, harmonic->m[i]);
    }

    harmonic->currentIteration = 0;
    harmonic->delta = harmonic->epsilon + 1.0;

    result = EPIC_SUCCESS;

    while (result != EPIC_SUCCESS_AND_CONVERGED || harmonic->currentIteration < mMax) {
        result = harmonic_update_cpu(harmonic);
        if (result != EPIC_SUCCESS && result != EPIC_SUCCESS_AND_CONVERGED) {
            fprintf(stderr, "Error[harmonic_complete_cpu]: %s\n",
                            "Failed to perform the Gauss-Seidel update (and check) step.");
            return result;
        }

        /* *** DEBUG ***
        if (harmonic->currentIteration % 100 == 0) {
            printf("Iteration %i --- %e\n", harmonic->currentIteration, delta);
            fflush(stdout);
        }
        //*/
    }

    return EPIC_SUCCESS;
}


int harmonic_update_cpu(Harmonic *harmonic)
{
    if (harmonic->n == 2) {
        harmonic_update_2d_cpu(harmonic);
        harmonic_update_3d_cpu(harmonic);
    } else if (harmonic->n == 3) {
    } else if (harmonic->n == 4) {
    }

    harmonic->currentIteration++;

    if (harmonic->delta < harmonic->epsilon) {
        return EPIC_SUCCESS_AND_CONVERGED;
    } else {
        return EPIC_SUCCESS;
    }
}
