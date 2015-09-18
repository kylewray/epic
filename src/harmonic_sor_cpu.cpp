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

#include <stdio.h>
#include <algorithm>


int harmonic_sor_2d_cpu(Harmonic *harmonic)
{
    // Ensure data is valid before we begin.
    if (harmonic == nullptr || harmonic->m == nullptr || harmonic->u == nullptr ||
            harmonic->locked == nullptr || harmonic->epsilon <= 0.0f ||
            harmonic->omega < 1.0f || harmonic->omega >= 2.0f) {
        fprintf(stderr, "Error[harmonic_sor_2d_cpu]: %s\n", "Invalid data.");
        return HARMONIC_ERROR_INVALID_DATA;
    }

    // Make sure 'information' can at least be propagated throughout the entire grid.
    unsigned int mMax = 0;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        mMax = std::max(mMax, harmonic->m[i]);
    }

    harmonic->currentIteration = 0;

    float delta = harmonic->epsilon + 1.0f;
    while (delta > harmonic->epsilon || harmonic->currentIteration < mMax) {
        delta = 0.0f;

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

                // Update the value at this location.
                harmonic->u[x0 * harmonic->m[1] + x1] += 0.25f * (harmonic->u[(x0 - 1) * harmonic->m[1] + x1] +
                                                                  harmonic->u[(x0 + 1) * harmonic->m[1] + x1] +
                                                                  harmonic->u[x0 * harmonic->m[1] + (x1 - 1)] +
                                                                  harmonic->u[x0 * harmonic->m[1] + (x1 + 1)] -
                                                                  4.0f * harmonic->u[x0 * harmonic->m[1] + x1]);

                // Compute the updated delta.
                delta = std::max(delta, std::fabs(uPrevious - harmonic->u[x0 * harmonic->m[1] + x1]));
            }
        }

        harmonic->currentIteration++;
    }

    return HARMONIC_SUCCESS;
}

//int harmonic_sor_3d_cpu(Harmonic *harmonic);

//int harmonic_sor_4d_cpu(Harmonic *harmonic);


