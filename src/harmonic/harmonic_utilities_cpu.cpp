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
#include "harmonic_utilities_cpu.h"
#include "error_codes.h"
#include "constants.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>


namespace epic {

int harmonic_utilities_set_cell_2d_cpu(Harmonic *harmonic, unsigned int x, unsigned int y, int type)
{
    if (harmonic == nullptr || harmonic->m == nullptr ||
            harmonic->u == nullptr || harmonic->locked == nullptr) {
        fprintf(stderr, "Error[harmonic_utilities_set_cell_cpu]: Invalid data.");
        return EPIC_ERROR_INVALID_DATA;
    }

    if (x >= harmonic->m[0] || y >= harmonic->m[1]) {
        fprintf(stderr, "Error[harmonic_utilities_set_cell_cpu]: Providex (x, y) vector is invalid.");
        return EPIC_ERROR_INVALID_LOCATION;
    }

    if (type == EPIC_CELL_TYPE_GOAL) {
        harmonic->u[x * harmonic->m[1] + y] = EPIC_LOG_SPACE_GOAL;
        harmonic->locked[x * harmonic->m[1] + y] = 1;
    } else if (type == EPIC_CELL_TYPE_OBSTACLE) {
        harmonic->u[x * harmonic->m[1] + y] = EPIC_LOG_SPACE_OBSTACLE;
        harmonic->locked[x * harmonic->m[1] + y] = 1;
    } else if (type == EPIC_CELL_TYPE_FREE) {
        harmonic->u[x * harmonic->m[1] + y] = EPIC_LOG_SPACE_FREE;
        harmonic->locked[x * harmonic->m[1] + y] = 0;
    } else {
        fprintf(stderr, "Error[harmonic_utilities_set_cell_cpu]: Type is invalid. No change made.");
        return EPIC_ERROR_INVALID_CELL_TYPE;
    }

    return EPIC_SUCCESS;
}

}; // namespace epic

