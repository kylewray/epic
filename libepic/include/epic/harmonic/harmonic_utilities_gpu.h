/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2015 Kyle Hollins Wray, University of Massachusetts
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


#ifndef HARMONIC_UTILITIES_GPU_H
#define HARMONIC_UTILITIES_GPU_H


#include "harmonic.h"

namespace epic {

/**
 *  Assign a cells to obstacle, goal, or free space in two dimensions on the GPU.
 *  @param  harmonic    The Harmonic object.
 *  @param  numThreads  The number of threads to use, a multiple of 32. Default is 1024.
 *  @param  k           The number of cell locations to update.
 *  @param  v           The k 2-dimensional cell locations: [x1, y1, x2, y2, ..., xk, yk].
 *  @param  types       Sets 0 for goal, 1 for obstacle, and 2 for free (constants.h).
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_utilities_set_cells_2d_gpu(Harmonic *harmonic, unsigned int numThreads,
        unsigned int k, unsigned int *v, unsigned int *types);

};


#endif // HARMONIC_UTILITIES_GPU_H


