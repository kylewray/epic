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


#ifndef HARMONIC_H
#define HARMONIC_H


namespace epic {

/**
 *  A structure for a Harmonic object within the inertia library.
 *  @param  n           The number of dimensions.
 *  @param  m           The size of each dimension.
 *  @param  u           The log values; assumes boarder values are locked.
 *  @param  locked      The locked cells in the grid; assumes boarder
 *                      values are locked.
 *  @param  epsilon     The convergence criterion value.
 *  @param  d_m         Device-side pointer of m.
 *  @param  d_u         Device-side pointer of u.
 *  @param  d_locked    Device-side pointer of locked.
 *  @param  d_delta     The delta values (size of numBlocks).
 */
typedef struct Harmonic {
    // Core Variables (User-Defined).
    unsigned int n;
    unsigned int *m;

    float *u;
    unsigned int *locked;

    float epsilon;
    float delta;
    unsigned int numIterationsToStaggerCheck;

    // Computation Variables (Utilized by Processes Only).
    unsigned int currentIteration;

    unsigned int *d_m;
    float *d_u;
    unsigned int *d_locked;

    float *d_delta;
} Harmonic;

};


#endif // HARMONIC_H

