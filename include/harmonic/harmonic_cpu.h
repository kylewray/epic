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


#ifndef HARMONIC_CPU_H
#define HARMONIC_CPU_H


#include "harmonic.h"

namespace epic {

/**
 *  Execute a Gauss-Seidel harmonic function solver until convergence. Uses the CPU.
 *  Note: There is no 'initialize', 'execute', or 'uninitialize' function for
 *  the CPU version. This is essentially equivalent to 'execute' for the GPU version.
 *  @param  harmonic    The Harmonic object.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_complete_cpu(Harmonic *harmonic);

/**
 *  Perform a single update step of the Gauss-Seidel CPU implementation.
 *  @param  harmonic    The Harmonic object.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_update_cpu(Harmonic *harmonic);

/**
 *  Perform a single update step of the Gauss-Seidel CPU implementation, updating delta,
 *  and return convergence or not.
 *  @param  harmonic    The Harmonic object.
 *  @return Returns zero (EPIC_SUCCESS) upon success, EPIC_SUCCESS_AND_CONVERGED on
 *          convergence, or non-zero otherwise.
 */
extern "C" int harmonic_update_and_check_cpu(Harmonic *harmonic);

};


#endif // HARMONIC_CPU_H

