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


#ifndef HARMONIC_LEGACY_CPU_H
#define HARMONIC_LEGACY_CPU_H


namespace epic {

/**
 *  Execute a Successive Over-Relaxation (SOR) harmonic function solver until convergence.
 *  Uses the CPU. Important: This legacy code assumes non-log-space for u!
 *  @param  w       The width (x-axis).
 *  @param  h       The height (y-axis).
 *  @param  epsilon The desired precision. Default is 0.1.
 *  @param  omega   The relaxation parameter in [0, 2]. Default is 1.5.
 *  @param  locked  The locked (xy array): 1 for locked, 0 for unlocked.
 *  @param  u       The potentials (xy array): 0 for goal, 1 for obstacle, and in range (0, 1)
 *                  otherwise. Stored in row-major form (e.g., y * w + x). This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_legacy_sor_2d_float_cpu(unsigned int w, unsigned int h, float epsilon,
        float omega, unsigned int *locked, float *u);

/**
 *  Execute a Successive Over-Relaxation (SOR) harmonic function solver until convergence.
 *  Uses the CPU. Important: This legacy code assumes non-log-space for u!
 *  @param  w       The width (x-axis).
 *  @param  h       The height (y-axis).
 *  @param  epsilon The desired precision. Default is 0.1.
 *  @param  omega   The relaxation parameter in [0, 2]. Default is 1.5.
 *  @param  locked  The locked (xy array): 1 for locked, 0 for unlocked.
 *  @param  u       The potentials (xy array): 0 for goal, 1 for obstacle, and in range (0, 1)
 *                  otherwise. Stored in row-major form (e.g., y * w + x). This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_legacy_sor_2d_double_cpu(unsigned int w, unsigned int h, double epsilon,
        double omega, unsigned int *locked, double *u);

/**
 *  Execute a Successive Over-Relaxation (SOR) harmonic function solver until convergence.
 *  Uses the CPU. Important: This legacy code assumes non-log-space for u!
 *  @param  w       The width (x-axis).
 *  @param  h       The height (y-axis).
 *  @param  epsilon The desired precision. Default is 0.1.
 *  @param  omega   The relaxation parameter in [0, 2]. Default is 1.5.
 *  @param  locked  The locked (xy array): 1 for locked, 0 for unlocked.
 *  @param  u       The potentials (xy array): 0 for goal, 1 for obstacle, and in range (0, 1)
 *                  otherwise. Stored in row-major form (e.g., y * w + x). This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_legacy_sor_2d_long_double_cpu(unsigned int w, unsigned int h, long double epsilon,
        long double omega, unsigned int *locked, long double *u);

};


#endif // HARMONIC_LEGACY_CPU_H

