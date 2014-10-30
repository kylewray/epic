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

#ifndef CPU_H
#define CPU_H


// Since it is not C++ 11.
#define nullptr NULL


/**
 * Compute the fixed point of the 2-dimensional harmonic function provided following
 * the Jacobi method. The harmonic function u must be defined such that boundaries or
 * "goal states" (i.e., any fixed value) have the sign bit flipped. All other values
 * will be modified in-place. The process terminates when the maximal change between
 * any state is less than epsilon.
 * @param	m		The number of dimensions.
 * @param	u		The harmonic function (see above).
 * @param	epsilon The termination criterion.
 * @return	Either 0 if there was no error, or 1 if the parameters were invalid.
 */
int cpu_harmonic_jacobi_2d(const unsigned int *m, float **u, float epsilon);

/**
 * Compute the fixed point of the 2-dimensional harmonic function provided following
 * the Gauss-Seidel method. The harmonic function u must be defined such that boundaries
 * or "goal states" (i.e., any fixed value) have the sign bit flipped. All other values
 * will be modified in-place. The process terminates when the maximal change between
 * any state is less than epsilon.
 * @param	m		The number of dimensions.
 * @param	u		The harmonic function (see above).
 * @param	epsilon The termination criterion.
 * @return	Either 0 if there was no error, or 1 if the parameters were invalid.
 */
int cpu_harmonic_gauss_seidel_2d(const unsigned int *m, float **u, float epsilon);

/**
 * Compute the fixed point of the 2-dimensional harmonic function provided following
 * the Successive-Over-Relaxation (SOR) method. The harmonic function u must be defined
 * such that boundaries or "goal states" (i.e., any fixed value) have the sign bit
 * flipped. All other values will be modified in-place. The process terminates when the
 * maximal change between any state is less than epsilon.
 * @param	m		The number of dimensions.
 * @param	u		The harmonic function (see above).
 * @param	epsilon The termination criterion.
 * @param	omega	The relaxation parameter (step-size-like variable) for the SOR method.
 * @return	Either 0 if there was no error, or 1 if the parameters were invalid.
 */
int cpu_harmonic_sor_2d(const unsigned int *m, float **u, float epsilon, float omega);


#endif // CPU_H
