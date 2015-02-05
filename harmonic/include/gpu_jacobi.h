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

#ifndef GPU_JACOBI_H
#define GPU_JACOBI_H


// Since it is not C++ 11.
#define nullptr NULL


// Note: We only use "unsigned int" for indices. The assumption is that we will never have more
// than the max value of an unsigned int: 4,294,967,295, number of states.


/**
 * Allocate and transfer the n-dimensional harmonic function to the device. (Used for the Jacobi method.)
 * @param	n			The number of dimensions.
 * @param	m			The n-dimensional array, specifying the size of each dimension.
 * @param	u			The harmonic function as an array following the dimensions of m.
 * @param	d_row		The resulting memory address for the row indices within d_u (and d_uPrime)
 * 						which point to which cells must be updated once the value is computed.
 * @param	d_col		The resulting memory address for the row indices within d_u (and d_uPrime)
 * 						which point to which cells must be updated once the value is computed.
 * @param	d_locked	If each cell was locked or not (m-array).
 * @param	d_u			The resulting memory address of the discrete values on the device. Stored as
 * 						an n by m dimensional matrix. Each column represents a particular
 * 						pixel, with each row being the value of one if the neighbors. We'll use d_row
 * 						and d_col to figure out all of the values that need to be updated after the
 * 						value is computed.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_jacobi_alloc(unsigned int n, unsigned int m, float *u,
		unsigned int *&d_row, unsigned int *&d_col, bool *&d_locked, float *&d_u, float *&d_uPrime);

/**
 * Iterate the harmonic function solver which uses the Jacobi method.
 * @param	n			The number of dimensions.
 * @param	m			The total number of cells.
 * @param	epsilon		The max convergence check.
 * @param	d_row		The resulting memory address for the row indices within d_u (and d_uPrime)
 * 						which point to which cells must be updated once the value is computed.
 * @param	d_col		The resulting memory address for the row indices within d_u (and d_uPrime)
 * 						which point to which cells must be updated once the value is computed.
 * @param	d_locked	If each cell was locked or not (m-array).
 * @param	d_u			The resulting memory address of the discrete values on the device. Stored as
 * 						an n by m dimensional matrix. Each column represents a particular
 * 						pixel, with each row being the value of one if the neighbors. We'll use d_row
 * 						and d_col to figure out all of the values that need to be updated after the
 * 						value is computed.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @param	numBlocks	The number of blocks to execute.
 * @param	numThreads	The number of threads to run per block.
 * @param	stagger		The iteration stagger step to check for convergence.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_jacobi_execute(unsigned int n, unsigned int m, float epsilon,
		unsigned int *d_row, unsigned int *d_col, bool *d_locked, float *d_u, float *d_uPrime,
		unsigned int numBlocks, unsigned int numThreads,
		unsigned int stagger);

/**
 * Free the harmonic function and the discrete values from the device. (Used for the Jacobi method.
 * @param	d_row		The resulting memory address for the row indices within d_u (and d_uPrime)
 * 						which point to which cells must be updated once the value is computed.
 * @param	d_col		The resulting memory address for the row indices within d_u (and d_uPrime)
 * 						which point to which cells must be updated once the value is computed.
 * @param	d_locked	If each cell was locked or not (m-array).
 * @param	d_u			The resulting memory address of the discrete values on the device. Stored as
 * 						an n by m dimensional matrix. Each column represents a particular
 * 						pixel, with each row being the value of one if the neighbors. We'll use d_row
 * 						and d_col to figure out all of the values that need to be updated after the
 * 						value is computed.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_free(unsigned int *d_row, unsigned int *d_col, bool *d_locked, float *d_u, float *d_uPrime);


#endif // GPU_JACOBI_H
