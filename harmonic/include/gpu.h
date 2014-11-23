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

#ifndef GPU_H
#define GPU_H


// Since it is not C++ 11.
#define nullptr NULL


/**
 * Allocate and transfer the 2-dimensional harmonic function to the device. (Used for the Jacobi method.)
 * @param	m			The 2-dimensional array, specifying the size of each dimension.
 * @param	u			The harmonic function as an array following the dimensions of m.
 * @param	d_m			The resulting memory address for the dimension sizes on the device.
 * @param	d_u			The resulting memory address of the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_alloc_2d(unsigned int *m, float *u,
		unsigned int *&d_m, float *&d_u, float *&d_uPrime);

/**
 * Iterate the harmonic function solver which uses the Jacobi method.
 * @param	m			The 2-dimensional array, specifying the dimensions for each dimension.
 * @param	epsilon		The max convergence check.
 * @param	d_m			The memory address for the dimension sizes on the device.
 * @param	d_u			The memory address on the device for the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @param	numThreads	The number of threads to run per block.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_execute_2d(unsigned int *m, float epsilon,
		unsigned int *d_m, float *d_u, float *d_uPrime,
		unsigned int numThreads);

/**
 * Obtain a gradient from the device at a particular cell (index). (Used for the Jacobi method.)
 * @param	m		The 2-dimensional array, specifying the dimensions for each dimension.
 * @param	d_u		The memory address on the device for the discrete values on the device.
 * @param	u		The entire matrix of values, which is assumed to be created in memory already.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_get_2d(unsigned int *m, float *d_u, float *u);

/**
 * Free the harmonic function and the discrete values from the device. (Used for the Jacobi method.)
 * @param	d_m			The memory address on the device for the dimension sizes.
 * @param	d_u			The memory address on the device for the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_free_2d(unsigned int *d_m, float *d_u, float *d_uPrime);

/**
 * Allocate and transfer the 3-dimensional harmonic function to the device. (Used for the Jacobi method.)
 * @param	m			The 3-dimensional array, specifying the size of each dimension.
 * @param	u			The harmonic function as an array following the dimensions of m.
 * @param	d_m			The resulting memory address for the dimension sizes on the device.
 * @param	d_u			The resulting memory address of the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_alloc_3d(unsigned int *m, float *u,
		unsigned int *&d_m, float *&d_u, float *&d_uPrime);

/**
 * Iterate the harmonic function solver which uses the Jacobi method.
 * @param	m			The 3-dimensional array, specifying the dimensions for each dimension.
 * @param	epsilon		The max convergence check.
 * @param	d_m			The memory address for the dimension sizes on the device.
 * @param	d_u			The memory address on the device for the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @param	numThreads	The number of threads to run per block.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_execute_3d(unsigned int *m, float epsilon,
		unsigned int *d_m, float *d_u, float *d_uPrime,
		unsigned int numThreads);

/**
 * Obtain a gradient from the device at a particular cell (index). (Used for the Jacobi method.)
 * @param	m		The 3-dimensional array, specifying the dimensions for each dimension.
 * @param	d_u		The memory address on the device for the discrete values on the device.
 * @param	u		The entire matrix of values, which is assumed to be created in memory already.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_get_3d(unsigned int *m, float *d_u, float *u);

/**
 * Free the harmonic function and the discrete values from the device. (Used for the Jacobi method.)
 * @param	d_m			The memory address on the device for the dimension sizes.
 * @param	d_u			The memory address on the device for the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int gpu_harmonic_free_3d(unsigned int *d_m, float *d_u, float *d_uPrime);


#endif // GPU_H
