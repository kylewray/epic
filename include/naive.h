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


#ifndef NAIVE_H
#define NAIVE_H


// Since it is not C++ 11.
#define nullptr NULL


/**
 * Allocate and transfer the harmonic function to the device.
 * @param	n			The number of dimensions.
 * @param	m			The n-dimensional array, specifying the dimensions for each dimension.
 * @param	h			The harmonic function as an n-dimensional array.
 * @param	d_m			The resulting memory address for the dimension sizes on the device.
 * @param	d_u			The resulting memory address of the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int harmonic_alloc(unsigned int n, const unsigned int *m, const float *h,
		unsigned int *&d_m, float *&d_u, float *&d_uPrime);

/**
 * Iterate the harmonic function solver which uses the Jacobi method.
 * @param	n			The number of dimensions.
 * @param	m			The n-dimensional array, specifying the dimensions for each dimension.
 * @param	epsilon		The max convergence check.
 * @param	d_m			The memory address for the dimension sizes on the device.
 * @param	d_u			The memory address on the device for the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @param	b			The 3d-block dimension.
 * @param	t			The 3d-thread dimension.
 * @param	stagger		How many iterations to stagger convergence checks.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int harmonic_execute(unsigned int n, const unsigned int *m, float epsilon,
		unsigned int *d_m, float *d_u, float *d_uPrime,
		unsigned int *b, unsigned int *t,
		unsigned int stagger);

/**
 * Obtain a gradient from the device at a particular cell (index).
 * @param	n		The number of dimensions.
 * @param	m		The n-dimensional array, specifying the dimensions for each dimension.
 * @param	d_u		The memory address on the device for the discrete values on the device.
 * @param	u		The entire matrix of values, which is assumed to be created in memory already.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int harmonic_get(unsigned int n, const unsigned int *m, float *d_u, float *u);

/**
 * Obtain a gradient from the device at a particular cell (index).
 * @param	i		The index to grab the value from in u.
 * @param	d_u		The memory address on the device for the discrete values on the device.
 * @param	val		The value at the index in u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
//int harmonic_get(const unsigned long long int i, float *d_u, ???);

/**
 * Obtain a gradient from the device at a particular cell (coordinate).
 * @param	n		The number of dimensions.
 * @param	m		The n-dimensional array, specifying the dimensions for each dimension.
 * @param	d_u		The memory address on the device for the discrete values on the device.
 * @param	c		The coordinate to retrieve in u.
 * @param	val		The value at the index in u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
//int harmonic_get(unsigned int n, const unsigned int *m, float *d_u, const unsigned int *c, ???);

/**
 * Free the harmonic function and the discrete values from the device.
 * @param	d_m			The memory address on the device for the dimension sizes.
 * @param	d_u			The memory address on the device for the discrete values on the device.
 * @prarm	d_uPrime	The memory address on the device for the extra iteration-focused copy of d_u.
 * @return	Return 0 if no error, 1 if an error occurred.
 */
int harmonic_free(unsigned int *d_m, float *d_u, float *d_uPrime);


#endif // NAIVE_H
