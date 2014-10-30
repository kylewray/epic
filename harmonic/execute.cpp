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


#include <stdio.h>
#include <cstdlib>

#include "include/worlds.h"
#include "include/naive.h"
#include "include/cpu.h"
#include "include/gpu.h"

int main(int argc, char *argv[])
{
	unsigned int version = 1;

	srand(1);

	if (version == 0) { // CUDA
//		unsigned int *m = nullptr;
//		float *h = nullptr;
//
//		//* ----- Simple World -----
//		create_simple_world_1d(m, h);
//		//*/
//
//		/* ----- Variable World -----
//		// Note: Only works with equal dimension sizes!!! Bug with index math somewhere...
//		create_variable_world_1d(64, 64, m, h, 10, 10);
//		//*/
//
//		unsigned int *d_m;
//		float *d_u;
//		float *d_uPrime;
//
//		// Setup the number of blocks and threads so that we have enough to execute all the threads.
//		unsigned int b[3];
//		b[0] = 6; b[1] = 6; b[2] = 4; // Equivalent to 3 * 3 * 3 = 18 blocks.
//		unsigned int t[3];
//		t[0] = 8; t[1] = 8; t[2] = 8; // Equivalent to 8 * 8 * 8 = 512 threads.
//
//		// Allocate and execute Jacobi iteration on the GPU (naive version).
//		if (harmonic_alloc(2, m, h, d_m, d_u, d_uPrime) != 0) {
//			return 1;
//		}
//		if (harmonic_execute(2, m, 0.01f, d_m, d_u, d_uPrime, b, t, 25) != 0) {
//			return 1;
//		}
//
//		// Get the world from the GPU and print it.
//		float *u = new float[m[0] * m[1]];
//
//		harmonic_get(2, m, d_u, u);
//		print_world_1d(m, u);
//
//		// Release everything.
//		if (harmonic_free(d_m, d_u, d_uPrime) != 0) {
//			return 1;
//		}
//		delete [] u;
//		delete [] m;
//		delete [] h;
	} else if (version == 1) { // CPU
		unsigned int *m = nullptr;
		float **h = nullptr;

		/* ----- Simple World -----
		create_simple_world_2d(m, h);
		//*/

		//* ----- Variable World -----
		// Note: Only works with equal dimension sizes!!! Bug with index math somewhere...
		create_variable_world_2d(256, 32, m, h, 10, 15);
		//*/

		// Solve it and print it out.
		cpu_harmonic_jacobi_2d(m, h, 0.001f);
//		cpu_harmonic_gauss_seidel_2d(m, h, 0.001f);
//		cpu_harmonic_sor_2d(m, h, 0.001f, 1.5f);
		print_world_2d(m, h);

		// Release everything.
		for (int i = 0; i < m[0]; i++) {
			delete [] h[i];
		}
		delete [] h;
		delete [] m;
	}

	return 0;
}
