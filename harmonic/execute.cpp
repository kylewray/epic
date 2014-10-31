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


#include <iostream>
#include <cstdlib>
#include <ctime>

#include "include/worlds.h"
#include "include/naive.h"
#include "include/cpu.h"
#include "include/gpu.h"

int main(int argc, char *argv[])
{
	unsigned int version = 1; // 0 = CPU, 1 = GPU
	unsigned int cpuVariant = 2; // 0 = Jacobi, 1 = Gauss-Seidel, 2 = SOR
	unsigned int numThreads = 256;

	float epsilon = 0.0001f;

	unsigned int size = 256;
	unsigned int numObstacles = 100;
	unsigned int maxObstacleSize = 100;

	bool printResult = false;

	srand(1);

	if (version == 0) { // CPU
		unsigned int *m = nullptr;
		float **u = nullptr;

		/* ----- Simple World -----
		create_simple_world_2d(m, u);
		//*/

		//* ----- Variable World -----
		// Note: Only works with equal dimension sizes!!! Bug with index math somewhere...
		create_variable_world_2d(size, size, m, u, numObstacles, maxObstacleSize);
		//*/

		// ***** START TIMER *****
		time_t start = time(0);

		// Solve it and print it out.
		if (cpuVariant == 0) {
			cpu_harmonic_jacobi_2d(m, u, epsilon);
		} else if (cpuVariant == 1) {
			cpu_harmonic_gauss_seidel_2d(m, u, 0.001f);
		} else if (cpuVariant == 2) {
			cpu_harmonic_sor_2d(m, u, 0.001f, 1.5f);
		}

		// ***** STOP TIMER *****
		time_t stop = time(0);
		double elapsed = difftime(stop, start) * 1000.0;
		std:: cout << "Elapsed Time (in seconds): " << elapsed << std::endl;

		if (printResult) {
			print_world_2d(m, u);
		}

		// Release everything.
		for (int i = 0; i < m[0]; i++) {
			delete [] u[i];
		}
		delete [] u;
		delete [] m;
	} else if (version == 1) { // CUDA
		unsigned int *m = nullptr;
		float *u = nullptr;

		/* ----- Simple World -----
		create_simple_world_1d(m, u);
		//*/

		//* ----- Variable World -----
		// Performance Note: Ensure rows and cols are divisible by 32 or 64 to guarantee 4 byte word
		// alignment (since they are floats and don't require padding).
		create_variable_world_1d(size, size, m, u, numObstacles, maxObstacleSize);
		//*/

		// ***** START TIMER *****
		time_t start = time(0);

		unsigned int *d_m;
		float *d_u;
		float *d_uPrime;

		// Allocate and execute Jacobi iteration on the GPU (naive version).
		if (gpu_harmonic_alloc_2d(m, u, d_m, d_u, d_uPrime) != 0) {
			return 1;
		}
		if (gpu_harmonic_execute_2d(m, epsilon, d_m, d_u, d_uPrime, numThreads) != 0) {
			return 1;
		}

		// ***** STOP TIMER *****
		time_t stop = time(0);
		double elapsed = difftime(stop, start) * 1000.0;
		std:: cout << "Elapsed Time (in seconds): " << elapsed << std::endl;

		// Get the world from the GPU and print it.
		if (printResult) {
			gpu_harmonic_get_2d(m, d_u, u);
			print_world_1d(m, u);
		}

		// Release everything.
		if (gpu_harmonic_free_2d(d_m, d_u, d_uPrime) != 0) {
			return 1;
		}
		delete [] u;
		delete [] m;
	}

	return 0;
}
