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


#include "trials.h"

int trials_2d(unsigned int numThreads, float epsilon,
		unsigned int minSize, unsigned int maxSize, unsigned int stepSize,
		unsigned int numObstacles)
{
	std::cout << "N (Size),CPU Time (s),GPU Time (s)" << std::endl;

	for (unsigned int size = minSize; size <= maxSize; size += stepSize) {
		unsigned int maxObstacleSize = size / 2;

		std::cout << size << ",";
		std::cout.flush();

		unsigned int *m = new unsigned int[2];
		m[0] = size;
		m[1] = size;

		// Create the world outside of timing.
		float *cpu_u = nullptr;
		create_variable_world_2d(m, cpu_u, numObstacles, maxObstacleSize);

//		std::clock_t start = std::clock();
		long long start = get_current_time();

		cpu_harmonic_sor_2d(m, cpu_u, epsilon, 1.5f);

//		std::cout << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000.0) << "\t\t";
		std::cout << (double)(get_current_time() - start) / 1000.0 << ",";
		std::cout.flush();

		// Cleanup the world.
		delete [] cpu_u;

		// Create the world again outside of timing.
		float *gpu_u = nullptr;
		create_variable_world_2d(m, gpu_u, numObstacles, maxObstacleSize);

//		start = std::clock();
		start = get_current_time();

		unsigned int *d_m;
		float *d_u;
		float *d_uPrime;

		if (gpu_harmonic_alloc_2d(m, gpu_u, d_m, d_u, d_uPrime) != 0) {
			return 1;
		}
		if (gpu_harmonic_execute_2d(m, epsilon, d_m, d_u, d_uPrime, numThreads) != 0) {
			return 1;
		}

//		std::cout << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000.0) << "\t\t";
		std::cout << (double)(get_current_time() - start) / 1000.0 << ",";
		std::cout.flush();

		// Cleanup the world.
		if (gpu_harmonic_free_2d(d_m, d_u, d_uPrime) != 0) {
			return 1;
		}
		delete [] gpu_u;
		delete [] m;

		std::cout << std::endl;
	}

	return 0;
}

int single_trial_2d()
{
	unsigned int version = 0; // 0 = CPU, 1 = GPU
	unsigned int cpuVariant = 2; // 0 = Jacobi, 1 = Gauss-Seidel, 2 = SOR
	unsigned int numThreads = 512;

	float epsilon = 0.0001f;

	unsigned int size = 256;
	unsigned int numObstacles = 10;
	unsigned int maxObstacleSize = 50;

	bool printResult = true;

	srand(1);

	if (version == 0) { // CPU
		unsigned int *m = nullptr;
		float *u = nullptr;

		//* ----- Simple World -----
		create_simple_world_2d(m, u);
		//*/

		/* ----- Variable World -----
		m = new unsigned int[2];
		m[0] = size;
		m[1] = size;
		create_variable_world_2d(m, u, numObstacles, maxObstacleSize);
		//*/

		long long start = get_current_time();

		// Solve it and print it out.
		if (cpuVariant == 0) {
			cpu_harmonic_jacobi_2d(m, u, epsilon);
		} else if (cpuVariant == 1) {
			cpu_harmonic_gauss_seidel_2d(m, u, epsilon);
		} else if (cpuVariant == 2) {
			cpu_harmonic_sor_2d(m, u, epsilon, 1.5f);
		}

		std::cout << "Elapsed Time (in seconds): " << (double)(get_current_time() - start) / 1000.0 << std::endl;

		if (printResult) {
			print_world_2d(m, u);
		}

		// Release everything.
		delete [] u;
		delete [] m;
	} else if (version == 1) { // CUDA
		unsigned int *m = nullptr;

		float *u = nullptr;

		//* ----- Simple World -----
		create_simple_world_2d(m, u);
		//*/

		/* ----- Variable World -----
		m = new unsigned int[2];
		m[0] = size;
		m[1] = size;
		create_variable_world_2d(m, u, numObstacles, maxObstacleSize);
		//*/

		long long start = get_current_time();

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

		std::cout << "Elapsed Time (in seconds): " << (double)(get_current_time() - start) / 1000.0 << std::endl;

		// Get the world from the GPU and print it.
		if (printResult) {
			gpu_harmonic_get_2d(m, d_u, u);
			print_world_2d(m, u);
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
