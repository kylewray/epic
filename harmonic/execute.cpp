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
//#include <ctime>
#include <sys/time.h>

#include "include/worlds.h"
#include "include/naive.h"
#include "include/cpu.h"
#include "include/gpu.h"

long long get_current_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000LL + tv.tv_usec / 1000;
}

int trials_2d(unsigned int numThreads, float epsilon,
		unsigned int minSize, unsigned int maxSize, unsigned int stepSize,
		unsigned int numObstacles)
{
	std::cout << "N (Size)\t,\tCPU Time (s)\t,\tGPU Time (s)" << std::endl;

	for (unsigned int size = minSize; size <= maxSize; size += stepSize) {
		unsigned int maxObstacleSize = size / 2;

		std::cout << size << "\t,\t";
		std::cout.flush();

		// Create the world ouside of timing.
		unsigned int *cpu_m = nullptr;
		float **cpu_u = nullptr;
		create_variable_world_2d(size, size, cpu_m, cpu_u, numObstacles, maxObstacleSize);

//		std::clock_t start = std::clock();
		long long start = get_current_time();

		cpu_harmonic_sor_2d(cpu_m, cpu_u, epsilon, 1.5f);

//		std::cout << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000.0) << "\t\t";
		std::cout << (double)(get_current_time() - start) / 1000.0 << "\t,\t";
		std::cout.flush();

		// Cleanup the world.
		for (int i = 0; i < cpu_m[0]; i++) {
			delete [] cpu_u[i];
		}
		delete [] cpu_u;
		delete [] cpu_m;

		// Create the world again outside of timing.
		unsigned int *gpu_m = nullptr;
		float *gpu_u = nullptr;
		create_variable_world_1d(size, size, gpu_m, gpu_u, numObstacles, maxObstacleSize);

//		start = std::clock();
		start = get_current_time();

		unsigned int *d_m;
		float *d_u;
		float *d_uPrime;

		if (gpu_harmonic_alloc_2d(gpu_m, gpu_u, d_m, d_u, d_uPrime) != 0) {
			return 1;
		}
		if (gpu_harmonic_execute_2d(gpu_m, epsilon, d_m, d_u, d_uPrime, numThreads) != 0) {
			return 1;
		}

//		std::cout << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000.0) << "\t\t";
		std::cout << (double)(get_current_time() - start) / 1000.0 << "\t,\t";
		std::cout.flush();

		// Cleanup the world.
		if (gpu_harmonic_free_2d(d_m, d_u, d_uPrime) != 0) {
			return 1;
		}
		delete [] gpu_u;
		delete [] gpu_m;

		std::cout << std::endl;
	}

	return 0;
}

int single_trial_2d()
{
	unsigned int version = 1; // 0 = CPU, 1 = GPU
	unsigned int cpuVariant = 2; // 0 = Jacobi, 1 = Gauss-Seidel, 2 = SOR
	unsigned int numThreads = 512;

	float epsilon = 0.0001f;

	unsigned int size = 512;
	unsigned int numObstacles = 10;
	unsigned int maxObstacleSize = 50;

	bool printResult = true;

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
			cpu_harmonic_gauss_seidel_2d(m, u, epsilon);
		} else if (cpuVariant == 2) {
			cpu_harmonic_sor_2d(m, u, epsilon, 1.5f);
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

int main(int argc, char *argv[])
{
//	single_trial_2d();

	trials_2d(128, 0.0001f, 128, 128, 32, 10);
	trials_2d(256, 0.0001f, 256, 256, 32, 10);
	trials_2d(512, 0.0001f, 512, 512, 32, 10);
//	trials_2d(512, 0.0001f, 1024, 1024, 32, 10);

	return 0;
}
