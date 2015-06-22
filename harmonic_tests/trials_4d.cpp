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

#include "../../harmonic/include/gpu_utils.h"

int trials_4d(unsigned int numBlocksX, unsigned int numBlocksY, unsigned int numBlocksZ, unsigned int numThreads,
		unsigned int stagger, float epsilon, unsigned int minSize, unsigned int maxSize, unsigned int stepSize,
		unsigned int numObstacles, unsigned int numExecutions)
{
	unsigned int numThreadsEpic = 1024;

	std::cout << "N,CPU Mean,CPU Stdev,CPU 95% CI,GPU v1 Mean,GPU v1 Stdev,GPU v1 95% CI,GPU v2 Mean,GPU v2 Stdev,GPU v2 95% CI,GPU v3 d,GPU v3 dNew,GPU v3 Mean,GPU v3 Stdev,GPU v3 95% CI" << std::endl;

	for (unsigned int size = minSize; size <= maxSize; size += stepSize) {
		std::vector<double> times;

		unsigned int maxObstacleSize = size / 2;

		std::cout << size << ",";
		std::cout.flush();

		unsigned int *m = new unsigned int[4];
		m[0] = size;
		m[1] = size;
		m[2] = size;
		m[3] = size;

		times.clear();

		for (unsigned int k = 0; k < numExecutions; k++) {
			// Create the world outside of timing.
			float *cpu_u = nullptr;
			create_variable_world_4d(m, cpu_u, numObstacles, maxObstacleSize);

			long long start = get_current_time();

			cpu_harmonic_sor_4d(m, cpu_u, epsilon, 1.5f);

			times.push_back((double)(get_current_time() - start) / 1000.0);

			// Cleanup the world.
			delete [] cpu_u;
		}

		std::cout << compute_statistics(times) << ",";
		std::cout.flush();

		times.clear();

		for (unsigned int k = 0; k < numExecutions; k++) {
			// Create the world again outside of timing.
			float *gpu_u = nullptr;
			create_variable_world_4d(m, gpu_u, numObstacles, maxObstacleSize);

			unsigned int *d_m;
			float *d_u;
			float *d_uPrime;

			if (gpu_jacobi_v1_alloc_4d(m, gpu_u, d_m, d_u, d_uPrime) != 0) {
				return 1;
			}

			long long start = get_current_time();

			if (gpu_jacobi_v1_execute_4d(m, epsilon, d_m, d_u, d_uPrime, numBlocksX, numBlocksY, numBlocksZ, numThreads, stagger) != 0) {
				return 1;
			}

			times.push_back((double)(get_current_time() - start) / 1000.0);

			// Cleanup the world.
			if (gpu_jacobi_v1_free_4d(d_m, d_u, d_uPrime) != 0) {
				return 1;
			}

			delete [] gpu_u;
		}

		std::cout << compute_statistics(times) << ",";
		std::cout.flush();

		times.clear();

		for (unsigned int k = 0; k < numExecutions; k++) {
			// Create the world again outside of timing.
			float *gpu_u = nullptr;
			create_variable_world_4d(m, gpu_u, numObstacles, maxObstacleSize);

			int *d_index;
			bool *d_locked;
			float *d_u;
			float *d_uPrime;

			unsigned int n = 4;
			unsigned int d = m[0] * m[1] * m[2] * m[3];

			if (gpu_jacobi_v2_alloc(n, d, m, gpu_u, d_index, d_locked, d_u, d_uPrime) != 0) {
				return 1;
			}

			long long start = get_current_time();

			if (gpu_jacobi_v2_execute(n, d, epsilon, d_index, d_locked, d_u, d_uPrime, numThreadsEpic, stagger) != 0) {
				return 1;
			}

			times.push_back((double)(get_current_time() - start) / 1000.0);

			// Cleanup the world.
			if (gpu_jacobi_v2_free(d_index, d_locked, d_u, d_uPrime) != 0) {
				return 1;
			}

			delete [] gpu_u;
		}

		std::cout << compute_statistics(times) << ",";
		std::cout.flush();

		times.clear();

		float dNewAverage = 0.0f;
		for (unsigned int k = 0; k < numExecutions; k++) {
			// Create the world again outside of timing.
			float *gpu_u = nullptr;
			create_variable_world_4d(m, gpu_u, numObstacles, maxObstacleSize);

			unsigned int dNew;
			unsigned int *cellIndexActualToAdjusted;
			unsigned int *cellIndexAdjustedToActual;
			int *d_index;
			bool *d_locked;
			float *d_u;
			float *d_uPrime;

			unsigned int n = 4;
			unsigned int d = m[0] * m[1] * m[2] * m[3];

			if (gpu_jacobi_alloc(n, d, m, cellIndexActualToAdjusted, cellIndexAdjustedToActual, gpu_u,
					dNew, d_index, d_locked, d_u, d_uPrime) != 0) {
				return 1;
			}

			long long start = get_current_time();

			if (gpu_jacobi_execute(n, dNew, epsilon, d_index, d_locked, d_u, d_uPrime, numThreadsEpic, stagger) != 0) {
				return 1;
			}

			times.push_back((double)(get_current_time() - start) / 1000.0);

			// Cleanup the world.
			if (gpu_jacobi_free(cellIndexActualToAdjusted, cellIndexAdjustedToActual, d_index, d_locked, d_u, d_uPrime) != 0) {
				return 1;
			}

			delete [] gpu_u;

			dNewAverage = (float)(k * dNewAverage + dNew) / (float)(k + 1);
		}
		std::cout << (m[0] * m[1] * m[2] * m[3]) << "," << dNewAverage << ",";

		std::cout << compute_statistics(times) << std::endl;
		std::cout.flush();

		times.clear();

		delete [] m;
	}

	return 0;
}
