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

#include "../include/gpu.h"

__global__ void gpu_harmonic_iteration_2d(unsigned int *m, float *u, float *uPrime, float epsilon, unsigned long long int *running)
{
	unsigned int i = blockIdx.x % m[0];
	unsigned int j = threadIdx.x + (unsigned int)(blockIdx.x / m[0]) * blockDim.x;

	if (i >= m[0] || j >= m[1] || signbit(u[i * m[1] + j]) != 0) {
		return;
	}

	unsigned int ip = min(m[0] - 1, i + 1);
	unsigned int im = max(0, (int)i - 1);
	unsigned int jp = min(m[1] - 1, j + 1);
	unsigned int jm = max(0, (int)j - 1);

	float val = 0.25f *
			(fabsf(u[ip * m[1] + j]) +
			fabsf(u[im * m[1] + j]) +
			fabsf(u[i * m[1] + jp]) +
			fabsf(u[i * m[1] + jm]));

	*running = *running + (unsigned long long int)(fabsf(val - u[i * m[1] + j]) > epsilon);

	uPrime[i * m[1] + j] = val;
}

int gpu_harmonic_alloc_2d(unsigned int *m, float *u,
		unsigned int *&d_m, float *&d_u, float *&d_uPrime)
{
	// Ensure the data is valid.
	if (u == nullptr || m == nullptr || m[0] == 0 || m[1] == 0) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Invalid data." << std::endl;
		return 1;
	}

	// Allocate the memory on the device.
	if (cudaMalloc(&d_m, 2 * sizeof(unsigned int)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the dimension size values." << std::endl;
		return 2;
	}
	if (cudaMalloc(&d_u, m[0] * m[1] * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the harmonic function values." << std::endl;
		return 2;
	}
	if (cudaMalloc(&d_uPrime, m[0] * m[1] * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the harmonic function values." << std::endl;
		return 2;
	}

	// Copy the data from the host to the device. Note: Even if things like d_uPrime get overwritten,
	// you MUST malloc AND memcpy to use them!
	if (cudaMemcpy(d_m, m, 2 * sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the dimension size function." << std::endl;
		return 3;
	}
	if (cudaMemcpy(d_u, u, m[0] * m[1] * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the harmonic function." << std::endl;;
		return 3;
	}
	if (cudaMemcpy(d_uPrime, u, m[0] * m[1] * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the harmonic function (prime)." << std::endl;
		return 3;
	}

	return 0;
}

int gpu_harmonic_execute_2d(unsigned int *m, float epsilon,
		unsigned int *d_m, float *d_u, float *d_uPrime,
		unsigned int numThreads)
{
	// Ensure the data is valid.
	if (m == nullptr || epsilon <= 0.0f || d_m == nullptr || d_u == nullptr || numThreads == 0) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Invalid data." << std::endl;
		return 1;
	}

	// Also ensure that the number of threads executed are valid.
	unsigned int numBlocks = m[0] * ((unsigned int)(m[1] / numThreads) + 1);
	if (numThreads % 32 != 0) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Must specify a number of threads divisible by 32 (the number of threads in a warp)." << std::endl;
		return 1;
	}

	// Now ensure that there are enough total threads (over all blocks) to run the solver.
	if (numBlocks * numThreads < m[0] * m[1]) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to specify enough blocks and threads to execute the solver." << std::endl;
		return 1;
	}

	// Create the running value, which keeps the iterations going so long as at least one element needs updating.
	unsigned long long int *running = new unsigned long long int[1];
	*running = 1;

	unsigned long long int *d_running = nullptr;
	if (cudaMalloc(&d_running, sizeof(unsigned long long int)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to allocate device-side memory for the running variable." << std::endl;
		return 2;
	}

	// Iterate until convergence.
	unsigned long long int iterations = 0;

	// Note: Must ensure that iterations is even so that d_u stores the final result, not d_uPrime.
//	while (iterations <= 200) {
	while (*running > 0) {
		unsigned int stagger = 100;

		// Reset delta on the device.
		if (iterations % stagger == 0) {
			*running = 0;

			if (cudaMemcpy(d_running, running, sizeof(unsigned long long int), cudaMemcpyHostToDevice) != cudaSuccess) {
				std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to copy running object from host to device." << std::endl;
				return 3;
			}
		}

		// Perform one step of the iteration.
		if (iterations % 2 == 0) {
			gpu_harmonic_iteration_2d<<< numBlocks, numThreads >>>(d_m, d_u, d_uPrime, epsilon, d_running);
		} else {
			gpu_harmonic_iteration_2d<<< numBlocks, numThreads >>>(d_m, d_uPrime, d_u, epsilon, d_running);
		}

		// Wait for the kernel to finish before looping more.
		cudaDeviceSynchronize();

		// Copy the running value computed by each thread back to the host.
		if (iterations % stagger == 0) {
			if (cudaMemcpy(running, d_running, sizeof(unsigned long long int), cudaMemcpyDeviceToHost) != cudaSuccess) {
				std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to copy running object from device to host." << std::endl;
				return 3;
			}
		}

		iterations++;
	}

//	std::cout << "Completed in " << iterations << " iterations." << std::endl;

	// Free the memory of the delta value.
	delete [] running;
	if (cudaFree(d_running) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to free memory for the running flag." << std::endl;
		return 4;
	}

	return 0;
}

int gpu_harmonic_get_2d(unsigned int *m, float *d_u, float *u)
{
	if (cudaMemcpy(u, d_u, m[0] * m[1] * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
		std::cerr << "Error[harmonic_get]: Failed to copy memory from device to host for the entire result." << std::endl;
		return 1;
	}
	return 0;
}

int gpu_harmonic_free_2d(unsigned int *d_m, float *d_u, float *d_uPrime)
{
	if (cudaFree(d_m) != cudaSuccess) {
		std::cerr << "Error[harmonic_free]: Failed to free memory for the dimension sizes." << std::endl;
		return 1;
	}
	if (cudaFree(d_u) != cudaSuccess) {
		std::cerr << "Error[harmonic_free]: Failed to free memory for the harmonic function." << std::endl;
		return 1;
	}
	if (cudaFree(d_uPrime) != cudaSuccess) {
		std::cerr << "Error[harmonic_free]: Failed to free memory for the harmonic function (prime)." << std::endl;
		return 1;
	}
	return 0;
}

