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


#include <iostream>

#include "../include/gpu.h"

__global__ void gpu_jacobi_check(unsigned int n, unsigned int m, unsigned int *row, unsigned int *col, float *locked, float *u, float *uPrime, float epsilon, unsigned int *running)
{
	unsigned int cell = blockIdx.x * blockDim.x + threadIdx.x;

	// Skip if the index is over the number of cells, or this is a locked obstacle cell.
	if (cell >= m || locked[cell]) {
		return;
	}

	// If the difference is greater than epsilon, then we need to keep looping.
	for (unsigned int i = 0; i < 2 * n; i++) {
		if (fabs(u[i * m + cell] - uPrime[i * m + cell]) > epsilon) {
			*running = 1;
		}
	}
}

__global__ void gpu_jacobi_iteration(unsigned int n, unsigned int m, unsigned int *row, unsigned int *col, float *locked, float *u, float *uPrime, float epsilon)
{
	unsigned int cell = blockIdx.x * blockDim.x + threadIdx.x;

	// Skip if the index is over the number of cells, or this is a locked obstacle cell.
	if (cell >= m || locked[cell]) {
		return;
	}

	// Average the 2*n neighbors, following the Jacobi method.
	float val = 0.0f;
	for (unsigned int i = 0; i < 2 * n; i++) {
		val += u[i * m + cell];
	}
	val /= (float)n;

	// Update all occurrences of this value.
	for (unsigned int i = 0; i < 2 * n; i++) {
		uPrime[row[i * m + cell] * m + col[i * m + cell]] = val;
	}
}

int gpu_jacobi_alloc(unsigned int n, unsigned int m, float *u,
		unsigned int *&d_row, unsigned int *&d_col, bool *&d_locked, float *&d_u, float *&d_uPrime)
{
	// Ensure the data is valid.
	if (n == 0 || m == 0 || u == nullptr) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Invalid data." << std::endl;
		return 1;
	}

	// Compute, allocate, and copy the row variable.
	unsigned int *row = new unsigned int[2 * n * m];

	for (unsigned int cell = 0; cell < m; cell++) {
		for (unsigned int neighbor = 0; neighbor < 2 * n; neighbor++) {
			row[neighbor * m + cell] = 1337;
		}
	}

	if (cudaMalloc(&d_row, 2 * n * m * sizeof(unsigned int)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the row values." << std::endl;
		return 2;
	}
	if (cudaMemcpy(d_row, row, 2 * n * m * sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the rows." << std::endl;
		return 3;
	}

	delete [] row;

	// Compute, allocate, and copy the col variable.
	unsigned int *col = new unsigned int[2 * n * m];

	for (unsigned int cell = 0; cell < m; cell++) {
		for (unsigned int neighbor = 0; neighbor < 2 * n; neighbor++) {
			col[neighbor * m + cell] = 1337;
		}
	}

	if (cudaMalloc(&d_col, 2 * n * m * sizeof(unsigned int)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the col values." << std::endl;
		return 2;
	}
	if (cudaMemcpy(d_col, row, 2 * n * m * sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the cols." << std::endl;
		return 3;
	}

	delete [] col;

	// Compute, allocate, and copy the locked variable.
	bool *locked = new bool[m];

	for (unsigned int cell = 0; cell < m; cell++) {
		locked[cell] = (u[cell] < 0.0f);
	}

	if (cudaMalloc(&d_locked, m * sizeof(bool)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the locked values." << std::endl;
		return 2;
	}
	if (cudaMemcpy(d_locked, locked, m * sizeof(bool), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the locked." << std::endl;;
		return 3;
	}

	delete [] locked;

	// Allocate and copy u.
	unsigned int *uDevice = new unsigned int[2 * n * m];

	for (unsigned int cell = 0; cell < m; cell++) {
		for (unsigned int neighbor = 0; neighbor < 2 * n; neighbor++) {
			uDevice[neighbor * m + cell] = u[cell]; // TODO: This must be the correct value corresponding from the row/col following the indices above.
		}
	}

	if (cudaMalloc(&d_u, 2 * n * m * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the harmonic function values." << std::endl;
		return 2;
	}
	if (cudaMemcpy(d_u, uDevice, 2 * n * m * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the harmonic function." << std::endl;;
		return 3;
	}

	delete [] uDevice;

	// Allocate and copy uPrime.
	unsigned int *uPrimeDevice = new unsigned int[2 * n * m];

	for (unsigned int cell = 0; cell < m; cell++) {
		for (unsigned int neighbor = 0; neighbor < 2 * n; neighbor++) {
			uPrimeDevice[neighbor * m + cell] = u[cell]; // TODO: This must be the correct value corresponding from the row/col following the indices above.
		}
	}

	if (cudaMalloc(&d_uPrime, 2 * n * m * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to allocate device-side memory for the harmonic (prime) function values." << std::endl;
		return 2;
	}
	if (cudaMemcpy(d_uPrime, uPrimeDevice, 2 * n * m * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_alloc_2d]: Failed to copy memory from host to device for the harmonic function (prime)." << std::endl;
		return 3;
	}

	delete [] uPrimeDevice;

	return 0;
}

int gpu_harmonic_execute_2d(unsigned int *m, float epsilon,
		unsigned int *d_m, float *d_u, float *d_uPrime,
		unsigned int numBlocks, unsigned int numThreads,
		unsigned int stagger)
{
	// Ensure the data is valid.
	if (m == nullptr || epsilon <= 0.0f || d_m == nullptr || d_u == nullptr || numBlocks == 0 || numThreads == 0) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Invalid data." << std::endl;
		return 1;
	}

	// Also ensure that the number of threads executed are valid.
	if (numThreads % 32 != 0) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Must specify a number of threads divisible by 32 (the number of threads in a warp)." << std::endl;
		return 1;
	}

	// We must ensure that the stagger for convergence checking is even (i.e., num iterations), so that d_u stores the final result, not d_uPrime.
	if (stagger % 2 == 1) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Stagger for convergence checking must be even." << std::endl;
		return 1;
	}

	// Create the running value, which keeps the iterations going so long as at least one element needs updating.
	unsigned int *running = new unsigned int;
	*running = 1;

	unsigned int *d_running = nullptr;
	if (cudaMalloc(&d_running, sizeof(unsigned int)) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to allocate device-side memory for the running variable." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_running, running, sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to copy running object from host to device." << std::endl;
		return 3;
	}

	// Iterate until convergence.
	unsigned long long int iterations = 0;

	// Important Note: Must ensure that iterations is even so that d_u stores the final result, not d_uPrime.
	while (*running > 0) {
		// Perform one step of the iteration, either using u and storing in uPrime, or vice versa.
		if (iterations % 2 == 0) {
			gpu_harmonic_iteration_2d<<< numBlocks, numThreads >>>(d_m, d_u, d_uPrime, epsilon);
		} else {
			gpu_harmonic_iteration_2d<<< numBlocks, numThreads >>>(d_m, d_uPrime, d_u, epsilon);
		}
		if (cudaGetLastError() != cudaSuccess) {
			std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to execute the 'iteration' kernel." << std::endl;
			return 3;
		}

		// Wait for the kernel to finish before looping more.
		if (cudaDeviceSynchronize() != cudaSuccess) {
			std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to synchronize the device." << std::endl;
			return 3;
		}

		// Reset the running variable, check for convergence, then copy the running value back to the host.
		if (iterations % stagger == 0) {
			*running = 0;

			if (cudaMemcpy(d_running, running, sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
				std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to copy running object from host to device." << std::endl;
				return 3;
			}

			gpu_harmonic_check_2d<<< numBlocks, numThreads >>>(d_m, d_u, d_uPrime, epsilon, d_running);
			if (cudaGetLastError() != cudaSuccess) {
				std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to execute the 'check' kernel." << std::endl;
				return 3;
			}

			if (cudaDeviceSynchronize() != cudaSuccess) {
				std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to synchronize the device when checking for convergence." << std::endl;
				return 3;
			}

			if (cudaMemcpy(running, d_running, sizeof(unsigned int), cudaMemcpyDeviceToHost) != cudaSuccess) {
				std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to copy running object from device to host." << std::endl;
				return 3;
			}
		}

		iterations++;
	}

//	std::cout << "GPU Jacobi 2D: Completed in " << iterations << " iterations." << std::endl;

	// Free the memory of the delta value.
	delete running;
	if (cudaFree(d_running) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_execute_2d]: Failed to free memory for the running flag." << std::endl;
		return 4;
	}

	return 0;
}

int gpu_harmonic_free(unsigned int *d_row, unsigned int *d_col, bool *d_locked, float *d_u, float *d_uPrime)
{
	if (cudaFree(d_row) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_free_2d]: Failed to free memory for the rows." << std::endl;
		return 1;
	}
	if (cudaFree(d_col) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_free_2d]: Failed to free memory for the cols." << std::endl;
		return 1;
	}
	if (cudaFree(d_locked) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_free_2d]: Failed to free memory for the locked." << std::endl;
		return 1;
	}
	if (cudaFree(d_u) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_free_2d]: Failed to free memory for the harmonic function." << std::endl;
		return 1;
	}
	if (cudaFree(d_uPrime) != cudaSuccess) {
		std::cerr << "Error[gpu_harmonic_free_2d]: Failed to free memory for the harmonic function (prime)." << std::endl;
		return 1;
	}
	return 0;
}
