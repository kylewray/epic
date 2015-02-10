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
#include <math.h>

#include "../include/gpu_jacobi_v2.h"

__global__ void gpu_jacobi_v2_check(unsigned int n, unsigned int d,
//		cudaTextureObject_t lockedTex,
		bool *locked,
		float *u, float *uPrime, float epsilon, unsigned int *running)
{
	unsigned int cell = blockIdx.x * blockDim.x + threadIdx.x;

	// Skip if the index is over the number of cells, or this is a locked obstacle cell.
	if (cell >= d || locked[cell]) {
//	if (cell >= d || tex1Dfetch<unsigned char>(lockedTex, cell) != 0) {
		return;
	}

	// If the difference is greater than epsilon, then we need to keep looping.
	for (unsigned int i = 0; i < 2 * n; i++) {
		if (fabs(u[i * d + cell] - uPrime[i * d + cell]) > epsilon) {
			*running = 1;
		}
	}
}

__global__ void gpu_jacobi_v2_iteration(unsigned int n, unsigned int d,
		cudaTextureObject_t indexTex,
//		cudaTextureObject_t lockedTex,
//		int *index,
		bool *locked,
		float *u, float *uPrime, float epsilon)
{
	unsigned int cell = blockIdx.x * blockDim.x + threadIdx.x;

	// Skip if the index is over the number of cells, or this is a locked obstacle cell.
	if (cell >= d || locked[cell]) {
//	if (cell >= d || tex1Dfetch<unsigned char>(lockedTex, cell) != 0) {
		return;
	}

	// Average the 2*n neighbors, following the Jacobi method.
	float val = 0.0f;
	for (unsigned int i = 0; i < 2 * n; i++) {
		val += u[i * d + cell];
	}
	val /= (float)(2 * n);

	// Update all occurrences of this value.
	for (unsigned int i = 0; i < 2 * n; i++) {
//		uPrime[row[i * d + cell] * d + col[i * d + cell]] = val;

		int adjust = 1 - i % 2;
//		if (signbit((float)index[i * d + cell]) != 0) {
		if (signbit((float)tex1Dfetch<int>(indexTex, i * d + cell)) != 0) {
			adjust = abs(adjust - 1);
		}

		__syncthreads();

//		unsigned int adjust = abs((float)((1 - i % 2) - (unsigned int)(signbit((float)index[i * d + cell]) != 0)));

//		uPrime[((unsigned int)(i / 2) * 2 + adjust) * d + abs(index[i * d + cell])] = val;
		uPrime[((unsigned int)(i / 2) * 2 + adjust) * d + abs(tex1Dfetch<int>(indexTex, i * d + cell))] = val;
	}
}

void gpu_jacobi_v2_index_to_coordinate(unsigned int n, unsigned int *m, unsigned int i, unsigned int *&c)
{
	// Actually allocate the memory for the coordinate.
	c = new unsigned int[n];

	// Compute the coordinate by modifying the through the index and continually
	// removing the 'pieces' corresponding to each dimension, based on its size.
	for (unsigned int k = 0; k < n; k++) {
		// Make sure it goes in the correct order over k.
		c[n - k - 1] = i % m[k];
		i = (unsigned int)(i / m[k]);
	}
}

void gpu_jacobi_v2_coordinate_to_index(unsigned int n, unsigned int *m, unsigned int *c, unsigned int &i)
{
	// The index offset based on the current dimension.
	unsigned int mk = 1;

	i = 0;

	// For each of the dimensions, compute the adjustment using the coordinate.
	for (unsigned int k = 0; k < n; k++) {
		// This is the offset based on previously computed dimensions.
		mk = 1;
		for (unsigned int j = 0; j < k; j++) {
			mk *= m[j];
		}

		// Make sure it goes in the correct order over k.
		i += c[n - k - 1] * mk;
	}
}

int gpu_jacobi_v2_alloc(unsigned int n, unsigned int d, unsigned int *m, float *u,
		int *&d_index, bool *&d_locked, float *&d_u, float *&d_uPrime)
{
	// Ensure the data is valid.
	if (n == 0 || d == 0 || m == nullptr || u == nullptr) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Invalid data." << std::endl;
		return 1;
	}

	// Compute, allocate, and copy the col variable.
	int *index = new int[2 * n * d];

	for (unsigned int cell = 0; cell < d; cell++) {
		// Convert the cell index to a coordinate.
		unsigned int *coord = nullptr;
		gpu_jacobi_v2_index_to_coordinate(n, m, cell, coord);

		// For each dimension, for both neighbors, compute the coordinate, convert to index, then assign value.
		for (unsigned int k = 0; k < n; k++) {
			// Adjust to make the coordinate refer to the neighbor with the -1 direction.
			bool adjusted = false;
			if (coord[k] > 0) {
				coord[k]--;
				adjusted = true;
			}

			// Convert this neighbor to a cell index (j).
			unsigned int j = 0;
			gpu_jacobi_v2_coordinate_to_index(n, m, coord, j);

			// For this column, we assign the cell index.
			if (adjusted) {
				index[(2 * k + 0) * d + cell] = j;
			} else {
				index[(2 * k + 0) * d + cell] = -((int)j);
			}

			// Properly adjust the coordinate back to the cell center.
			if (adjusted) {
				coord[k]++;
			}

			// Adjust to make the coordinate refer to the neighbor with the +1 direction.
			adjusted = false;
			if (coord[k] < m[k] - 1) {
				coord[k]++;
				adjusted = true;
			}

			// Convert this neighbor to a cell index (j).
			j = 0;
			gpu_jacobi_v2_coordinate_to_index(n, m, coord, j);

			// For this column, we assign the cell index.
			if (adjusted) {
				index[(2 * k + 1) * d + cell] = j;
			} else {
				index[(2 * k + 1) * d + cell] = -((int)j);
			}

			// Properly adjust the coordinate back to the cell center.
			if (adjusted) {
				coord[k]--;
			}
		}

		delete [] coord;
	}

	if (cudaMalloc(&d_index, 2 * n * d * sizeof(int)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to allocate device-side memory for the index values." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_index, index, 2 * n * d * sizeof(int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to copy memory from host to device for the indexes." << std::endl;
		return 3;
	}

	// Compute, allocate, and copy the locked variable.
	bool *locked = new bool[d];

	for (unsigned int cell = 0; cell < d; cell++) {
		if (signbit(u[cell]) != 0) {
			locked[cell] = 1;
		} else {
			locked[cell] = 0;
		}
	}

	if (cudaMalloc(&d_locked, d * sizeof(bool)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to allocate device-side memory for the locked values." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_locked, locked, d * sizeof(bool), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to copy memory from host to device for the locked." << std::endl;;
		return 3;
	}

	// Allocate and copy u.
	float *uDevice = new float[2 * n * d];

	for (unsigned int cell = 0; cell < d; cell++) {
		for (unsigned int neighbor = 0; neighbor < 2 * n; neighbor++) {
			uDevice[neighbor * d + cell] = fabs(u[abs(index[neighbor * d + cell])]);
		}
	}

	if (cudaMalloc(&d_u, 2 * n * d * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to allocate device-side memory for the harmonic function values." << std::endl;
		return 2;
	}
	if (cudaMemcpy(d_u, uDevice, 2 * n * d * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to copy memory from host to device for the harmonic function." << std::endl;;
		return 3;
	}

	if (cudaMalloc(&d_uPrime, 2 * n * d * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to allocate device-side memory for the harmonic (prime) function values." << std::endl;
		return 2;
	}
	if (cudaMemcpy(d_uPrime, uDevice, 2 * n * d * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_alloc]: Failed to copy memory from host to device for the harmonic function (prime)." << std::endl;
		return 3;
	}

	// Free everything afterwards, since you need to use the index (row/col) to figure out which spots to setup in the uDevice.
	delete [] index;
	delete [] locked;
	delete [] uDevice;

	return 0;
}

int gpu_jacobi_v2_execute(unsigned int n, unsigned int d, float epsilon,
		int *d_index, bool *d_locked, float *d_u, float *d_uPrime,
		unsigned int numThreads,
		unsigned int stagger)
{
	// Ensure the data is valid.
	if (n == 0 || d == 0 || epsilon <= 0.0f || d_index == nullptr || d_locked == nullptr ||
			d_u == nullptr || d_uPrime == nullptr || numThreads == 0) {
		std::cerr << "Error[gpu_jacobi_v2_execute]: Invalid data." << std::endl;
		return 1;
	}

	// Also ensure that the number of threads executed are valid.
	unsigned int numBlocks = (unsigned int)(d / numThreads) + 1;
	if (numThreads % 32 != 0) {
		std::cerr << "Error[gpu_jacobi_v2_execute]: Must specify a number of threads divisible by 32 (the number of threads in a warp)." << std::endl;
		return 1;
	}

	// We must ensure that the stagger for convergence checking is even (i.e., num iterations), so that d_u stores the final result, not d_uPrime.
	if (stagger == 0 || stagger % 2 == 1) {
		std::cerr << "Error[gpu_jacobi_v2_execute]: Stagger for convergence checking must be a positive even integer." << std::endl;
		return 1;
	}

	// Create the running value, which keeps the iterations going so long as at least one element needs updating.
	unsigned int *running = new unsigned int;
	*running = 1;

	unsigned int *d_running = nullptr;
	if (cudaMalloc(&d_running, sizeof(unsigned int)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to allocate device-side memory for the running variable." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_running, running, sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to copy running object from host to device." << std::endl;
		return 3;
	}

	// Assign the index memory as texture memory.
	cudaResourceDesc indexResDesc;
	memset(&indexResDesc, 0, sizeof(indexResDesc));
	indexResDesc.resType = cudaResourceTypeLinear;
	indexResDesc.res.linear.devPtr = d_index;
	indexResDesc.res.linear.desc.f = cudaChannelFormatKindSigned;
	indexResDesc.res.linear.desc.x = 32; // Bits per channel, since int = 2^{32}.
	indexResDesc.res.linear.sizeInBytes = 2 * n * d * sizeof(int);

	cudaTextureDesc indexTexDesc;
	memset(&indexTexDesc, 0, sizeof(indexTexDesc));
	indexTexDesc.readMode = cudaReadModeElementType;

	// Actually create the texture object.
	cudaTextureObject_t indexTex = 0;
	cudaCreateTextureObject(&indexTex, &indexResDesc, &indexTexDesc, nullptr);

//	// Assign the locked memory as texture memory.
//	cudaResourceDesc lockedResDesc;
//	memset(&lockedResDesc, 0, sizeof(lockedResDesc));
//	lockedResDesc.resType = cudaResourceTypeLinear;
//	lockedResDesc.res.linear.devPtr = d_locked;
//	lockedResDesc.res.linear.desc.f = cudaChannelFormatKindUnsigned;
//	lockedResDesc.res.linear.desc.x = 8; // Bits per channel, since bool = 2^{8}.
//	lockedResDesc.res.linear.sizeInBytes = d * sizeof(unsigned char); // Same as bool.
//
//	cudaTextureDesc lockedTexDesc;
//	memset(&lockedTexDesc, 0, sizeof(lockedTexDesc));
//	lockedTexDesc.readMode = cudaReadModeElementType;
//
//	// Actually create the texture object.
//	cudaTextureObject_t lockedTex = 0;
//	cudaCreateTextureObject(&lockedTex, &lockedResDesc, &lockedTexDesc, nullptr);

	// Iterate until convergence.
	unsigned int iterations = 0;

	// Important Note: Must ensure that iterations is even so that d_u stores the final result, not d_uPrime.
	// Also, always run at least 'stagger' iterations.
	while (*running > 0 || iterations < stagger) {
//		std::cout << "Iteration " << iterations << std::endl; std::cout.flush();

		// Perform one step of the iteration, either using u and storing in uPrime, or vice versa.
		if (iterations % 2 == 0) {
			gpu_jacobi_v2_iteration<<< numBlocks, numThreads >>>(n, d, indexTex, d_locked, d_u, d_uPrime, epsilon);
		} else {
			gpu_jacobi_v2_iteration<<< numBlocks, numThreads >>>(n, d, indexTex, d_locked, d_uPrime, d_u, epsilon);
		}
		if (cudaGetLastError() != cudaSuccess) {
			std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to execute the 'iteration' kernel." << std::endl;
			return 3;
		}

		// Wait for the kernel to finish before looping more.
		if (cudaDeviceSynchronize() != cudaSuccess) {
			std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to synchronize the device." << std::endl;
			return 3;
		}

		// Reset the running variable, check for convergence, then copy the running value back to the host.
		if (iterations % stagger == 0) {
			*running = 0;

			if (cudaMemcpy(d_running, running, sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to copy running object from host to device." << std::endl;
				return 3;
			}

			gpu_jacobi_v2_check<<< numBlocks, numThreads >>>(n, d, d_locked, d_u, d_uPrime, epsilon, d_running);
			if (cudaGetLastError() != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to execute the 'check' kernel." << std::endl;
				return 3;
			}

			if (cudaDeviceSynchronize() != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to synchronize the device when checking for convergence." << std::endl;
				return 3;
			}

			if (cudaMemcpy(running, d_running, sizeof(unsigned int), cudaMemcpyDeviceToHost) != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to copy running object from device to host." << std::endl;
				return 3;
			}
		}

		iterations++;
	}

//	std::cout << "GPU Jacobi 2D: Completed in " << iterations << " iterations." << std::endl;

	// Free the texture bindings.
	cudaDestroyTextureObject(indexTex);
//	cudaDestroyTextureObject(lockedTex);

	// Free the memory of the delta value.
	delete running;
	if (cudaFree(d_running) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_execute]: Failed to free memory for the running flag." << std::endl;
		return 4;
	}

	return 0;
}

int gpu_jacobi_v2_get_all(unsigned int n, unsigned int d, int *d_index, bool *d_locked, float *d_u, float *u)
{
	// Read the indexes (rows/cols) and actual u-values from the device.
	int *index = new int[2 * n * d];

	if (cudaMemcpy(index, d_index, 2 * n * d * sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_get_all]: Failed to copy memory from device to host for the indexes." << std::endl;
		return 1;
	}

	bool *locked = new bool[d];

	if (cudaMemcpy(locked, d_locked, d * sizeof(bool), cudaMemcpyDeviceToHost) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_get_all]: Failed to copy memory from device to host for the locked." << std::endl;
		return 1;
	}

	float *uDevice = new float[2 * n * d];

	if (cudaMemcpy(uDevice, d_u, 2 * n * d * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_get_all]: Failed to copy memory from device to host for the resultant u values." << std::endl;
		return 1;
	}

	// Read the values from uDevice and store them in u.
	for (unsigned int cell = 0; cell < d; cell++) {
		unsigned int i = 0; // Just fix one of them.

		int adjust = 1 - i % 2;
		if (signbit((float)index[i * d + cell]) != 0) {
			adjust = abs(adjust - 1);
		}
//		unsigned int adjust = abs((float)((1 - i % 2) - (unsigned int)(signbit((float)index[i * d + cell]) != 0)));

		u[cell] = uDevice[((unsigned int)(i / 2) * 2 + adjust) * d + abs(index[i * d + cell])];

		if (locked[cell]) {
			u[cell] = -u[cell];
		}
	}

	// Free everything afterwards, since you need to use the index (row/col) to figure out which spots to read in uDevice.
	delete [] index;
	delete [] locked;
	delete [] uDevice;

	return 0;
}

int gpu_jacobi_v2_free(int *d_index, bool *d_locked, float *d_u, float *d_uPrime)
{
	if (cudaFree(d_index) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_free]: Failed to free memory for the indexes." << std::endl;
		return 1;
	}
	if (cudaFree(d_locked) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_free]: Failed to free memory for the locked." << std::endl;
		return 1;
	}
	if (cudaFree(d_u) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_free]: Failed to free memory for the harmonic function." << std::endl;
		return 1;
	}
	if (cudaFree(d_uPrime) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_v2_free]: Failed to free memory for the harmonic function (prime)." << std::endl;
		return 1;
	}
	return 0;
}
