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

#include "../include/gpu_jacobi.h"

__global__ void gpu_jacobi_check(unsigned int n, unsigned int d,
		bool *locked, float *u, float *uPrime, float epsilon, unsigned int *running)
{
	unsigned int cell = blockIdx.x * blockDim.x + threadIdx.x;

	// Skip if the index is over the number of cells, or this is a locked obstacle cell.
	if (cell >= d || locked[cell]) {
		return;
	}

	if (fabs(u[(2 * n) * d + cell] - uPrime[(2 * n) * d + cell]) > epsilon) {
		*running = 1;
	}
}

__global__ void gpu_jacobi_iteration(unsigned int n, unsigned int d,
		int *index, bool *locked, float *u, float *uPrime, float epsilon, float omega)
{
	unsigned int cell = blockIdx.x * blockDim.x + threadIdx.x;

	// Skip if the index is over the number of cells, or this is a locked obstacle cell.
	if (cell >= d || locked[cell]) {
		return;
	}

	float val = 0.0f;
	for (unsigned int i = 0; i < 2 * n; i++) {
		val += u[i * d + cell];
	}
	val *= omega / (float)(2 * n);
	val += (1.0f - omega) * u[(2 * n) * d + cell];

	// Update all occurrences of this value.
	for (unsigned int i = 0; i < 2 * n; i++) {
		int adjust = 1 - i % 2;
		if (signbit((float)index[i * d + cell]) != 0) {
			adjust = abs(adjust - 1);
		}

		__syncthreads();

		uPrime[((unsigned int)(i / 2) * 2 + adjust) * d + abs(index[i * d + cell])] = val;
	}

	// Finally, update the reference copy of itself on its own column.
	uPrime[(2 * n) * d + cell] = val;
}

void gpu_jacobi_index_to_coordinate(unsigned int n, unsigned int *m, unsigned int i, unsigned int *&c)
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

void gpu_jacobi_coordinate_to_index(unsigned int n, unsigned int *m, unsigned int *c, unsigned int &i)
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

bool check_removable(unsigned int n, unsigned int *m, float *u, unsigned int cell)
{
	bool removeCell = false;

	if (signbit(u[cell]) != 0) {
		removeCell = true;

		// Convert the cell index to a coordinate.
		unsigned int *coord = nullptr;
		gpu_jacobi_index_to_coordinate(n, m, cell, coord);

		// This is an obstacle, so check all of its neighbors to see if
		// they are also locked. If so, we can remove this cell from
		// our computation.
		for (unsigned int k = 0; k < n; k++) {
			// Adjust to make the coordinate refer to the neighbor with the -1 direction.
			bool adjusted = false;
			if (coord[k] > 0) {
				coord[k]--;
				adjusted = true;
			}

			// Convert this neighbor to a cell index (j).
			unsigned int j = 0;
			gpu_jacobi_coordinate_to_index(n, m, coord, j);

			// If this neighbor (-1) is not locked, then we know its an important
			// cell, so we can stop and keep it.
			if (signbit(u[j]) == 0) {
				removeCell = false;
				break;
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
			gpu_jacobi_coordinate_to_index(n, m, coord, j);

			// Again, if this neighbor (+1) is not locked, then we know its an important
			// cell, so we can stop and keep it.
			if (signbit(u[j]) == 0) {
				removeCell = false;
				break;
			}

			// Properly adjust the coordinate back to the cell center.
			if (adjusted) {
				coord[k]--;
			}
		}

		delete [] coord;
	}

	return removeCell;
}

int gpu_jacobi_alloc(unsigned int n, unsigned int d, unsigned int *m,
		unsigned int *&cellIndexActualToAdjusted, unsigned int *&cellIndexAdjustedToActual, float *u,
		unsigned int &dNew, int *&d_index, bool *&d_locked, float *&d_u, float *&d_uPrime)
{
	// Ensure the data is valid.
	if (n == 0 || d == 0 || m == nullptr || u == nullptr) {
		std::cerr << "Error[gpu_jacobi_alloc]: Invalid data." << std::endl;
		return 1;
	}

	// First, compute how many cells can actually be removed, assigning the new 'd'.
	dNew = d;

	for (unsigned int cell = 0; cell < d; cell++) {
		// If this is an obstacle, check all of its neighbors to see if
		// they are also locked. If so, we can remove this cell from
		// our computation.
		if (check_removable(n, m, u, cell)) {
			dNew--;
		}
	}

	// Next, compute the mappings between actual and adjusted indexes.
	cellIndexActualToAdjusted = new unsigned int[d];
	cellIndexAdjustedToActual = new unsigned int[dNew];

	unsigned int cellRemovalAdjustment = 0;

	for (unsigned int cell = 0; cell < d; cell++) {
		// If this is an obstacle, check all of its neighbors to see if
		// they are also locked. If so, we can remove this cell from
		// our computation.
		if (check_removable(n, m, u, cell)) {
			// Just assign a value to this...
			cellIndexActualToAdjusted[cell] = 0; //cell - cellRemovalAdjustment;
			cellRemovalAdjustment++;
			continue;
		}

		cellIndexActualToAdjusted[cell] = cell - cellRemovalAdjustment;
		cellIndexAdjustedToActual[cell - cellRemovalAdjustment] = cell;
	}

	// Compute, allocate, and copy the col variable.
	int *index = new int[2 * n * dNew];

	for (unsigned int cell = 0; cell < d; cell++) {
		// If this is removable, skip it and remember how many have been skipped so far.
		if (check_removable(n, m, u, cell)) {
			continue;
		}

		// Convert the cell index to a coordinate.
		unsigned int *coord = nullptr;
		gpu_jacobi_index_to_coordinate(n, m, cell, coord);

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
			gpu_jacobi_coordinate_to_index(n, m, coord, j);

			// For this column, we assign the cell index.
			if (adjusted) {
				index[(2 * k + 0) * dNew + cellIndexActualToAdjusted[cell]] = cellIndexActualToAdjusted[j];
			} else {
				index[(2 * k + 0) * dNew + cellIndexActualToAdjusted[cell]] = -((int)cellIndexActualToAdjusted[j]);
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
			gpu_jacobi_coordinate_to_index(n, m, coord, j);

			// For this column, we assign the cell index.
			if (adjusted) {
				index[(2 * k + 1) * dNew + cellIndexActualToAdjusted[cell]] = cellIndexActualToAdjusted[j];
			} else {
				index[(2 * k + 1) * dNew + cellIndexActualToAdjusted[cell]] = -((int)cellIndexActualToAdjusted[j]);
			}

			// Properly adjust the coordinate back to the cell center.
			if (adjusted) {
				coord[k]--;
			}
		}

		delete [] coord;
	}

	if (cudaMalloc(&d_index, 2 * n * dNew * sizeof(int)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to allocate device-side memory for the index values." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_index, index, 2 * n * dNew * sizeof(int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to copy memory from host to device for the indexes." << std::endl;
		return 3;
	}

	// Compute, allocate, and copy the locked variable.
	bool *locked = new bool[dNew];

	for (unsigned int cell = 0; cell < d; cell++) {
		// If this is removable, skip it and remember how many have been skipped so far.
		if (check_removable(n, m, u, cell)) {
			continue;
		}

		if (signbit(u[cell]) != 0) {
			locked[cellIndexActualToAdjusted[cell]] = 1;
		} else {
			locked[cellIndexActualToAdjusted[cell]] = 0;
		}
	}

	if (cudaMalloc(&d_locked, dNew * sizeof(bool)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to allocate device-side memory for the locked values." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_locked, locked, dNew * sizeof(bool), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to copy memory from host to device for the locked." << std::endl;;
		return 3;
	}

	// Allocate and copy u.
	float *uDevice = new float[(2 * n + 1) * dNew];

	for (unsigned int cell = 0; cell < d; cell++) {
		// If this is removable, skip it and remember how many have been skipped so far.
		if (check_removable(n, m, u, cell)) {
			continue;
		}

		for (unsigned int neighbor = 0; neighbor < 2 * n; neighbor++) {
			uDevice[neighbor * dNew + cellIndexActualToAdjusted[cell]] =
					fabs(u[cellIndexAdjustedToActual[abs(index[neighbor * dNew + cellIndexActualToAdjusted[cell]])]]);
		}

		uDevice[(2 * n) * dNew + cellIndexActualToAdjusted[cell]] = fabs(u[cell]);
	}

	if (cudaMalloc(&d_u, (2 * n + 1) * dNew * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to allocate device-side memory for the harmonic function values." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_u, uDevice, (2 * n + 1) * dNew * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to copy memory from host to device for the harmonic function." << std::endl;;
		return 3;
	}

	if (cudaMalloc(&d_uPrime, (2 * n + 1) * dNew * sizeof(float)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to allocate device-side memory for the harmonic (prime) function values." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_uPrime, uDevice, (2 * n + 1) * dNew * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_alloc]: Failed to copy memory from host to device for the harmonic function (prime)." << std::endl;
		return 3;
	}

	// Free everything afterwards, since you need to use the index (row/col) to figure out which spots to setup in the uDevice.
	delete [] index;
	delete [] locked;
	delete [] uDevice;

	return 0;
}

int gpu_jacobi_execute(unsigned int n, unsigned int d, float epsilon,
		int *d_index, bool *d_locked, float *d_u, float *d_uPrime,
		unsigned int numThreads,
		unsigned int stagger)
{
	// Ensure the data is valid.
	if (n == 0 || d == 0 || epsilon <= 0.0f || d_index == nullptr || d_locked == nullptr ||
			d_u == nullptr || d_uPrime == nullptr || numThreads == 0) {
		std::cerr << "Error[gpu_jacobi_execute]: Invalid data." << std::endl;
		return 1;
	}

	// Also ensure that the number of threads executed are valid.
	unsigned int numBlocks = (unsigned int)(d / numThreads) + 1;
	if (numThreads % 32 != 0) {
		std::cerr << "Error[gpu_jacobi_execute]: Must specify a number of threads divisible by 32 (the number of threads in a warp)." << std::endl;
		return 1;
	}

	// We must ensure that the stagger for convergence checking is even (i.e., num iterations), so that d_u stores the final result, not d_uPrime.
	if (stagger == 0 || stagger % 2 == 1) {
		std::cerr << "Error[gpu_jacobi_execute]: Stagger for convergence checking must be a positive even integer." << std::endl;
		return 1;
	}

	// Create the running value, which keeps the iterations going so long as at least one element needs updating.
	unsigned int *running = new unsigned int;
	*running = 1;

	unsigned int *d_running = nullptr;
	if (cudaMalloc(&d_running, sizeof(unsigned int)) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_execute]: Failed to allocate device-side memory for the running variable." << std::endl;
		return 2;
	}

	if (cudaMemcpy(d_running, running, sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_execute]: Failed to copy running object from host to device." << std::endl;
		return 3;
	}

//	// Assign the index memory as texture memory.
//	cudaResourceDesc indexResDesc;
//	memset(&indexResDesc, 0, sizeof(indexResDesc));
//	indexResDesc.resType = cudaResourceTypeLinear;
//	indexResDesc.res.linear.devPtr = d_index;
//	indexResDesc.res.linear.desc.f = cudaChannelFormatKindSigned;
//	indexResDesc.res.linear.desc.x = 32; // Bits per channel, since int = 2^{32}.
//	indexResDesc.res.linear.sizeInBytes = 2 * n * d * sizeof(int);
//
//	cudaTextureDesc indexTexDesc;
//	memset(&indexTexDesc, 0, sizeof(indexTexDesc));
//	indexTexDesc.readMode = cudaReadModeElementType;
//
//	// Actually create the texture object.
//	cudaTextureObject_t indexTex = 0;
//	cudaCreateTextureObject(&indexTex, &indexResDesc, &indexTexDesc, nullptr);

	// Iterate until convergence.
	unsigned int iterations = 0;

	// Important Note: Must ensure that iterations is even so that d_u stores the final result, not d_uPrime.
	// Also, always run at least 'stagger' iterations.
	while (*running > 0 || iterations < stagger) {
//	while (iterations < 10) {
//		std::cout << "Iteration " << iterations << std::endl; std::cout.flush();

		// Perform one step of the iteration, either using u and storing in uPrime, or vice versa.
		if (iterations % 2 == 0) {
			gpu_jacobi_iteration<<< numBlocks, numThreads >>>(n, d, d_index, d_locked, d_u, d_uPrime, epsilon, 1.0f); //0.5f);
//			gpu_jacobi_iteration<<< numBlocks, numThreads >>>(n, d, indexTex, d_locked, d_u, d_uPrime, epsilon, 0.5f);
		} else {
			gpu_jacobi_iteration<<< numBlocks, numThreads >>>(n, d, d_index, d_locked, d_uPrime, d_u, epsilon, 1.0f); //1.5f);
//			gpu_jacobi_iteration<<< numBlocks, numThreads >>>(n, d, indexTex, d_locked, d_uPrime, d_u, epsilon, 1.5f);
		}
//		if (iterations % 2 == 0) {
//			gpu_jacobi_iteration<<< numBlocks, numThreads >>>(n, d, indexTex, d_locked, d_u, d_uPrime, epsilon, 0.5f + 0.5f * min(1.0f, (float)iterations / 100.0f));
//		} else {
//			gpu_jacobi_iteration<<< numBlocks, numThreads >>>(n, d, indexTex, d_locked, d_uPrime, d_u, epsilon, 30.0f / (float)min(30, iterations + 1));
//		}
		if (cudaGetLastError() != cudaSuccess) {
			std::cerr << "Error[gpu_jacobi_execute]: Failed to execute the 'iteration' kernel." << std::endl;
			return 3;
		}

		// Wait for the kernel to finish before looping more.
		if (cudaDeviceSynchronize() != cudaSuccess) {
			std::cerr << "Error[gpu_jacobi_execute]: Failed to synchronize the device." << std::endl;
			return 3;
		}

		// Reset the running variable, check for convergence, then copy the running value back to the host.
		if (iterations % stagger == 0) {
			*running = 0;

			if (cudaMemcpy(d_running, running, sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_execute]: Failed to copy running object from host to device." << std::endl;
				return 3;
			}

			gpu_jacobi_check<<< numBlocks, numThreads >>>(n, d, d_locked, d_u, d_uPrime, epsilon, d_running);
			if (cudaGetLastError() != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_execute]: Failed to execute the 'check' kernel." << std::endl;
				return 3;
			}

			if (cudaDeviceSynchronize() != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_execute]: Failed to synchronize the device when checking for convergence." << std::endl;
				return 3;
			}

			if (cudaMemcpy(running, d_running, sizeof(unsigned int), cudaMemcpyDeviceToHost) != cudaSuccess) {
				std::cerr << "Error[gpu_jacobi_execute]: Failed to copy running object from device to host." << std::endl;
				return 3;
			}
		}

		iterations++;
	}

//	std::cout << "GPU Jacobi 2D: Completed in " << iterations << " iterations." << std::endl;

	// Free the texture bindings.
//	cudaDestroyTextureObject(indexTex);

	// Free the memory of the delta value.
	delete running;
	if (cudaFree(d_running) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_execute]: Failed to free memory for the running flag." << std::endl;
		return 4;
	}

	return 0;
}

int gpu_jacobi_get_all(unsigned int n, unsigned int d, unsigned int dNew, unsigned int *m,
		unsigned int *cellIndexActualToAdjusted, int *d_index, bool *d_locked, float *d_u,
		float *u)
{
	// Read the indexes (rows/cols) and actual u-values from the device.
	int *index = new int[2 * n * dNew];

	if (cudaMemcpy(index, d_index, 2 * n * dNew * sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_get_all]: Failed to copy memory from device to host for the indexes." << std::endl;
		return 1;
	}

	bool *locked = new bool[dNew];

	if (cudaMemcpy(locked, d_locked, dNew * sizeof(bool), cudaMemcpyDeviceToHost) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_get_all]: Failed to copy memory from device to host for the locked." << std::endl;
		return 1;
	}

	float *uDevice = new float[(2 * n + 1) * dNew];

	if (cudaMemcpy(uDevice, d_u, (2 * n + 1) * dNew * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_get_all]: Failed to copy memory from device to host for the resultant u values." << std::endl;
		return 1;
	}

	// Read the values from uDevice and store them in u.
	for (unsigned int cell = 0; cell < d; cell++) {
		// If this is removable, skip it and remember how many have been skipped so far.
		if (check_removable(n, m, u, cell)) {
			u[cell] = -1.0f; // By definition, it is locked as an obstacle.
			continue;
		}

//		unsigned int neighbor = 0; // Just fix one of the neighbors.
//
//		int adjust = 1 - neighbor % 2;
//		if (signbit((float)index[neighbor * dNew + cellIndexActualToAdjusted[cell]]) != 0) {
//			adjust = abs(adjust - 1);
//		}
//
//		u[cell] = uDevice[((unsigned int)(neighbor / 2) * 2 + adjust) * dNew + abs(index[neighbor * dNew + cellIndexActualToAdjusted[cell]])];

		u[cell] = uDevice[(2 * n) * dNew + cellIndexActualToAdjusted[cell]];

		if (locked[cellIndexActualToAdjusted[cell]]) {
			u[cell] = -u[cell];
		}
	}

	// Free everything afterwards, since you need to use the index (row/col) to figure out which spots to read in uDevice.
	delete [] index;
	delete [] locked;
	delete [] uDevice;

	return 0;
}

int gpu_jacobi_free(unsigned int *&cellIndexActualToAdjusted, unsigned int *&cellIndexAdjustedToActual,
		int *d_index, bool *d_locked, float *d_u, float *d_uPrime)
{
	delete [] cellIndexActualToAdjusted;
	cellIndexActualToAdjusted = nullptr;

	delete [] cellIndexAdjustedToActual;
	cellIndexAdjustedToActual = nullptr;

	if (cudaFree(d_index) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_free]: Failed to free memory for the indexes." << std::endl;
		return 1;
	}
	if (cudaFree(d_locked) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_free]: Failed to free memory for the locked." << std::endl;
		return 1;
	}
	if (cudaFree(d_u) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_free]: Failed to free memory for the harmonic function." << std::endl;
		return 1;
	}
	if (cudaFree(d_uPrime) != cudaSuccess) {
		std::cerr << "Error[gpu_jacobi_free]: Failed to free memory for the harmonic function (prime)." << std::endl;
		return 1;
	}
	return 0;
}
