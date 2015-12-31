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


#include "harmonic.h"
#include "harmonic_utilities_gpu.h"
#include "error_codes.h"
#include "constants.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>


namespace epic {

__global__ void harmonic_utilities_set_cells_2d_gpu(unsigned int *m, float *u, unsigned int *locked,
    unsigned int k, unsigned int *v, unsigned int *types)
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= k) {
        return;
    }

    unsigned int x = v[i * 2 + 0];
    unsigned int y = v[i * 2 + 1];

    if (x >= m[0] || y >= m[1]) {
        return;
    }

    if (types[i] == EPIC_CELL_TYPE_GOAL) {
        u[x * m[1] + y] = EPIC_LOG_SPACE_GOAL;
        locked[x * m[1] + y] = 1;
    } else if (types[i] == EPIC_CELL_TYPE_OBSTACLE) {
        u[x * m[1] + y] = EPIC_LOG_SPACE_OBSTACLE;
        locked[x * m[1] + y] = 1;
    } else if (types[i] == EPIC_CELL_TYPE_FREE) {
        u[x * m[1] + y] = EPIC_LOG_SPACE_FREE;
        locked[x * m[1] + y] = 0;
    }
}


int harmonic_utilities_set_cells_2d_gpu(Harmonic *harmonic, unsigned int numThreads,
    unsigned int k, unsigned int *v, unsigned int *types)
{
    if (harmonic == nullptr || harmonic->n == 0 || harmonic->m == nullptr ||
            harmonic->u == nullptr || harmonic->locked == nullptr ||
            k == 0 || v == nullptr || types == nullptr) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: Invalid data.");
        return EPIC_ERROR_INVALID_DATA;
    }

    unsigned int *d_v = nullptr;
    unsigned int *d_types = nullptr;
    unsigned int numBlocks = (unsigned int)(k / numThreads) + 1;

    // Allocate memory and copy the data for the parameters we will pass to the device.
    if (cudaMalloc(&d_v, 2 * k * sizeof(unsigned int)) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                "Failed to allocate device-side memory for the vector of cell locations.");
        return EPIC_ERROR_DEVICE_MALLOC;
    }
    if (cudaMemcpy(d_v, v, 2 * k * sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                "Failed to allocate device-side memory for the vector of cell locations.");
        return EPIC_ERROR_MEMCPY_TO_DEVICE;
    }

    if (cudaMalloc(&d_types, k * sizeof(unsigned int)) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                "Failed to allocate device-side memory for the vector of types.");
        return EPIC_ERROR_DEVICE_MALLOC;
    }
    if (cudaMemcpy(d_types, types, k * sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                "Failed to allocate device-side memory for the vector of types.");
        return EPIC_ERROR_MEMCPY_TO_DEVICE;
    }

    // Call the function to update the values on the device's memory.
    harmonic_utilities_set_cells_2d_gpu<<< numBlocks, numThreads >>>(harmonic->d_m,
                                        harmonic->d_u, harmonic->d_locked,
                                        k, d_v, d_types);

    // Check if there was an error executing the kernel.
    if (cudaGetLastError() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                        "Failed to execute the 'set cells' kernel.");
        return EPIC_ERROR_KERNEL_EXECUTION;
    }

    // Wait for the kernel to finish before looping more.
    if (cudaDeviceSynchronize() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                    "Failed to synchronize the device after 'set cells' kernel.");
        return EPIC_ERROR_DEVICE_SYNCHRONIZE;
    }

    // Free memory for the parameters we allocated on the device.
    if (cudaFree(d_v) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                "Failed to free device-side memory for the vector of cell locations.");
        return EPIC_ERROR_DEVICE_FREE;
    }
    d_v = nullptr;

    if (cudaFree(d_types) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_utilities_set_cells_2d_gpu]: %s\n",
                "Failed to free device-side memory for the vector of types.");
        return EPIC_ERROR_DEVICE_FREE;
    }
    d_types = nullptr;

    return EPIC_SUCCESS;
}

}; // namespace epic

