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


#include "harmonic.h"
#include "harmonic_gpu.h"
#include "harmonic_model_gpu.h"
#include "error_codes.h"
#include "constants.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>


__global__ void harmonic_update_2d_gpu(unsigned int *m, float *u, unsigned int *locked, unsigned int currentIteration)
{
    for (unsigned int x0 = blockIdx.x; x0 < m[0]; x0 += gridDim.x) {
        unsigned int offset = (unsigned int)((currentIteration % 2) != (x0 % 2));

        for (unsigned int x1 = 2 * threadIdx.x + offset; x1 < m[1]; x1 += 2 * blockDim.x) {
            // If this is locked, then wait for the other threads to finish in the warp, then continue.
            if (locked[x0 * m[1] + x1]) {
                __syncthreads();
                continue;
            }

            // Update the value at this location with the log-sum-exp trick.
            float maxVal = FLT_MIN;
            maxVal = fmaxf(u[(x0 - 1) * m[1] + x1], u[(x0 + 1) * m[1] + x1]);
            maxVal = fmaxf(maxVal, u[x0 * m[1] + (x1 - 1)]);
            maxVal = fmaxf(maxVal, u[x0 * m[1] + (x1 + 1)]);

            u[x0 * m[1] + x1] =  maxVal + __logf(__expf(u[(x0 - 1) * m[1] + x1] - maxVal) +
                                                __expf(u[(x0 + 1) * m[1] + x1] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 - 1)] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 + 1)] - maxVal)) -
                                            1.38629436f; //This is equivalent to ln(2.0 * n) for n = 2.

            __syncthreads();
        }
    }
}


__global__ void harmonic_update_and_check_2d_gpu(unsigned int *m, float *u, unsigned int *locked,
                                            unsigned int currentIteration, float *delta)
{
    // Since float and unsigned int are 4 bytes each, and we need each array to be the size of
    // the number of threads, we will need to call this with: sizeof(float) * numThreads.
    // Note: blockDim.x == numThreads
    extern __shared__ float sdata[];
    float *deltaLocalMax = (float *)sdata;

    deltaLocalMax[threadIdx.x] = 0.0f;

    __syncthreads();

    for (unsigned int x0 = blockIdx.x; x0 < m[0]; x0 += gridDim.x) {
        unsigned int offset = (unsigned int)((currentIteration % 2) != (x0 % 2));

        for (unsigned int x1 = 2 * threadIdx.x + offset; x1 < m[1]; x1 += 2 * blockDim.x) {
            // If this is locked, then wait for the other threads to finish in the warp, then continue.
            if (locked[x0 * m[1] + x1]) {
                __syncthreads();
                continue;
            }

            float uPrevious = u[x0 * m[1] + x1];

            // Update the value at this location with the log-sum-exp trick.
            float maxVal = FLT_MIN;
            maxVal = fmaxf(u[(x0 - 1) * m[1] + x1], u[(x0 + 1) * m[1] + x1]);
            maxVal = fmaxf(maxVal, u[x0 * m[1] + (x1 - 1)]);
            maxVal = fmaxf(maxVal, u[x0 * m[1] + (x1 + 1)]);

            u[x0 * m[1] + x1] =  maxVal + __logf(__expf(u[(x0 - 1) * m[1] + x1] - maxVal) +
                                                __expf(u[(x0 + 1) * m[1] + x1] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 - 1)] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 + 1)] - maxVal)) -
                                            1.38629436f; //This is equivalent to ln(2.0 * n) for n = 2.

            // Compute the updated delta.
            deltaLocalMax[threadIdx.x] = fmaxf(deltaLocalMax[threadIdx.x], fabs(uPrevious - u[x0 * m[1] + x1]));

            __syncthreads();
        }
    }

    // At the end, perform a reduction to efficiently compute the maximal delta for this thread block.
    for (unsigned int index = blockDim.x / 2; index > 0; index >>= 1) {
        if (threadIdx.x < index) {
            if (deltaLocalMax[threadIdx.x] < deltaLocalMax[threadIdx.x + index]) {
                deltaLocalMax[threadIdx.x] = deltaLocalMax[threadIdx.x + index];
            }
        }

        __syncthreads();
    }

    // Store the maximal delta in the array for delta values. We will use another kernel to quickly
    // do a reduction over this to find the max delta.
    if (threadIdx.x == 0) {
        delta[blockIdx.x] = deltaLocalMax[0];
    }
}


__global__ void harmonic_compute_max_delta_gpu(unsigned int numBlocks, float *delta)
{
    // Stride this thread to compute its individual max.
    for (unsigned int i = threadIdx.x; i < numBlocks; i += blockDim.x) {
        delta[threadIdx.x] = fmaxf(delta[threadIdx.x], delta[i]);
    }

    __syncthreads();

    // Do a final reduction on these values to efficiently compute the true maximal delta.
    // Note: Afterwards, delta[0] will hold the max delta over all cells.
    for (unsigned int index = blockDim.x / 2; index > 0; index >>= 1) {
        if (threadIdx.x < index && threadIdx.x < numBlocks && threadIdx.x + index < numBlocks) {
            //delta[threadIdx.x] = fmaxf(delta[threadIdx.x], delta[threadIdx.x + index]);
            if (delta[threadIdx.x] < delta[threadIdx.x + index]) {
                delta[threadIdx.x] = delta[threadIdx.x + index];
            }
        }

        __syncthreads();
    }
}


unsigned int harmonic_compute_num_blocks_gpu(Harmonic *harmonic, unsigned int numThreads)
{
    if (harmonic->n == 2) {
        return harmonic->m[0];
    } else if (harmonic->n == 3) {
    } else if (harmonic->n == 4) {
    }

    return 0;
}


int harmonic_complete_gpu(Harmonic *harmonic, unsigned int numThreads)
{
    int result;

    result = harmonic_initialize_dimension_size_gpu(harmonic);
    if (result != EPIC_SUCCESS) {
        return result;
    }
    result = harmonic_initialize_potential_values_gpu(harmonic);
    if (result != EPIC_SUCCESS) {
        return result;
    }
    result = harmonic_initialize_locked_gpu(harmonic);
    if (result != EPIC_SUCCESS) {
        return result;
    }

    result = harmonic_execute_gpu(harmonic, numThreads);
    if (result != EPIC_SUCCESS) {
        return result;
    }

    result = EPIC_SUCCESS;
    if (harmonic_uninitialize_dimension_size_gpu(harmonic) != EPIC_SUCCESS) {
        result = EPIC_ERROR_DEVICE_FREE;
    }
    if (harmonic_uninitialize_potential_values_gpu(harmonic) != EPIC_SUCCESS) {
        result = EPIC_ERROR_DEVICE_FREE;
    }
    if (harmonic_uninitialize_locked_gpu(harmonic) != EPIC_SUCCESS) {
        result = EPIC_ERROR_DEVICE_FREE;
    }

    return result;
}


int harmonic_initialize_gpu(Harmonic *harmonic, unsigned int numThreads)
{
    // Reset the current iteration.
    harmonic->currentIteration = 0;

    // Ensure the data is valid.
    if (harmonic->n == 0 || harmonic->m == nullptr || harmonic->d_delta != nullptr) {
        fprintf(stderr, "Error[harmonic_initialize_gpu]: %s\n", "Invalid input.");
        return EPIC_ERROR_INVALID_DATA;
    }

    unsigned int numBlocks = harmonic_compute_num_blocks_gpu(harmonic, numThreads);

    // Allocate the memory on the device.
    if (cudaMalloc(&harmonic->d_delta, numBlocks * sizeof(float)) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_initialize_gpu]: %s\n",
                "Failed to allocate device-side memory for delta.");
        return EPIC_ERROR_DEVICE_MALLOC;
    }

    return EPIC_SUCCESS;
}


int harmonic_execute_gpu(Harmonic *harmonic, unsigned int numThreads)
{
    // The result from calling other functions.
    int result;

    // Ensure data is valid before we begin.
    if (harmonic == nullptr || harmonic->m == nullptr || harmonic->u == nullptr ||
            harmonic->locked == nullptr || harmonic->epsilon <= 0.0 ||
            harmonic->d_m == nullptr || harmonic->d_u == nullptr ||
            harmonic->d_locked == nullptr) {
        fprintf(stderr, "Error[harmonic_execute_gpu]: %s\n", "Invalid data.");
        return EPIC_ERROR_INVALID_DATA;
    }

    if (numThreads % 32 != 0) {
        fprintf(stderr, "Error[harmonic_execute_gpu]: %s\n",
                    "Must specficy a number of threads divisible by 32 (the number of threads in a warp).");
        return EPIC_ERROR_INVALID_CUDA_PARAM;
    }

    result = harmonic_initialize_gpu(harmonic, numThreads);
    if (result != EPIC_SUCCESS) {
        fprintf(stderr, "Error[harmonic_execute_gpu]: %s\n", "Failed to initialize GPU variables.");
        return result;
    }

    // Make sure 'information' can at least be propagated throughout the entire grid.
    unsigned int mMax = 0;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        mMax = std::max(mMax, harmonic->m[i]);
    }

    harmonic->delta = harmonic->epsilon + 1.0f;

    result = EPIC_SUCCESS;

    // Keep going until a threshold is reached.
    while (result != EPIC_SUCCESS_AND_CONVERGED || harmonic->currentIteration < mMax) {
        // We check for convergence on a staggered number of iterations.
        if (harmonic->currentIteration % harmonic->numIterationsToStaggerCheck == 0) {
            result = harmonic_update_and_check_gpu(harmonic, numThreads);
            if (result != EPIC_SUCCESS && result != EPIC_SUCCESS_AND_CONVERGED) {
                fprintf(stderr, "Error[harmonic_execute_gpu]: %s\n",
                                "Failed to perform the Gauss-Seidel update and check step.");
                return result;
            }
        } else {
            result = harmonic_update_gpu(harmonic, numThreads);
            if (result != EPIC_SUCCESS) {
                fprintf(stderr, "Error[harmonic_execute_gpu]: %s\n",
                                "Failed to perform the Gauss-Seidel update step.");
                return result;
            }
        }

        /* *** DEBUG ***
        if (harmonic->currentIteration % harmonic->numIterationsToStaggerCheck == 0) {
            printf("Iteration %i --- %e\n", harmonic->currentIteration, harmonic->delta);
            fflush(stdout);
        }
        //*/
    }

    result = harmonic_get_potential_values_gpu(harmonic);
    if (result != EPIC_SUCCESS) {
        fprintf(stderr, "Error[harmonic_execute_gpu]: %s\n", "Failed to get all the potential values.");
        return result;
    }

    result = harmonic_uninitialize_gpu(harmonic);
    if (result != EPIC_SUCCESS) {
        fprintf(stderr, "Error[harmonic_execute_gpu]: %s\n", "Failed to uninitialize GPU variables.");
        return result;
    }

    return EPIC_SUCCESS;
}


int harmonic_uninitialize_gpu(Harmonic *harmonic)
{
    int result;

    result = EPIC_SUCCESS;

    // Reset the current iteration.
    harmonic->currentIteration = 0;

    if (harmonic->d_delta != nullptr) {
        if (cudaFree(harmonic->d_delta) != cudaSuccess) {
            fprintf(stderr, "Error[harmonic_uninitialize_gpu]: %s\n",
                    "Failed to free device-side memory for delta.");
            result = EPIC_ERROR_DEVICE_FREE;
        }
    }
    harmonic->d_delta = nullptr;

    return result;
}


int harmonic_update_gpu(Harmonic *harmonic, unsigned int numThreads)
{
    unsigned int numBlocks = harmonic_compute_num_blocks_gpu(harmonic, numThreads);

    if (harmonic->n == 2) {
        harmonic_update_2d_gpu<<< numBlocks, numThreads >>>(harmonic->d_m, harmonic->d_u, harmonic->d_locked,
                                                            harmonic->currentIteration);
    } else if (harmonic->n == 3) {
    } else if (harmonic->n == 4) {
    }

    // Check if there was an error executing the kernel.
    if (cudaGetLastError() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_gpu]: %s\n",
                        "Failed to execute the 'Gauss-Seidel update' kernel.");
        return EPIC_ERROR_KERNEL_EXECUTION;
    }

    // Wait for the kernel to finish before looping more.
    if (cudaDeviceSynchronize() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_gpu]: %s\n",
                    "Failed to synchronize the device after 'Gauss-Seidel update' kernel.");
        return EPIC_ERROR_DEVICE_SYNCHRONIZE;
    }

    harmonic->currentIteration++;

    return EPIC_SUCCESS;
}


int harmonic_update_and_check_gpu(Harmonic *harmonic, unsigned int numThreads)
{
    // The number of blocks depends on n, m, and the number of threads.
    unsigned int numBlocks = harmonic_compute_num_blocks_gpu(harmonic, numThreads);

    if (harmonic->n == 2) {
        harmonic_update_and_check_2d_gpu<<< numBlocks, numThreads,
                                            numThreads * sizeof(float) >>>(
                                                harmonic->d_m, harmonic->d_u, harmonic->d_locked,
                                                harmonic->currentIteration, harmonic->d_delta);
    } else if (harmonic->n == 3) {
    } else if (harmonic->n == 4) {
    }

    // Check if there was an error executing the kernel.
    if (cudaGetLastError() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_and_check_gpu]: %s\n",
                        "Failed to execute the 'Gauss-Seidel update' kernel.");
        return EPIC_ERROR_KERNEL_EXECUTION;
    }

    // Wait for the kernel to finish before looping more.
    if (cudaDeviceSynchronize() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_and_check_gpu]: %s\n",
                    "Failed to synchronize the device after 'Gauss-Seidel update' kernel.");
        return EPIC_ERROR_DEVICE_SYNCHRONIZE;
    }

    harmonic_compute_max_delta_gpu<<< 1, numThreads >>>(numBlocks, harmonic->d_delta);

    // Check if there was an error executing the kernel.
    if (cudaGetLastError() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_and_check_gpu]: %s\n",
                        "Failed to execute the 'delta check update' kernel.");
        return EPIC_ERROR_KERNEL_EXECUTION;
    }

    // Wait for the kernel to finish before looping more.
    if (cudaDeviceSynchronize() != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_and_check_gpu]: %s\n",
                    "Failed to synchronize the device after 'delta check update' kernel.");
        return EPIC_ERROR_DEVICE_SYNCHRONIZE;
    }

    // Retrieve the max delta value to check for convergence. Note: The first value in d_delta holds the maximal value.
    if (cudaMemcpy(&harmonic->delta, harmonic->d_delta, 1 * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_and_check_gpu]: %s\n",
                "Failed to copy memory from device to host for the max delta.");
        return EPIC_ERROR_MEMCPY_TO_HOST;
    }

    harmonic->currentIteration++;

    if (harmonic->delta < harmonic->epsilon) {
        return EPIC_SUCCESS_AND_CONVERGED;
    } else {
        return EPIC_SUCCESS;
    }
}


int harmonic_get_potential_values_gpu(Harmonic *harmonic)
{
    // Compute the number of cells.
    unsigned int numCells = 1;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        numCells *= harmonic->m[i];
    }

    // Copy the final (or intermediate) result from device to host.
    if (cudaMemcpy(harmonic->u, harmonic->d_u, numCells * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_get_potential_values_gpu]: %s\n",
                "Failed to copy memory from device to host for the potential values.");
        return EPIC_ERROR_MEMCPY_TO_HOST;
    }

    return EPIC_SUCCESS;
}

