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
#include "error_codes.h"
#include "constants.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>

__global__ void harmonic_2d_update_gpu(unsigned int *m, float *u, unsigned int *locked, unsigned int currentIteration)
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
            maxVal = max(u[(x0 - 1) * m[1] + x1], u[(x0 + 1) * m[1] + x1]);
            maxVal = max(maxVal, u[x0 * m[1] + (x1 - 1)]);
            maxVal = max(maxVal, u[x0 * m[1] + (x1 + 1)]);

            u[x0 * m[1] + x1] =  maxVal + __logf(__expf(u[(x0 - 1) * m[1] + x1] - maxVal) +
                                                __expf(u[(x0 + 1) * m[1] + x1] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 - 1)] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 + 1)] - maxVal)) -
                                            1.38629436f; //This is equivalent to ln(2.0 * n) for n = 2.

            __syncthreads();
        }
    }
}

/*__global__ void harmonic_2d_update_and_check_gpu(unsigned int *m, float *u, unsigned int *locked, unsigned int currentIteration)
{
    // Since float and unsigned int are 4 bytes each, and we need each array to be the size of
    // the number of threads, we will need to call this with: sizeof(float) * numThreads.
    // Note: blockDim.x == numThreads
    extern __shared__ float sdata[];
    float *deltaLocalMax = (float *)sdata;

    deltaLocalMax[threaIdx.x] = 0.0f;

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
            maxVal = max(u[(x0 - 1) * m[1] + x1], u[(x0 + 1) * m[1] + x1]);
            maxVal = max(maxVal, u[x0 * m[1] + (x1 - 1)]);
            maxVal = max(maxVal, u[x0 * m[1] + (x1 + 1)]);

            u[x0 * m[1] + x1] =  maxVal + __logf(__expf(u[(x0 - 1) * m[1] + x1] - maxVal) +
                                                __expf(u[(x0 + 1) * m[1] + x1] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 - 1)] - maxVal) +
                                                __expf(u[x0 * m[1] + (x1 + 1)] - maxVal)) -
                                            1.38629436f; //This is equivalent to ln(2.0 * n) for n = 2.

            // Compute the updated delta.
            deltaLocalMax[threadIdx.x] = max(deltaLocalMax[threadIdx.x], fabs(uPrevious - u[x0 * m[1] + x1]));

            __syncthreads();
        }
    }

    // At the end, perform a reduction to efficiently compute the maximal delta for this thread block.
    for (unsigned int index = blockDim.x / 2; index > 0; index >>= 1) {
        if (threadIdx.x < index && threadIdx.x < m[1] && threadIdx.x + index < m[1]) {
            if (deltaLocalMax[threadIdx.x] < deltaLocalMax[threadIdx.x + index]) {
                deltaLocalMax[threadIdx.x] = deltaLocalMax[threadIdx.x + index];
            }
        }

        __syncthreads();
    }

    // Store the maximal delta in the array for delta values. We will use another kernel to quickly
    // do a reduction over this to find the max delta.
    if (threadIdx.x == 0) {
        deltaMax[blockIdx.x] = deltaLocalMax[0];
    }
}
*/

int harmonic_2d_gpu(Harmonic *harmonic, unsigned int numThreads)
{
    // Ensure data is valid before we begin.
    if (harmonic == nullptr || harmonic->m == nullptr || harmonic->u == nullptr ||
            harmonic->locked == nullptr || harmonic->epsilon <= 0.0 ||
            harmonic->d_m == nullptr || harmonic->d_u == nullptr ||
            harmonic->d_locked == nullptr) {
        fprintf(stderr, "Error[harmonic_2d_gpu]: %s\n", "Invalid data.");
        return INERTIA_ERROR_INVALID_DATA;
    }

    if (numThreads % 32 != 0) {
        fprintf(stderr, "Error[harmonic_2d_gpu]: %s\n",
                    "Must specficy a number of threads divisible by 32 (the number of threads in a warp).");
        return INERTIA_ERROR_INVALID_CUDA_PARAM;
    }

    // Make sure 'information' can at least be propagated throughout the entire grid.
    unsigned int mMax = 0;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        mMax = std::max(mMax, harmonic->m[i]);
    }
    mMax = 5000; // *** DEBUG ***

    harmonic->currentIteration = 0;


    // Determine how many blocks are required.
    unsigned int numBlocks = harmonic->m[0];

    // Keep going until a threshold is reached.
    while (/*delta > harmonic->epsilon ||*/ harmonic->currentIteration < mMax) {
        //delta = 0.0;

        harmonic_2d_update_gpu<<< numBlocks, numThreads >>>(harmonic->d_m, harmonic->d_u, harmonic->d_locked, harmonic->currentIteration);

        // *** DEBUG ***
        if (harmonic->currentIteration % 100 == 0) {
            //printf("Iteration %i --- %e\n", harmonic->currentIteration, delta);
            printf("Iteration %i\n", harmonic->currentIteration);
            fflush(stdout);
        }
        // *************

        harmonic->currentIteration++;
    }

    // Compute the number of cells.
    unsigned int numCells = 1;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        numCells *= harmonic->m[i];
    }

    // Copy the final (or intermediate) result from device to host.
    if (cudaMemcpy(harmonic->u, harmonic->d_u, numCells * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_2d_gpu]: %s\n",
                "Failed to copy memory from device to host for the potential values.");
        return INERTIA_ERROR_MEMCPY_TO_HOST;
    }

    return INERTIA_SUCCESS;
}

//int harmonic_3d_gpu(Harmonic *harmonic, unsigned int numThreads);

//int harmonic_4d_gpu(Harmonic *harmonic, unsigned int numThreads);



