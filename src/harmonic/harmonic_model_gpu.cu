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


#include <epic/harmonic/harmonic.h>
#include <epic/harmonic/harmonic_model_gpu.h>
#include <epic/error_codes.h>

#include <stdio.h>


namespace epic {

int harmonic_initialize_dimension_size_gpu(Harmonic *harmonic)
{
    // Ensure the data is valid.
    if (harmonic->n == 0 || harmonic->m == nullptr) {
        fprintf(stderr, "Error[harmonic_initialize_dimension_size_gpu]: %s\n", "Invalid input.");
        return EPIC_ERROR_INVALID_DATA;
    }

    // Allocate the memory on the device.
    if (cudaMalloc(&harmonic->d_m, harmonic->n * sizeof(unsigned int)) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_initialize_dimension_size_gpu]: %s\n",
                "Failed to allocate device-side memory for the dimension size.");
        return EPIC_ERROR_DEVICE_MALLOC;
    }

    // Copy the data from the host to the device.
    if (cudaMemcpy(harmonic->d_m, harmonic->m, harmonic->n * sizeof(unsigned int),
                    cudaMemcpyHostToDevice) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_initialize_dimension_size_gpu]: %s\n",
                "Failed to copy memory from host to device for the dimension size.");
        return EPIC_ERROR_MEMCPY_TO_DEVICE;
    }

    return EPIC_SUCCESS;
}


int harmonic_uninitialize_dimension_size_gpu(Harmonic *harmonic)
{
    if (harmonic->d_m != nullptr) {
        if (cudaFree(harmonic->d_m) != cudaSuccess) {
            fprintf(stderr, "Error[harmonic_uninitialize_dimension_size_gpu]: %s\n",
                    "Failed to free device-side memory for the dimension size.");
            return EPIC_ERROR_DEVICE_FREE;
        }
    }
    harmonic->d_m = nullptr;

    return EPIC_SUCCESS;
}


int harmonic_initialize_potential_values_gpu(Harmonic *harmonic)
{
    // Ensure the data is valid.
    if (harmonic->n == 0 || harmonic->m == nullptr || harmonic->u == nullptr) {
        fprintf(stderr, "Error[harmonic_initialize_potential_values_gpu]: %s\n", "Invalid input.");
        return EPIC_ERROR_INVALID_DATA;
    }

    // Compute the number of cells.
    unsigned int numCells = 1;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        numCells *= harmonic->m[i];
    }

    // Allocate the memory on the device.
    if (cudaMalloc(&harmonic->d_u, numCells * sizeof(float)) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_initialize_potential_values_gpu]: %s\n",
                "Failed to allocate device-side memory for the potential values.");
        return EPIC_ERROR_DEVICE_MALLOC;
    }

    // Copy the data from the host to the device.
    if (cudaMemcpy(harmonic->d_u, harmonic->u, numCells * sizeof(float),
                    cudaMemcpyHostToDevice) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_initialize_potential_values_gpu]: %s\n",
                "Failed to copy memory from host to device for the potential values.");
        return EPIC_ERROR_MEMCPY_TO_DEVICE;
    }

    return EPIC_SUCCESS;
}


int harmonic_uninitialize_potential_values_gpu(Harmonic *harmonic)
{
    if (harmonic->d_u != nullptr) {
        if (cudaFree(harmonic->d_u) != cudaSuccess) {
            fprintf(stderr, "Error[harmonic_uninitialize_potential_values_gpu]: %s\n",
                    "Failed to free device-side memory for the potential values.");
            return EPIC_ERROR_DEVICE_FREE;
        }
    }
    harmonic->d_u = nullptr;

    return EPIC_SUCCESS;
}


int harmonic_initialize_locked_gpu(Harmonic *harmonic)
{
    // Ensure the data is valid.
    if (harmonic->n == 0 || harmonic->m == nullptr || harmonic->locked == nullptr) {
        fprintf(stderr, "Error[harmonic_initialize_locked_gpu]: %s\n", "Invalid input.");
        return EPIC_ERROR_INVALID_DATA;
    }

    // Compute the number of cells.
    unsigned int numCells = 1;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        numCells *= harmonic->m[i];
    }

    // Allocate the memory on the device.
    if (cudaMalloc(&harmonic->d_locked, numCells * sizeof(unsigned int)) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_initialize_locked_gpu]: %s\n",
                "Failed to allocate device-side memory for the locked cells.");
        return EPIC_ERROR_DEVICE_MALLOC;
    }

    // Copy the data from the host to the device.
    if (cudaMemcpy(harmonic->d_locked, harmonic->locked, numCells * sizeof(unsigned int),
                    cudaMemcpyHostToDevice) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_initialize_locked_gpu]: %s\n",
                "Failed to copy memory from host to device for the locked cells.");
        return EPIC_ERROR_MEMCPY_TO_DEVICE;
    }

    return EPIC_SUCCESS;
}


int harmonic_uninitialize_locked_gpu(Harmonic *harmonic)
{
    if (harmonic->d_locked != nullptr) {
        if (cudaFree(harmonic->d_locked) != cudaSuccess) {
            fprintf(stderr, "Error[harmonic_uninitialize_locked_gpu]: %s\n",
                    "Failed to free device-side memory for the locked cells.");
            return EPIC_ERROR_DEVICE_FREE;
        }
    }
    harmonic->d_locked = nullptr;

    return EPIC_SUCCESS;
}


int harmonic_update_model_gpu(Harmonic *harmonic)
{
    if (harmonic->n == 0 || harmonic->m == nullptr ||
            harmonic->u == nullptr || harmonic->d_u == nullptr ||
            harmonic->locked == nullptr || harmonic->d_locked == nullptr) {
        fprintf(stderr, "Error[harmonic_update_model_gpu]: %s\n", "Invalid data.");
        return EPIC_ERROR_INVALID_DATA;
    }

    // Compute the number of cells.
    unsigned int numCells = 1;
    for (unsigned int i = 0; i < harmonic->n; i++) {
        numCells *= harmonic->m[i];
    }

    // Copy the data from the host to the device.
    if (cudaMemcpy(harmonic->d_u, harmonic->u, numCells * sizeof(float),
                    cudaMemcpyHostToDevice) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_model_gpu]: %s\n",
                "Failed to copy memory from host to device for the potential values.");
        return EPIC_ERROR_MEMCPY_TO_DEVICE;
    }

    // Copy the data from the host to the device.
    if (cudaMemcpy(harmonic->d_locked, harmonic->locked, numCells * sizeof(unsigned int),
                    cudaMemcpyHostToDevice) != cudaSuccess) {
        fprintf(stderr, "Error[harmonic_update_model_gpu]: %s\n",
                "Failed to copy memory from host to device for the locked cells.");
        return EPIC_ERROR_MEMCPY_TO_DEVICE;
    }

    return EPIC_SUCCESS;
}

}; // namespace epic

