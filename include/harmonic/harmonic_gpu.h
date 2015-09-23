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


#ifndef HARMONIC_GPU_H
#define HARMONIC_GPU_H


/**
 *  Execute a Gauss-Seidel harmonic function solver until convergence. Uses the GPU (CUDA).
 *  @param  harmonic    The Harmonic object.
 *  @param  numThreads  The number of threads, as a multiple of 32 (e.g., 1024).
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_complete_gpu(Harmonic *harmonic, unsigned int numThreads);

/**
 *  Step 1/3: Initialize the GPU-specific variables for the Gauss-Seidel GPU implementation.
 *  @param  harmonic    The Harmonic object.
 *  @param  numThreads  The number of threads, as a multiple of 32 (e.g., 1024).
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_initialize_gpu(Harmonic *harmonic, unsigned int numThreads);

/**
 *  Step 2/3: Execute the Gauss-Seidel GPU update step until convergence.
 *  @param  harmonic    The Harmonic object.
 *  @param  numThreads  The number of threads, as a multiple of 32 (e.g., 1024).
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_execute_gpu(Harmonic *harmonic, unsigned int numThreads);

/**
 *  Step 3/3: Uninitialize the GPU-specific variables for the Gauss-Seidel GPU implementation.
 *  @param  harmonic    The Harmonic object.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_uninitialize_gpu(Harmonic *harmonic);

/**
 *  Perform a single update step of the Gauss-Seidel GPU implementation.
 *  @param  harmonic    The Harmonic object.
 *  @param  numThreads  The number of threads, as a multiple of 32 (e.g., 1024).
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_update_gpu(Harmonic *harmonic, unsigned int numThreads);

/**
 *  Perform a single update step of the Gauss-Seidel GPU implementation, plus check
 *  for convergence of the iteration.
 *  @param  harmonic    The Harmonic object.
 *  @param  numThreads  The number of threads, as a multiple of 32 (e.g., 1024).
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_update_and_check_gpu(Harmonic *harmonic, unsigned int numThreads);

/**
 *  Copy the potential values from the GPU-side memory to the CPU-side memory.
 *  @param  harmonic    The Harmonic object.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_get_potential_values_gpu(Harmonic *harmonic);


#endif // HARMONIC_GPU_H

