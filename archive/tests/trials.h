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


#ifndef TRIALS_H
#define TRIALS_H


#include <iostream>
#include <cstdlib>
#include <sys/time.h>

#include "worlds.h"
#include "naive.h"
#include "cpu.h"
#include "gpu_jacobi_v1.h"
#include "gpu_jacobi_v2.h"
#include "gpu_jacobi.h"

#include <string>
#include <vector>

/**
 * Return the current time in milliseconds.
 */
long long get_current_time();

/**
 * Compute some statistics of the given samples.
 * @param	samples		A vector of samples.
 * @return	A comma-delimited string of mean and 95% confidence interval.
 */
std::string compute_statistics(std::vector<double> samples);

/**
 * Perform a single 2D trial, given the number of arguments.
 * @return	Returns 0 if successful; non-zero otherwise.
 */
int trials_2d(unsigned int numBlocks, unsigned int numThreads, unsigned int stagger, float epsilon,
		unsigned int minSize, unsigned int maxSize, unsigned int stepSize,
		unsigned int numObstacles, unsigned int numExecutions);

/**
 * Perform a single 2D trial, for sanity check comparison, and output the resultant function.
 * @return	Returns 0 if successful; non-zero otherwise.
 */
int single_trial_2d();

/**
 * Perform a single 3D trial, given the number of arguments.
 * @return	Returns 0 if successful; non-zero otherwise.
 */
int trials_3d(unsigned int numBlocksX, unsigned int numBlocksY, unsigned int numThreads,
		unsigned int stagger,
		float epsilon, unsigned int minSize, unsigned int maxSize, unsigned int stepSize,
		unsigned int numObstacles, unsigned int numExecutions);

/**
 * Perform a single 3D trial, for sanity check comparison, and output the resultant function.
 * @return	Returns 0 if successful; non-zero otherwise.
 */
int single_trial_3d();

/**
 * Perform a single 4D trial, given the number of arguments.
 * @return	Returns 0 if successful; non-zero otherwise.
 */
int trials_4d(unsigned int numBlocksX, unsigned int numBlocksY, unsigned int numBlocksZ, unsigned int numThreads,
		unsigned int stagger,
		float epsilon, unsigned int minSize, unsigned int maxSize, unsigned int stepSize,
		unsigned int numObstacles, unsigned int numExecutions);


#endif // TRIALS_H
