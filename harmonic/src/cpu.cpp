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
#include <math.h>

#include "../include/cpu.h"

int cpu_harmonic_jacobi_2d(const unsigned int *m, float **u, float epsilon)
{
	// Ensure that valid data was passed.
	if (m == nullptr || u == nullptr || epsilon <= 0.0f) {
		std::cerr << "Error[cpu_harmonic_jacobi_2d]: Invalid data." << std::endl;
		return 1;
	}

	// Create a copy of u to oscillate between u and uPrime during iteration.
	float **uPrime = new float *[m[0]];
	for (int i = 0; i < m[0]; i++) {
		uPrime[i] = new float[m[1]];
		for (int j = 0; j < m[1]; j++) {
			uPrime[i][j] = u[i][j];
		}
	}

	unsigned int iterations = 0;
	float delta = epsilon + 1.0f;

	while (delta > epsilon || (delta <= epsilon && iterations % 2 == 1)) {
		delta = 0.0f;

		for (unsigned int i = 0; i < m[0]; i++) {
			for (unsigned int j = 0; j < m[1]; j++) {
				// If this is assigned a value, then skip it.
				if (signbit(u[i][j])) {
					continue;
				}

				// Compute the offsets, and ensure it does not go out of bounds.
				unsigned int ip = std::min(m[0] - 1, i + 1);
				unsigned int im = std::max(0, (int)i - 1);
				unsigned int jp = std::min(m[1] - 1, j + 1);
				unsigned int jm = std::max(0, (int)j - 1);

				// Swap between updating u and uPrime.
				if (iterations % 2 == 0) {
					uPrime[i][j] = 0.25f * (fabs(u[ip][j]) + fabs(u[im][j]) + fabs(u[i][jp]) + fabs(u[i][jm]));
				} else {
					u[i][j] = 0.25f * (fabs(uPrime[ip][j]) + fabs(uPrime[im][j]) + fabs(uPrime[i][jp]) + fabs(uPrime[i][jm]));
				}

				// Compute delta, the difference between this iteration and the previous iteration.
				delta = std::max(delta, (float)fabs(u[i][j] - uPrime[i][j]));
			}
		}

		iterations++;
	}

	std::cout << "Completed in " << iterations << " iterations." << std::endl;

	// Free the memory allocated!
	for (int i = 0; i < m[0]; i++) {
		delete [] uPrime[i];
	}
	delete [] uPrime;

	return 0;
}

int cpu_harmonic_gauss_seidel_2d(const unsigned int *m, float **u, float epsilon)
{
	// Ensure that valid data was passed.
	if (m == nullptr || u == nullptr || epsilon <= 0.0f) {
		std::cerr << "Error[cpu_harmonic_gauss_seidel_2d]: Invalid data." << std::endl;
		return 1;
	}

	unsigned int iterations = 0;
	float delta = epsilon + 1.0f;

	while (delta > epsilon) {
		delta = 0.0f;

		for (unsigned int i = 0; i < m[0]; i++) {
			for (unsigned int j = 0; j < m[1]; j++) {
				// If this is assigned a value, then skip it.
				if (signbit(u[i][j])) {
					continue;
				}

				// Compute the offsets, and ensure it does not go out of bounds.
				unsigned int ip = std::min(m[0] - 1, i + 1);
				unsigned int im = std::max(0, (int)i - 1);
				unsigned int jp = std::min(m[1] - 1, j + 1);
				unsigned int jm = std::max(0, (int)j - 1);

				float old = u[i][j];

				// Note: By construction of the for-loop, im and jm are actually the next iteration.
				u[i][j] = 0.25f * (fabs(u[ip][j]) + fabs(u[im][j]) + fabs(u[i][jp]) + fabs(u[i][jm]));

				// Compute delta, the difference between this iteration and the previous iteration.
				delta = std::max(delta, (float)fabs(u[i][j] - old));
			}
		}

		iterations++;
	}

	std::cout << "Completed in " << iterations << " iterations." << std::endl;

	return 0;
}

int cpu_harmonic_sor_2d(const unsigned int *m, float **u, float epsilon, float omega)
{
	// Ensure that valid data was passed.
	if (m == nullptr || u == nullptr || epsilon <= 0.0f) {
		std::cerr << "Error[cpu_harmonic_gauss_seidel_2d]: Invalid data." << std::endl;
		return 1;
	}

	unsigned int iterations = 0;
	float delta = epsilon + 1.0f;

	while (delta > epsilon) {
		delta = 0.0f;

		for (unsigned int i = 0; i < m[0]; i++) {
			for (unsigned int j = 0; j < m[1]; j++) {
				// If this is assigned a value, then skip it.
				if (signbit(u[i][j])) {
					continue;
				}

				// Compute the offsets, and ensure it does not go out of bounds.
				unsigned int ip = std::min(m[0] - 1, i + 1);
				unsigned int im = std::max(0, (int)i - 1);
				unsigned int jp = std::min(m[1] - 1, j + 1);
				unsigned int jm = std::max(0, (int)j - 1);

				float old = u[i][j];

				// Note: By construction of the for-loop, im and jm are actually the next iteration.
				u[i][j] += omega * 0.25f * (fabs(u[ip][j]) + fabs(u[im][j]) + fabs(u[i][jp]) + fabs(u[i][jm]) - 4.0f * fabs(u[i][j]));

				// Compute delta, the difference between this iteration and the previous iteration.
				delta = std::max(delta, (float)fabs(u[i][j] - old));
			}
		}

		iterations++;
	}

	std::cout << "Completed in " << iterations << " iterations." << std::endl;

	return 0;
}
