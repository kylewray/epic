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

int cpu_harmonic_jacobi_2d(const unsigned int *m, float *u, float epsilon)
{
	// Ensure that valid data was passed.
	if (m == nullptr || u == nullptr || epsilon <= 0.0f) {
		std::cerr << "Error[cpu_harmonic_jacobi_2d]: Invalid data." << std::endl;
		return 1;
	}

	// Create a copy of u to oscillate between u and uPrime during iteration.
	float *uPrime = new float [m[0] * m[1]];
	for (int i = 0; i < m[0] * m[1]; i++) {
		uPrime[i] = u[i];
	}

	unsigned int iterations = 0;
	float delta = epsilon + 1.0f;

	while (delta > epsilon || (delta <= epsilon && iterations % 2 == 1)) {
		delta = 0.0f;

		for (unsigned int i = 0; i < m[0]; i++) {
			for (unsigned int j = 0; j < m[1]; j++) {
				// If this is assigned a value, then skip it.
				if (signbit(u[i * m[1] + j])) {
					continue;
				}

				// Compute the offsets, and ensure it does not go out of bounds.
				unsigned int ip = std::min(m[0] - 1, i + 1);
				unsigned int im = std::max(0, (int)i - 1);
				unsigned int jp = std::min(m[1] - 1, j + 1);
				unsigned int jm = std::max(0, (int)j - 1);

				// Swap between updating u and uPrime.
				if (iterations % 2 == 0) {
					uPrime[i * m[1] + j] = 0.25f * (fabs(u[ip * m[1] + j]) + fabs(u[im * m[1] + j]) + fabs(u[i * m[1] + jp]) + fabs(u[i * m[1] + jm]));
				} else {
					u[i * m[1] + j] = 0.25f * (fabs(uPrime[ip * m[1] + j]) + fabs(uPrime[im * m[1] + j]) + fabs(uPrime[i * m[1] + jp]) + fabs(uPrime[i * m[1] + jm]));
				}

				// Compute delta, the difference between this iteration and the previous iteration.
				delta = std::max(delta, (float)fabs(u[i * m[1] + j] - uPrime[i * m[1] + j]));
			}
		}

		iterations++;
	}

//	std::cout << "CPU Jacobi 2D: Completed in " << iterations << " iterations." << std::endl;

	// Free the memory allocated!
	delete [] uPrime;

	return 0;
}

int cpu_harmonic_gauss_seidel_2d(const unsigned int *m, float *u, float epsilon)
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
				if (signbit(u[i * m[1] + j])) {
					continue;
				}

				// Compute the offsets, and ensure it does not go out of bounds.
				unsigned int ip = std::min(m[0] - 1, i + 1);
				unsigned int im = std::max(0, (int)i - 1);
				unsigned int jp = std::min(m[1] - 1, j + 1);
				unsigned int jm = std::max(0, (int)j - 1);

				float old = u[i * m[1] + j];

				// Note: By construction of the for-loop, im and jm are actually the next iteration.
				u[i * m[1] + j] = 0.25f * (fabs(u[ip * m[1] + j]) + fabs(u[im * m[1] + j]) + fabs(u[i * m[1] + jp]) + fabs(u[i * m[1] + jm]));

				// Compute delta, the difference between this iteration and the previous iteration.
				delta = std::max(delta, (float)fabs(u[i * m[1] + j] - old));
			}
		}

		iterations++;
	}

//	std::cout << "CPU Gauss-Seidel 2D: Completed in " << iterations << " iterations." << std::endl;

	return 0;
}

int cpu_harmonic_sor_2d(const unsigned int *m, float *u, float epsilon, float omega)
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
				if (signbit(u[i * m[1] + j])) {
					continue;
				}

				// Compute the offsets, and ensure it does not go out of bounds.
				unsigned int ip = std::min(m[0] - 1, i + 1);
				unsigned int im = std::max(0, (int)i - 1);
				unsigned int jp = std::min(m[1] - 1, j + 1);
				unsigned int jm = std::max(0, (int)j - 1);

				float old = u[i * m[1] + j];

				// Note: By construction of the for-loop, im and jm are actually the next iteration.
				u[i * m[1] + j] += omega * 0.25f * (fabs(u[ip * m[1] + j]) + fabs(u[im * m[1] + j]) + fabs(u[i * m[1] + jp]) + fabs(u[i * m[1] + jm]) - 4.0f * fabs(u[i * m[1] + j]));

				// Compute delta, the difference between this iteration and the previous iteration.
				delta = std::max(delta, (float)fabs(u[i * m[1] + j] - old));
			}
		}

		iterations++;
	}

//	std::cout << "CPU SOR 2D: Completed in " << iterations << " iterations." << std::endl;

	return 0;
}

int cpu_harmonic_sor_3d(const unsigned int *m, float *u, float epsilon, float omega)
{
	// Ensure that valid data was passed.
	if (m == nullptr || u == nullptr || epsilon <= 0.0f) {
		std::cerr << "Error[cpu_harmonic_gauss_seidel_3d]: Invalid data." << std::endl;
		return 1;
	}

	unsigned int iterations = 0;
	float delta = epsilon + 1.0f;

	while (delta > epsilon) {
		delta = 0.0f;

		for (unsigned int i = 0; i < m[0]; i++) {
			for (unsigned int j = 0; j < m[1]; j++) {
				for (unsigned int k = 0; k < m[2]; k++) {
					// If this is assigned a value, then skip it.
					if (signbit(u[i * m[1] * m[2] + j * m[2] + k])) {
						continue;
					}

					// Compute the offsets, and ensure it does not go out of bounds.
					unsigned int ip = std::min(m[0] - 1, i + 1);
					unsigned int im = std::max(0, (int)i - 1);
					unsigned int jp = std::min(m[1] - 1, j + 1);
					unsigned int jm = std::max(0, (int)j - 1);
					unsigned int kp = std::min(m[2] - 1, k + 1);
					unsigned int km = std::max(0, (int)k - 1);

					float old = u[i * m[1] * m[2] + j * m[2] + k];

					// Note: By construction of the for-loop, im, jm, and km are actually the next iteration.
					u[i * m[1] * m[2] + j * m[2] + k] += omega * 0.166666666667f * (
							fabs(u[ip * m[1] * m[2] + j * m[2] + k]) +
							fabs(u[im * m[1] * m[2] + j * m[2] + k]) +
							fabs(u[i * m[1] * m[2] + jp * m[2] + k]) +
							fabs(u[i * m[1] * m[2] + jm * m[2] + k]) +
							fabs(u[i * m[1] * m[2] + j * m[2] + kp]) +
							fabs(u[i * m[1] * m[2] + j * m[2] + km]) -
							6.0f * fabs(u[i * m[1] * m[2] + j * m[2] + k]));

					// Compute delta, the difference between this iteration and the previous iteration.
					delta = std::max(delta, (float)fabs(u[i * m[1] * m[2] + j * m[2] + k] - old));
				}
			}
		}

		iterations++;
	}

//	std::cout << "CPU SOR 3D: Completed in " << iterations << " iterations." << std::endl;

	return 0;
}


int cpu_harmonic_sor_4d(const unsigned int *m, float *u, float epsilon, float omega)
{
	// Ensure that valid data was passed.
	if (m == nullptr || u == nullptr || epsilon <= 0.0f) {
		std::cerr << "Error[cpu_harmonic_gauss_seidel_4d]: Invalid data." << std::endl;
		return 1;
	}

	unsigned int iterations = 0;
	float delta = epsilon + 1.0f;

	while (delta > epsilon) {
		delta = 0.0f;

		for (unsigned int i = 0; i < m[0]; i++) {
			for (unsigned int j = 0; j < m[1]; j++) {
				for (unsigned int k = 0; k < m[2]; k++) {
					for (unsigned int l = 0; l < m[3]; l++) {
						// If this is assigned a value, then skip it.
						if (signbit(u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l])) {
							continue;
						}

						// Compute the offsets, and ensure it does not go out of bounds.
						unsigned int ip = std::min(m[0] - 1, i + 1);
						unsigned int im = std::max(0, (int)i - 1);
						unsigned int jp = std::min(m[1] - 1, j + 1);
						unsigned int jm = std::max(0, (int)j - 1);
						unsigned int kp = std::min(m[2] - 1, k + 1);
						unsigned int km = std::max(0, (int)k - 1);
						unsigned int lp = std::min(m[3] - 1, l + 1);
						unsigned int lm = std::max(0, (int)l - 1);

						float old = u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l];

						// Note: By construction of the for-loop, im, jm, km, and lm are actually the next iteration.
						u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l] += omega * 0.125f * (
								fabs(u[ip * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l]) +
								fabs(u[im * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l]) +
								fabs(u[i * m[1] * m[2] * m[3] + jp * m[2] * m[3] + k * m[3] + l]) +
								fabs(u[i * m[1] * m[2] * m[3] + jm * m[2] * m[3] + k * m[3] + l]) +
								fabs(u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + kp * m[3] + l]) +
								fabs(u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + km * m[3] + l]) +
								fabs(u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + lp]) +
								fabs(u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + lm]) -
								8.0f * fabs(u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l]));

						// Compute delta, the difference between this iteration and the previous iteration.
						delta = std::max(delta, (float)fabs(u[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l] - old));
					}
				}
			}
		}

		iterations++;
	}

//	std::cout << "CPU SOR 4D: Completed in " << iterations << " iterations." << std::endl;

	return 0;
}
