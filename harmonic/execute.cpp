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


#include <stdio.h>
#include <math.h>
#include <cstdlib>

#include "naive.h"

void create_simple_world(unsigned int *&m, float *&h)
{
	m = new unsigned int[2];
	m[0] = 10; m[1] = 10;

	h = new float[m[0] * m[1]];

	// By default, all are zero without signed bit flipped.
	for (int i = 0; i < m[0] / 2; i++) {
		for (int j = 0; j < m[1] / 2; j++) {
			h[i * m[1] + j] = 0.0f;
		}
	}

	// Boundaries.
	for (int i = 0; i < m[0]; i++) {
		h[i * m[1] + 0] = -1.0f;
		h[i * m[1] + m[1] - 1] = -1.0f;
	}
	for (int j = 0; j < m[1]; j++) {
		h[0 * m[1] + j] = -1.0f;
		h[(m[0] - 1) * m[1] + j] = -1.0f;
	}
	for (int i = 0; i < m[0] / 2; i++) {
		for (int j = 0; j < m[1] / 2; j++) {
			h[i * m[1] + j] = -1.0f;
		}
	}

	// Goal cells.
	h[4 * m[1] + 2] = -0.0f;
	h[4 * m[1] + 3] = -0.0f;
}

void create_variable_world(unsigned int mRows, unsigned int mCols, unsigned int *&m, float *&h,
		unsigned int numBoxes, unsigned int maxSize)
{
	m = new unsigned int[2];
	m[0] = mRows; m[1] = mCols;

	h = new float[m[0] * m[1]];

	// By default, all are zero without signed bit flipped.
	for (int i = 0; i < m[0] / 2; i++) {
		for (int j = 0; j < m[1] / 2; j++) {
			h[i * m[1] + j] = 0.0f;
		}
	}

	// Boundaries.
	for (int i = 0; i < m[0]; i++) {
		h[i * m[1] + 0] = -1.0f;
		h[i * m[1] + m[1] - 1] = -1.0f;
	}
	for (int j = 0; j < m[1]; j++) {
		h[0 * m[1] + j] = -1.0f;
		h[(m[0] - 1) * m[1] + j] = -1.0f;
	}

	// Random boxes.
	for (int k = 0; k < numBoxes; k++) {
		unsigned int r = (unsigned int)((float)(m[0] - maxSize - 1) * rand() / (float)RAND_MAX);
		unsigned int c = (unsigned int)((float)(m[1] - maxSize - 1) * rand() / (float)RAND_MAX);
		unsigned int x = (unsigned int)((float)maxSize * rand() / (float)RAND_MAX) + 1;
		unsigned int y = (unsigned int)((float)maxSize * rand() / (float)RAND_MAX) + 1;

		for (unsigned int i = r; i < r + y; i++) {
			for (unsigned int j = c; j < c + x; j++) {
				h[i * m[1] + j] = -1.0f;
			}
		}
	}

	// Randomly pick a goal point.
	unsigned int r = (unsigned int)((float)m[0] * rand() / (float)RAND_MAX);
	unsigned int c = (unsigned int)((float)m[1] * rand() / (float)RAND_MAX);

	while (h[r * m[1] + c] < 0.0f) {
		r = (unsigned int)((float)m[0] * rand() / (float)RAND_MAX);
		c = (unsigned int)((float)m[1] * rand() / (float)RAND_MAX);
	}
	h[r * m[1] + c] = -0.0f;

	printf("Goal Cell: (%i, %i)\n", r, c);
}

void print_world(unsigned int *m, float *d_u)
{
	float *u = new float[m[0] * m[1]];
	harmonic_get(2, m, d_u, u);

	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			unsigned long long int index = i * m[1] + j;
			if (fabs(u[index]) == 0.0f) {
				printf("++++ ");
			} else if (u[index] < 0.0f) {
					printf("     ");
			} else {
				printf("%.0f ", 9999.0f * fabs(u[index]));
			}
		}
		printf("\n");
	}

	delete [] u;
}

int main(int argc, char *argv[])
{
	unsigned int *m = nullptr;
	float *h = nullptr;

	srand(1337);

	//* ----- Simple World -----
	create_simple_world(m, h);
	//*/

	/* ----- Variable World -----
	create_variable_world(100, 30, m, h, 5, 10);
	//*/

	unsigned int *d_m;
	float *d_u;

	unsigned int b[3];
	b[0] = 3; b[1] = 3; b[2] = 3; // Equivalent to 3 * 3 * 3 = 18 blocks.
	unsigned int t[3];
	t[0] = 8; t[1] = 8; t[2] = 8; // Equivalent to 8 * 8 * 8 = 512 threads.

	if (harmonic_alloc(2, m, h, d_m, d_u) != 0) {
		return 1;
	}

	if (harmonic_execute(2, m, 0.01, d_m, d_u, b, t, 25) != 0) {
		return 1;
	}

	print_world(m, d_u);

	if (harmonic_free(d_m, d_u) != 0) {
		return 1;
	}

	return 0;
}
