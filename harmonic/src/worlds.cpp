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

#include "../include/worlds.h"

void create_simple_world_2d(unsigned int *&m, float *&h)
{
	m = new unsigned int[2];
	m[0] = 10; m[1] = 10;

	h = new float[m[0] * m[1]];

	// By default, all are zero without signed bit flipped.
	for (int i = 0; i < m[0] / 2; i++) {
		for (int j = 0; j < m[1] / 2; j++) {
//			h[i * m[1] + j] = 0.0f;
			h[i * m[1] + j] = 0.9f; // Most values are closer to 1 than 0, so this will improve performance.
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

void print_world_2d(unsigned int *m, float *u)
{
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			unsigned long long int index = i * m[1] + j;
			if (signbit(u[index]) != 0) {
				printf("----- ");
			} else if (fabs(u[index]) == 0.0f) {
				printf("+++++ ");
			} else {
				printf("%.3f ", fabs(u[index]));
			}

//			printf("%.3f ", u[index]);
		}
		printf("\n");
	}
}

void create_variable_world_2d(const unsigned int *m, float *&h, unsigned int numRectangles, unsigned int maxSize)
{
	h = new float[m[0] * m[1]];

	// By default, all are zero without signed bit flipped.
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
//			h[i * m[1] + j] = 0.0f;
			h[i * m[1] + j] = 0.9f; // Most values are closer to 1 than 0, so this will improve performance.
		}
	}

	// Boundaries.
	for (int i = 0; i < m[0]; i++) {
		h[i * m[1] + 0] = -1.0f;
		h[i * m[1] + (m[1] - 1)] = -1.0f;
	}
	for (int j = 0; j < m[1]; j++) {
		h[0 * m[1] + j] = -1.0f;
		h[(m[0] - 1) * m[1] + j] = -1.0f;
	}

	// Random boxes.
	for (int k = 0; k < numRectangles; k++) {
		unsigned int r = (unsigned int)((float)(m[0] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int c = (unsigned int)((float)(m[1] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int x = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;
		unsigned int y = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;

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

//	printf("Goal Cell: (%i, %i)\n", r, c);
}

void create_simple_world_3d(unsigned int *&m, float *&h)
{
	m = new unsigned int[3];
	m[0] = 10; m[1] = 10; m[2] = 10;

	h = new float[m[0] * m[1] * m[2]];

	// By default, all are zero without signed bit flipped.
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			for (int k = 0; k < m[2]; k++) {
//				h[i * m[1] * m[2] + j * m[2] + k] = 0.0f;
				h[i * m[1] * m[2] + j * m[2] + k] = 0.9f; // Most values are closer to 1 than 0, so this will improve performance.
			}
		}
	}

	// Boundaries.
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			h[i * m[1] * m[2] + j * m[2] + 0] = -1.0f;
			h[i * m[1] * m[2] + j * m[2] + (m[2] - 1)] = -1.0f;
		}
	}
	for (int j = 0; j < m[1]; j++) {
		for (int k = 0; k < m[2]; k++) {
			h[0 * m[1] * m[2] + j * m[2] + k] = -1.0f;
			h[(m[0] - 1) * m[1] * m[2] + j * m[2] + k] = -1.0f;
		}
	}
	for (int i = 0; i < m[0]; i++) {
		for (int k = 0; k < m[2]; k++) {
			h[i * m[1] * m[2] + 0 * m[2] + k] = -1.0f;
			h[i * m[1] * m[2] + (m[1] - 1) * m[2] + k] = -1.0f;
		}
	}
	for (int i = 0; i < m[0] / 2; i++) {
		for (int j = 0; j < m[1] / 2; j++) {
			for (int k = 0; k < m[2] / 2; k++) {
				h[i * m[1] * m[2] + j * m[2] + k] = -1.0f;
			}
		}
	}

	// Goal cells.
	h[4 * m[1] * m[2] + 4 *m[2] + 2] = -0.0f;
	h[4 * m[1] * m[2] + 4 *m[2] + 3] = -0.0f;
}

void print_world_3d(unsigned int *m, float *u)
{
	for (int i = 0; i < m[0]; i++) {
		printf("Outer Dimension %i of %i\n", i + 1, m[0]);
		for (int j = 0; j < m[1]; j++) {
			for (int k = 0; k < m[2]; k++) {
				unsigned long long int index = i * m[1] * m[2] + j * m[2] + k;
				if (signbit(u[index]) != 0) {
					printf("----- ");
				} else if (fabs(u[index]) == 0.0f) {
					printf("+++++ ");
				} else {
					printf("%.3f ", fabs(u[index]));
				}

//				printf("%.3f ", u[index]);
			}
			printf("\n");
		}
		printf("\n");
	}
}

void create_variable_world_3d(const unsigned int *m, float *&h, unsigned int numRectangles, unsigned int maxSize)
{
	h = new float[m[0] * m[1] * m[2]];

	// By default, all are zero without signed bit flipped.
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			for (int k = 0; k < m[2]; k++) {
//				h[i * m[1] * m[2] + j * m[2] + k] = 0.0f;
				h[i * m[1] * m[2] + j * m[2] + k] = 0.9f; // Most values are closer to 1 than 0, so this will improve performance.
			}
		}
	}

	// Boundaries.
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			h[i * m[1] * m[2] + j * m[2] + 0] = -1.0f;
			h[i * m[1] * m[2] + j * m[2] + (m[2] - 1)] = -1.0f;
		}
	}
	for (int j = 0; j < m[1]; j++) {
		for (int k = 0; k < m[2]; k++) {
			h[0 * m[1] * m[2] + j * m[2] + k] = -1.0f;
			h[(m[0] - 1) * m[1] * m[2] + j * m[2] + k] = -1.0f;
		}
	}
	for (int i = 0; i < m[0]; i++) {
		for (int k = 0; k < m[2]; k++) {
			h[i * m[1] * m[2] + 0 * m[2] + k] = -1.0f;
			h[i * m[1] * m[2] + (m[1] - 1) * m[2] + k] = -1.0f;
		}
	}

	// Random boxes.
	for (int k = 0; k < numRectangles; k++) {
		unsigned int r = (unsigned int)((float)(m[0] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int c = (unsigned int)((float)(m[1] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int d = (unsigned int)((float)(m[2] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int x = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;
		unsigned int y = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;
		unsigned int z = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;

		for (unsigned int i = r; i < r + x; i++) {
			for (unsigned int j = c; j < c + y; j++) {
				for (unsigned int k = d; k < d + z; k++) {
					h[i * m[1] * m[2] + j * m[2] + k] = -1.0f;
				}
			}
		}
	}

	// Randomly pick a goal point.
	unsigned int r = (unsigned int)((float)m[0] * rand() / (float)RAND_MAX);
	unsigned int c = (unsigned int)((float)m[1] * rand() / (float)RAND_MAX);
	unsigned int d = (unsigned int)((float)m[2] * rand() / (float)RAND_MAX);

	while (h[r * m[1] * m[2] + c * m[2] + d] < 0.0f) {
		r = (unsigned int)((float)m[0] * rand() / (float)RAND_MAX);
		c = (unsigned int)((float)m[1] * rand() / (float)RAND_MAX);
		d = (unsigned int)((float)m[2] * rand() / (float)RAND_MAX);
	}
	h[r * m[1] * m[2] + c * m[2] + d] = -0.0f;

//	printf("Goal Cell: (%i, %i, %i)\n", r, c, d);
}

void create_variable_world_4d(const unsigned int *m, float *&h, unsigned int numRectangles, unsigned int maxSize)
{
	h = new float[m[0] * m[1] * m[2] * m[3]];

	// By default, all are zero without signed bit flipped.
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			for (int k = 0; k < m[2]; k++) {
				for (int l = 0; l < m[3]; l++) {
//					h[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l] = 0.0f;
					h[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l] = 0.0f; // Most values are closer to 1 than 0, so this will improve performance.
				}
			}
		}
	}

	// Boundaries.
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			for (int k = 0; k < m[2]; k++) {
				h[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + 0] = -1.0f;
				h[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + (m[3] - 1)] = -1.0f;
			}
		}
	}
	for (int i = 0; i < m[0]; i++) {
		for (int j = 0; j < m[1]; j++) {
			for (int l = 0; l < m[3]; l++) {
				h[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + 0 * m[3] + l] = -1.0f;
				h[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + (m[2] - 1) * m[3] + l] = -1.0f;
			}
		}
	}
	for (int i = 0; i < m[0]; i++) {
		for (int k = 0; k < m[2]; k++) {
			for (int l = 0; l < m[3]; l++) {
				h[i * m[1] * m[2] * m[3] + 0 * m[2] * m[3] + k * m[3] + l] = -1.0f;
				h[i * m[1] * m[2] * m[3] + (m[1] - 1) * m[2] * m[3] + k * m[3] + l] = -1.0f;
			}
		}
	}
	for (int j = 0; j < m[1]; j++) {
		for (int k = 0; k < m[2]; k++) {
			for (int l = 0; l < m[3]; l++) {
				h[0 * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l] = -1.0f;
				h[(m[0] - 1) * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l] = -1.0f;
			}
		}
	}

	// Random boxes.
	for (int k = 0; k < numRectangles; k++) {
		unsigned int r = (unsigned int)((float)(m[0] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int c = (unsigned int)((float)(m[1] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int d = (unsigned int)((float)(m[2] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int e = (unsigned int)((float)(m[3] - (maxSize - 1)) * rand() / (float)RAND_MAX);
		unsigned int x = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;
		unsigned int y = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;
		unsigned int z = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;
		unsigned int a = (unsigned int)((float)(maxSize - 2) * rand() / (float)RAND_MAX) + 1;

		for (unsigned int i = r; i < r + x; i++) {
			for (unsigned int j = c; j < c + y; j++) {
				for (unsigned int k = d; k < d + z; k++) {
					for (unsigned int l = e; l < e + a; l++) {
						h[i * m[1] * m[2] * m[3] + j * m[2] * m[3] + k * m[3] + l] = -1.0f;
					}
				}
			}
		}
	}

	// Randomly pick a goal point.
	unsigned int r = (unsigned int)((float)m[0] * rand() / (float)RAND_MAX);
	unsigned int c = (unsigned int)((float)m[1] * rand() / (float)RAND_MAX);
	unsigned int d = (unsigned int)((float)m[2] * rand() / (float)RAND_MAX);
	unsigned int e = (unsigned int)((float)m[3] * rand() / (float)RAND_MAX);

	while (h[r * m[1] * m[2] * m[3] + c * m[2] * m[3] + d * m[3] + e] < 0.0f) {
		r = (unsigned int)((float)m[0] * rand() / (float)RAND_MAX);
		c = (unsigned int)((float)m[1] * rand() / (float)RAND_MAX);
		d = (unsigned int)((float)m[2] * rand() / (float)RAND_MAX);
		e = (unsigned int)((float)m[3] * rand() / (float)RAND_MAX);
	}
	h[r * m[1] * m[2] * m[3] + c * m[2] * m[3] + d * m[3] + e] = -0.0f;

//	printf("Goal Cell: (%i, %i, %i)\n", r, c, d);
}

