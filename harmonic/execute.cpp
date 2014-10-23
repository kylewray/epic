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
#include <cstdlib>

#include "include/naive.h"
#include "include/worlds.h"

int main(int argc, char *argv[])
{
	unsigned int *m = nullptr;
	float *h = nullptr;

	srand(1);

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

	float *u = new float[m[0] * m[1]];
	harmonic_get(2, m, d_u, u);

	print_world(m, u);

	delete [] m;
	delete [] h;
	delete [] u;

	if (harmonic_free(d_m, d_u) != 0) {
		return 1;
	}

	return 0;
}
