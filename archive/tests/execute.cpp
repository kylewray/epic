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


#include "trials.h"

int main(int argc, char *argv[])
{
	// Execute the full trials.
//	trials_2d(1024, 1024, 100, 0.0001f, 32, 2048, 32, 100, 10);
//	trials_3d(32, 32, 512, 100, 0.0001f, 16, 512, 16, 200, 10); // The two '512' values should be equal if you change it (if possible).
//	trials_4d(16, 16, 16, 96, 100, 0.0001f, 16, 80, 8, 400, 10); // The two '80' values should be equal if you change it (if possible).


	// Examine different size problems (e.g., min=128, max=128, step=32 (ignored)).
	// Each time vary the number of threads at the same speed. At the very end,
	// we note that the number of threads is less than the size of the area, so
	// we do blocks of threads in the awesome way.
//	trials_2d(128, 128, 100, 0.0001f, 128, 128, 32, 10);
//	trials_2d(256, 256, 100, 0.0001f, 256, 256, 32, 10);
//	trials_2d(512, 512, 100, 0.0001f, 512, 512, 32, 10);
//	trials_2d(512, 512, 100, 0.0001f, 1024, 1024, 32, 10);
//	trials_2d(512, 512, 100, 0.0001f, 2048, 2048, 32, 10);

	single_trial_2d();



	// 24 * 24 = 576 ... 576 % 32 = 18

//	trials_3d(32, 24, 32, 100, 0.0001f, 32, 32, 32, 10);
//	trials_3d(64, 24, 32, 100, 0.0001f, 64, 64, 32, 10);
//	trials_3d(96, 24, 32, 100, 0.0001f, 96, 96, 32, 10);
//	trials_3d(128, 24, 32, 100, 0.0001f, 128, 128, 32, 10);
//	trials_3d(256, 24, 32, 100, 0.0001f, 256, 256, 32, 10);
//	trials_3d(512, 24, 32, 100, 0.0001f, 512, 512, 32, 10);

//	single_trial_3d();



	return 0;
}
