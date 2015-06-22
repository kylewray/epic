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

#include <math.h>
#include <cstdio>

long long get_current_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000LL + tv.tv_usec / 1000;
}

std::string compute_statistics(std::vector<double> samples)
{
	double mean = 0.0;
	for (std::vector<double>::iterator s = samples.begin(); s != samples.end(); s++) {
		mean += *s;
	}
	mean /= (double)samples.size();

	double stdev = 0.0;
	for (std::vector<double>::iterator s = samples.begin(); s != samples.end(); s++) {
		stdev += (*s - mean) * (*s - mean);
	}
	stdev = sqrt(stdev / (double)(samples.size() - 1));

	double ci95 = stdev / sqrt((double)samples.size()) * 1.96;

	char buffer[512];
	snprintf(buffer, sizeof(buffer), "%f,%f,%f", mean, stdev, ci95);
	return buffer;
}
