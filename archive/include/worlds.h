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


#ifndef WORLDS_H
#define WORLDS_H


/*
 * Create a simple 10-by-10 grid world with the upper left 5-by-5 blocked as an
 * obstacle. The goal is on the bottom of this obstacle.
 * @param	m	The dimensions of the world.
 * @param	h	The resulting harmonic function.
 */
void create_simple_world_2d(unsigned int *&m, float *&h);

/**
 * Print out the world provided.
 * @param	m	The dimensions of the world.
 * @param	u	The resulting u function.
 */
void print_world_2d(unsigned int *m, float *u);

/**
 * Create a variable m1-by-m2 world with a random number of rectangles.
 * @param	m				The dimensions of the world (must be an array of size 2).
 * @param	h				The resulting harmonic function.
 * @param	numRectangles	The number of rectangles to randomly place.
 * @param	maxSize			The maximum width and height of the rectangles.
 */
void create_variable_world_2d(const unsigned int *m, float *&h, unsigned int numRectangles, unsigned int maxSize);

/*
 * Create a simple 10-by-10-by-10 grid world with the upper left 5-by-5-by-5 blocked as an
 * obstacle. The goal is on the bottom of this obstacle.
 * @param	m	The dimensions of the world.
 * @param	h	The resulting harmonic function.
 */
void create_simple_world_3d(unsigned int *&m, float *&h);

/**
 * Print out the world provided.
 * @param	m	The dimensions of the world.
 * @param	u	The resulting u function.
 */
void print_world_3d(unsigned int *m, float *u);

/**
 * Create a variable m1-by-m2-by-m3 world with a random number of rectangles.
 * @param	m				The dimensions of the world (must be an array of size 3).
 * @param	h				The resulting harmonic function.
 * @param	numRectangles	The number of rectangles to randomly place.
 * @param	maxSize			The maximum width and height of the rectangles.
 */
void create_variable_world_3d(const unsigned int *m, float *&h, unsigned int numRectangles, unsigned int maxSize);

/**
 * Create a variable m1-by-m2-by-m3-by-m4 world with a random number of rectangles.
 * @param	m				The dimensions of the world (must be an array of size 4).
 * @param	h				The resulting harmonic function.
 * @param	numRectangles	The number of rectangles to randomly place.
 * @param	maxSize			The maximum width and height of the rectangles.
 */
void create_variable_world_4d(const unsigned int *m, float *&h, unsigned int numRectangles, unsigned int maxSize);


#endif // WORLDS_H
