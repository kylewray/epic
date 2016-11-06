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


#ifndef HARMONIC_LEGACY_PATH_CPU_H
#define HARMONIC_LEGACY_PATH_CPU_H


namespace epic {

/**
 *  Compute the potential in two dimensions at an (x, y) location which is
 *  in "double pixel units" -- meaning these are "blended cell indexes."
 *  @param  w           The width dimension of u and locked.
 *  @param  h           The height dimension of u and locked.
 *  @param  locked      The locked cells of size w * h.
 *  @param  u           The potential values of size w * h.
 *  @param  x           The x "double pixel" or "blended cell index" location.
 *  @param  y           The y "double pixel" or "blended cell index" location.
 *  @param  potential   The potential at this location. This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_legacy_compute_potential_2d_cpu(unsigned int w, unsigned int h, unsigned int *locked, double *u,
        double x, double y, double &potential);

/**
 *  Compute the gradient in two dimensions at an (x, y) location which is
 *  in so-called "double pixel units."
 *  @param  w           The width dimension of u and locked.
 *  @param  h           The height dimension of u and locked.
 *  @param  locked      The locked cells of size w * h.
 *  @param  u           The potential values of size w * h.
 *  @param  x           The x "double pixel" or "blended cell index" location.
 *  @param  y           The y "double pixel" or "blended cell index" location.
 *  @param  cdPrecision The central difference (gradient) precision. Default is 0.5.
 *  @param  partialX    The gradient in the x axis. This will be modified.
 *  @param  partialY    The gradient in the y axis. This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_legacy_compute_gradient_2d_cpu(unsigned int w, unsigned int h, unsigned int *locked, double *u,
        double x, double y, double cdPrecision, double &partialX, double &partialY);

/**
 *  Compute a path by following the gradient of the potential values. The starting
 *  location is specified in "double pixel units" and the path is also in these units.
 *  @param  w               The width dimension of u and locked.
 *  @param  h               The height dimension of u and locked.
 *  @param  locked          The locked cells of size w * h.
 *  @param  u               The potential values of size w * h.
 *  @param  x               The x "double pixel" start location.
 *  @param  y               The y "double pixel" start location.
 *  @param  stepSize        The step size in "double pixels." Default is 0.05.
 *  @param  cdPrecision     The central difference (gradient) precision. Default is 0.5.
 *  @param  maxLength       The maximum path length (in number of steps).
 *  @param  flipped         Positive means to perform gradient ascent, otherwise descent.
 *  @param  k               The number of elements in the path. This will be modified.
 *  @param  path            The path (2k array) of the form [x1, y1, x2, y2, ..., xk, yk].
 *                          This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_legacy_compute_path_2d_cpu(unsigned int w, unsigned int h, unsigned int *locked, double *u,
        double x, double y, double stepSize, double cdPrecision, unsigned int maxLength, int flipped,
        unsigned int &k, double *&path);

/**
 *  Free the path provided. Useful for wrappers, such as Python.
 *  @param  path    The path to free. This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_legacy_free_path_cpu(double *&path);

};


#endif // HARMONIC_LEGACY_PATH_CPU_H

