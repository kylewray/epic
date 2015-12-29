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


#ifndef HARMONIC_PATH_CPU_H
#define HARMONIC_PATH_CPU_H


namespace epic {

#include "harmonic.h"

/**
 *  Compute the potential in two dimensions at an (x, y) location which is
 *  in "float pixel units" -- meaning these are "blended cell indexes."
 *  @param  harmonic    The Harmonic object.
 *  @param  x           The x "float pixel" or "blended cell index" location.
 *  @param  y           The y "float pixel" or "blended cell index" location.
 *  @param  potential   The potential at this location. This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_compute_potential_2d_cpu(Harmonic *harmonic,
        float x, float y, float &potential);

/**
 *  Compute the gradient in two dimensions at an (x, y) location which is
 *  in so-called "float pixel units."
 *  @param  harmonic    The Harmonic object.
 *  @param  x           The x "float pixel" or "blended cell index" location.
 *  @param  y           The y "float pixel" or "blended cell index" location.
 *  @param  cdPrecision The central difference (gradient) precision. Default is 0.5.
 *  @param  partialX    The gradient in the x axis. This will be modified.
 *  @param  partialY    The gradient in the y axis. This will be modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_compute_gradient_2d_cpu(Harmonic *harmonic, float x, float y,
        float cdPrecision, float &partialX, float &partialY);

/**
 *  Compute a path by following the gradient of the potential values. The starting
 *  location is specified in "float pixel units" and the path is also in these units.
 *  @param  harmonic        The Harmonic object.
 *  @param  x               The x "float pixel" start location.
 *  @param  y               The y "float pixel" start location.
 *  @param  stepSize        The step size in "float pixels." Default is 0.05.
 *  @param  cdPrecision     The central difference (gradient) precision. Default is 0.5.
 *  @param  maxLength       The maximum path length (in number of steps).
 *  @param  k               The number of elements in the path. This will be modified.
 *  @param  path            The path (2k array) of the form [x1, y1, x2, y2, ..., xk, yk].
 *                          This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_compute_path_2d_cpu(Harmonic *harmonic, float x, float y,
        float stepSize, float cdPrecision, unsigned int maxLength,
        unsigned int &k, float *&path);

};


#endif // HARMONIC_PATH_CPU_H

