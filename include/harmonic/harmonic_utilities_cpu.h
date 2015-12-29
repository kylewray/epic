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


#ifndef HARMONIC_UTILITIES_CPU_H
#define HARMONIC_UTILITIES_CPU_H


#include "harmonic.h"

namespace epic {

#define EPIC_CELL_TYPE_GOAL      0
#define EPIC_CELL_TYPE_OBSTACLE  1
#define EPIC_CELL_TYPE_FREE      2

/**
 *  Assign a cell to obstacle, goal, or free space in two dimensions.
 *  @param  harmonic    The Harmonic object.
 *  @param  x           The x location.
 *  @param  y           The y location.
 *  @param  type        Sets 0 for goal, 1 for obstacle, and 2 for free.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int harmonic_utilities_set_cell_2d_cpu(Harmonic *harmonic,
        unsigned int x, unsigned int y, int type);

/**
 *  Add or remove a filled square obstacle, axis-aligned in two dimensions.
 *  @param  harmonic    The Harmonic object.
 *  @param  x1          Top left x location.
 *  @param  y1          Top left y location.
 *  @param  x2          Bottom right x location.
 *  @param  y2          Bottom right y location.
 *  @param  add         True to add, false to remove.
 *  @return Returns zero upon success, non-zero otherwise.
 */
//extern "C" int harmonic_utilities_filled_square_cpu(Harmonic *harmonic,
//        unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2,
//        bool add);

/**
 *  Add or remove a filled circle obstacle, axis-aligned in two dimensions.
 *  @param  harmonic    The Harmonic object.
 *  @param  x           The x location.
 *  @param  y           The y location.
 *  @param  r           The radius of the sphere.
 *  @param  add         True to add, false to remove.
 *  @return Returns zero upon success, non-zero otherwise.
 */
//extern "C" int harmonic_utilities_filled_circle_cpu(Harmonic *harmonic,
//        unsigned int x, unsigned int y, unsigned int r, bool add);

};


#endif // HARMONIC_UTILITIES_CPU_H

