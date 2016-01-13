""" The MIT License (MIT)

    Copyright (c) 2015 Kyle Hollins Wray, University of Massachusetts

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import os
import sys

import math
import itertools as it

thisFilePath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(thisFilePath, "..", "..", "python"))
from epic.harmonic import *
from epic.harmonic_map import *

import epic.epic_harmonic as eh


FLOAT_COLOR = 60
DOUBLE_COLOR = 90
LONG_DOUBLE_COLOR = 120
FREE_SPACE_COLOR = 150


#wideMazePath = os.path.join(thisFilePath, "basic.png")
#wideMazePath = os.path.join(thisFilePath, "c_space.png")
#wideMazePath = os.path.join(thisFilePath, "maze_1.png")
wideMazePath = os.path.join(thisFilePath, "maze_4.png")

harmonicMap = HarmonicMap()
harmonicMap.load(wideMazePath)
#harmonicMap.show()


def compute_potential(u, x, y):
    """ Compute the potential at a location.

        Parameters:
            u   --  The potentials (row-major 1-d array).
            x   --  The x-axis location.
            y   --  The y-axis location.

        Returns:
            The potential at the location.
    """

    xtl = int(x - 0.5)
    ytl = int(y - 0.5)
    xtr = int(x + 0.5)
    ytr = int(y - 0.5)
    xbl = int(x - 0.5)
    ybl = int(y + 0.5)
    xbr = int(x + 0.5)
    ybr = int(y + 0.5)

    alpha = (x - xtl)
    beta = (y - ytl)

    one = (1.0 - alpha) * u[ytl * harmonicMap.originalImage.shape[1] + xtl] + \
          alpha * u[ytr * harmonicMap.originalImage.shape[1] + xtr]
    two = (1.0 - alpha) * u[ybl * harmonicMap.originalImage.shape[1] + xbl] + \
          alpha * u[ybr * harmonicMap.originalImage.shape[1] + xbr]
    return (1.0 - beta) * one + beta * two


def color_valid_gradient_in_image(u, c):
    """ Color the image on all cells with a valid gradient.

        Parameters:
            u   --  The potentials (row-major 1-d array).
            c   --  The grayscale color (0 to 255) to paint on the image.
    """

    originalColorMapping = dict()

    for y in range(1, harmonicMap.originalImage.shape[0] - 1):
        for x in range(1, harmonicMap.originalImage.shape[1] - 1):
            # Obstacles and goals are skipped.
            if harmonicMap.originalImage[y, x] == 255 or harmonicMap.originalImage[y, x] == 0:
                continue

            # Check if this pixel has a valid gradient; it is not valid without one.
            valid = True

            precision = 0.5

            value0 = compute_potential(u, x - precision, y)
            value1 = compute_potential(u, x + precision, y)
            value2 = compute_potential(u, x, y - precision)
            value3 = compute_potential(u, x, y + precision)

            partialX = (value1 - value0) / (2.0 * precision)
            partialY = (value3 - value2) / (2.0 * precision)

            denom = math.sqrt(pow(partialX, 2) + pow(partialY, 2))

            if denom <= 1e-10:
                valid = False

            # Draw color where precision is valid.
            if valid:
                originalColorMapping[(x, y)] = harmonicMap.originalImage[y, x]
                harmonicMap.originalImage[y, x] = c

            # Uncomment to instead draw the potential.
            #harmonicMap.originalImage[y, x] = int(255.0 *
            #        (1.0 - u[y * harmonicMap.originalImage.shape[1] + x]))

    # Compute all pixels that can reach a goal over these valid colored pixels.
    closedset = set()
    openset = [(xi, yj)
                for xi, yj in it.product(range(1, harmonicMap.originalImage.shape[1] - 1), range(1, harmonicMap.originalImage.shape[0] - 1))
            if harmonicMap.originalImage[yj, xi] == 255]

    while len(openset) > 0:
        node = openset.pop()
        closedset |= {node}

        for x, y in [(node[0] - 1, node[1]), (node[0] + 1, node[1]), (node[0], node[1] - 1), (node[0], node[1] + 1)]:
            if x < 0 or y < 0 or \
                    x >= harmonicMap.originalImage.shape[1] or y >= harmonicMap.originalImage.shape[0] or \
                    (x, y) in closedset or harmonicMap.originalImage[y, x] != c:
                continue

            openset += [(x, y)]

    # Finally, re-color everything to only render the valid pixels.
    for y in range(1, harmonicMap.originalImage.shape[0] - 1):
        for x in range(1, harmonicMap.originalImage.shape[1] - 1):
            # Obstacles and goals are skipped.
            if harmonicMap.originalImage[y, x] == 255 or harmonicMap.originalImage[y, x] == 0:
                continue

            # Only re-color this specific color to freespace.
            if harmonicMap.originalImage[y, x] == c and (x, y) not in closedset:
                harmonicMap.originalImage[y, x] = originalColorMapping[(x, y)]


array_type_m_uint = ct.c_uint * (harmonicMap.originalImage.size)
locked = array_type_m_uint(*np.array([[int(harmonicMap.originalImage[y, x] == 0 or \
                                            harmonicMap.originalImage[y, x] == 255) \
                                        for x in range(harmonicMap.originalImage.shape[1])] \
                                    for y in range(harmonicMap.originalImage.shape[0])]).flatten())

print("Computing SOR float...")
array_type_m_float = ct.c_float * (harmonicMap.originalImage.size)
uFloat = array_type_m_float(*np.array([[1.0 - float(harmonicMap.originalImage[y, x] == 255) \
                                    for x in range(harmonicMap.originalImage.shape[1])] \
                                for y in range(harmonicMap.originalImage.shape[0])]).flatten())
result = eh._epic.harmonic_legacy_sor_2d_float_cpu(harmonicMap.originalImage.shape[1],
                                                   harmonicMap.originalImage.shape[0],
                                                   1e-4, 1.5, locked, uFloat)

print("Computing SOR double...")
array_type_m_double = ct.c_double * (harmonicMap.originalImage.size)
uDouble = array_type_m_double(*np.array([[1.0 - float(harmonicMap.originalImage[y, x] == 255) \
                                    for x in range(harmonicMap.originalImage.shape[1])] \
                                for y in range(harmonicMap.originalImage.shape[0])]).flatten())
result = eh._epic.harmonic_legacy_sor_2d_double_cpu(harmonicMap.originalImage.shape[1],
                                                    harmonicMap.originalImage.shape[0],
                                                    1e-4, 1.5, locked, uDouble)

#print("Computing SOR long double...")
#array_type_m_longdouble = ct.c_longdouble * (harmonicMap.originalImage.size)
#uLongDouble = array_type_m_longdouble(*np.array([[1.0 - float(harmonicMap.originalImage[y, x] == 255) \
#                                    for x in range(harmonicMap.originalImage.shape[1])] \
#                                for y in range(harmonicMap.originalImage.shape[0])]).flatten())
#result = eh._epic.harmonic_legacy_sor_2d_long_double_cpu(harmonicMap.originalImage.shape[1],
#                                                         harmonicMap.originalImage.shape[0],
#                                                         1e-4, 1.5, locked, uLongDouble)

print("Computing Log-Space GS GPU...")
timing = harmonicMap.solve(process='gpu', epsilon=1e-4)

#print("Coloring map with long double colors...")
#color_valid_gradient_in_image(uLongDouble, LONG_DOUBLE_COLOR)

print("Coloring map with double colors...")
color_valid_gradient_in_image(uDouble, DOUBLE_COLOR)

print("Coloring map with float colors...")
color_valid_gradient_in_image(uFloat, FLOAT_COLOR)

harmonicMap.show()

print("Done.")

