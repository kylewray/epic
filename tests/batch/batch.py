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

thisFilePath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(thisFilePath, "..", "..", "python"))
from epic.harmonic import *
from epic.harmonic_map import *

import epic.epic_harmonic as eh

import compare_precision as cp


precision = 1e-3

domains = [
            {'name': "UMass", 'filename': os.path.join(thisFilePath, "umass.png")},
            {'name': "Willow", 'filename': os.path.join(thisFilePath, "willow_garage.png")},
            {'name': "Mine S", 'filename': os.path.join(thisFilePath, "small_mine.png")},
            {'name': "Mine L", 'filename': os.path.join(thisFilePath, "large_mine.png")},
            {'name': "C-Space", 'filename': os.path.join(thisFilePath, "c_space.png")},
            {'name': "Maze S", 'filename': os.path.join(thisFilePath, "small_maze.png")},
            {'name': "Maze L", 'filename': os.path.join(thisFilePath, "large_maze.png")},
          ]


def cpu_sor(harmonicMap):
    """ Run the CPU version of SOR.

        Parameters:
            harmonicMap     --  The HarmonicMap object.

        Returns:
            percentValid    --  The percentage of valid cells.
            timePerUpdate   --  The time per update step.
            timeToConverge  --  The time until it converged.
    """

    omega = 1.5

    numIterations = ct.c_uint(int(0))
    array_type_m_uint = ct.c_uint * (harmonicMap.originalImage.size)
    locked = array_type_m_uint(*np.array([[int(harmonicMap.originalImage[y, x] == 0 or \
                                            harmonicMap.originalImage[y, x] == 255) \
                                    for x in range(harmonicMap.originalImage.shape[1])] \
                                for y in range(harmonicMap.originalImage.shape[0])]).flatten())
    array_type_m_double = ct.c_double * (harmonicMap.originalImage.size)
    uDouble = array_type_m_double(*np.array([[1.0 - float(harmonicMap.originalImage[y, x] == 255) \
                                    for x in range(harmonicMap.originalImage.shape[1])] \
                                for y in range(harmonicMap.originalImage.shape[0])]).flatten())

    timing = (time.time(), time.clock())
    result = eh._epic.harmonic_legacy_sor_2d_double_cpu(harmonicMap.originalImage.shape[1],
                                                    harmonicMap.originalImage.shape[0],
                                                    precision, omega, locked, uDouble,
                                                    ct.byref(numIterations))
    timing = (time.time() - timing[0], time.clock() - timing[1])

    harmonicMap.solve(process='gpu', epsilon=1e-4)

    cp.color_valid_gradient_in_image(harmonicMap, uDouble, cp.DOUBLE_COLOR)

    numValidStreamlines = 0
    numStreamlineChecks = 0

    for y in range(1, harmonicMap.originalImage.shape[0] - 1):
        for x in range(1, harmonicMap.originalImage.shape[1] - 1):
            if harmonicMap.originalImage[y, x] == 0:
                continue

            numStreamlineChecks += 1

            if harmonicMap.originalImage[y, x] == cp.DOUBLE_COLOR or \
                    harmonicMap.originalImage[y, x] == 255:
                numValidStreamlines += 1

    return numValidStreamlines / numStreamlineChecks, timing[0] / int(numIterations.value), timing[0]


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Please specify an output filename.")
        sys.exit(0)

    print("Starting Batch Experiments...")

    with open(sys.argv[1], 'w') as f:
        f.write(",,,CPU SOR,,CPU log-GS,,GPU log-GS,\n")
        f.write("Domain,Size")
        f.write(",Percent Valid,Time per Update,Time to Converge")
        f.write(",Time per Update,Time to Converge")
        f.write(",Time per Update,Time to Converge\n")

        for img in domains:
            # ----- CPU SOR -----
            harmonicMap = HarmonicMap()
            harmonicMap.load(img['filename'])

            f.write("%s,%i," % (img['name'],
                        harmonicMap.originalImage.shape[0] * harmonicMap.originalImage.shape[1]))
            f.flush()

            percentValid, timePerUpdate, timeToConverge = cpu_sor(harmonicMap)
            f.write("%.5f," % (percentValid))
            f.write("%.5f," % (timePerUpdate))
            f.write("%.5f," % (timeToConverge))
            f.flush()

            print(".", end='')
            sys.stdout.flush()

            # ----- CPU log-GS -----
            harmonicMap = HarmonicMap()
            harmonicMap.load(img['filename'])
            timing = harmonicMap.solve(process='cpu', epsilon=precision)

            f.write("%.5f," % (timing[0] / int(harmonicMap.currentIteration)))
            f.write("%.5f," % (timing[0]))
            f.flush()

            print(".", end='')
            sys.stdout.flush()

            # ----- GPU log-GS -----
            harmonicMap = HarmonicMap()
            harmonicMap.load(img['filename'])
            timing = harmonicMap.solve(process='gpu', epsilon=precision)

            f.write("%.5f," % (timing[0] / int(harmonicMap.currentIteration)))
            f.write("%.5f" % (timing[0]))
            f.write("\n")
            f.flush()

            print(".")
            sys.stdout.flush()

        f.close()

    print("Done.")

