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
import time

import ctypes as ct
import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__))))
import inertia_harmonic as ih


class Harmonic(ih.InertiaHarmonic):
    """ A Harmonic object that can be used to easily solve harmonic functions. """

    def __init__(self):
        """ The constructor for the Harmonic class. """

        # Assign a nullptr for the device-side pointers. These will be set if the GPU is utilized.
        self.n = int(0)
        self.m = ct.POINTER(ct.c_uint)()
        self.u = ct.POINTER(ct.c_float)()
        self.locked = ct.POINTER(ct.c_uint)()
        self.epsilon = 1e-2
        self.delta = self.epsilon + 1.0
        self.numIterationsToStaggerCheck = int(100)
        self.currentIteration = int(0)
        self.d_m = ct.POINTER(ct.c_uint)()
        self.d_u = ct.POINTER(ct.c_float)()
        self.d_locked = ct.POINTER(ct.c_uint)()
        self.d_delta = ct.POINTER(ct.c_float)()

    def solve(self, algorithm='gauss-seidel', process='gpu', numThreads=1024, epsilon=1e-2):
        """ Solve the Harmonic function.

            Parameters:
                algorithm   --  The algorithm to use. Default is 'gauss-seidel'.
                process     --  Use the 'cpu' or 'gpu'. If 'gpu' fails, it tries 'cpu'. Default is 'gpu'.
                numThreads  --  The number of CUDA threads to execute (multiple of 32). Default is 1024.
                epsilon     --  The error from the true final value *in log space*. Default is 1e-2.

            Returns:
                A pair (wall-time, cpu-time) for the solver execution time, not including (un)initialization.
        """

        self.epsilon = epsilon

        timing = None

        if algorithm == 'gauss-seidel':
            if process == 'gpu':
                result = ih._inertia.harmonic_initialize_dimension_size_gpu(self)
                result += ih._inertia.harmonic_initialize_potential_values_gpu(self)
                result += ih._inertia.harmonic_initialize_locked_gpu(self)
                if result != 0:
                    print("Failed to initialize the harmonic variables for the 'inertia' library's GPU Gauss-Seidel solver.")
                    process = 'cpu'

                timing = (time.time(), time.clock())
                result = ih._inertia.harmonic_complete_gpu(self, int(numThreads))
                timing = (time.time() - timing[0], time.clock() - timing[1])

                if result != 0:
                    print("Failed to execute the 'inertia' library's GPU Gauss-Seidel solver.")
                    process = 'cpu'

                result = ih._inertia.harmonic_uninitialize_dimension_size_gpu(self)
                result += ih._inertia.harmonic_uninitialize_potential_values_gpu(self)
                result += ih._inertia.harmonic_uninitialize_locked_gpu(self)
                if result != 0:
                    # Note: Failing at uninitialization should not cause the CPU version to be executed.
                    print("Failed to uninitialize the harmonic variables for the 'inertia' library's GPU Gauss-Seidel solver.")

            if process == 'cpu':
                timing = (time.time(), time.clock())
                result = ih._inertia.harmonic_2d_cpu(self)
                timing = (time.time() - timing[0], time.clock() - timing[1])

                if result != 0:
                    print("Failed to execute the 'harmonic' library's CPU Gauss-Seidel solver.")
                    raise Exception()
        else:
            print("Failed to solve since the algorithm '%' is undefined." % (algorithm))

        return timing

    def __str__(self):
        """ Return the string of the Harmonic values.

            Returns:
                The string of the Harmonic values.
        """

        result = "n:        " + str(self.n) + "\n"
        result += "m:        " + str([self.m[i] for i in range(self.n)]) + "\n"
        result += "epsilon:  " + str(self.epsilon) + "\n"

        numCells = 1
        for i in range(self.n):
            numCells *= int(self.m[i])

        result += "u:\n" + str(np.array([self.u[i] for i in range(numCells)]).reshape([self.m[i] for i in range(self.n)])) + "\n\n"

        result += "locked:\n" + str(np.array([self.locked[i] for i in range(numCells)]).reshape([self.m[i] for i in range(self.n)])) + "\n\n"

        return result

