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
import epic_harmonic as eh


class HarmonicLegacy(object):
    """ A Legacy Harmonic object that can be used to easily solve harmonic functions. """

    def __init__(self):
        """ The constructor for the HarmonicLegacy class. """

        # Assign a nullptr for the device-side pointers.
        self.w = int(0)
        self.h = int(0)
        self.epsilon = 1e-2
        self.omega = 1.0
        self.locked = ct.POINTER(ct.c_uint)()
        self.u = ct.POINTER(ct.c_double)()
        self.currentIteration = int(0)

    def solve(self, omega=1.0, epsilon=1e-2):
        """ Solve the Harmonic Legacy function.

            Parameters:
                omega       --  The relaxation parameter. Default is 1.0.
                epsilon     --  The error from the true final value *in log space*. Default is 1e-2.

            Returns:
                A pair (wall-time, cpu-time) for the solver execution time, not including (un)initialization.
        """

        self.epsilon = epsilon
        self.omega = omega

        timing = None

        numIterations = ct.c_uint(int(0))

        timing = (time.time(), time.clock())
        result = eh._epic.harmonic_legacy_sor_2d_double_cpu(self.w, self.h,
                                                        self.epsilon, self.omega,
                                                        self.locked, self.u,
                                                        ct.byref(numIterations))
        timing = (time.time() - timing[0], time.clock() - timing[1])

        self.currentIteration = numIterations.value

        print("DONE! NUM ITERATIONS: %i" % (self.currentIteration))

        if result != 0:
            print("Failed to execute the 'harmonic' library's *legacy* CPU SOR solver.")

        return timing

    def __str__(self):
        """ Return the string of the Harmonic Legacy values.

            Returns:
                The string of the Harmonic Legacy values.
        """

        result = "w:        " + str(self.w) + "\n"
        result += "h:        " + str(self.h) + "\n"
        result += "epsilon:  " + str(self.epsilon) + "\n"
        result += "omega:  " + str(self.omega) + "\n"
        result += "locked:\n" + str(np.array([self.locked[i] for i in range(self.w * self.h)]).reshape((w, h))) + "\n\n"
        result += "u:\n" + str(np.array([self.u[i] for i in range(self.w * self.h)]).reshape((w, h))) + "\n\n"

        return result

