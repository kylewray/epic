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

import ctypes as ct
import platform
import os.path


# Check if we need to create the harmonic variable. If so, import the correct library
# file depending on the platform.
#try:
#    _inertia
#except NameError:
_inertia = None
if platform.system() == "Windows":
    _inertia = ct.CDLL(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    "..", "..", "lib", "inertia.dll"))
else:
    _inertia = ct.CDLL(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    "..", "..", "lib", "inertia.so"))


class InertiaHarmonic(ct.Structure):
    """ The C struct Harmonic object. """

    _fields_ = [("n", ct.c_uint),
                ("m", ct.POINTER(ct.c_uint)),
                ("u", ct.POINTER(ct.c_longdouble)),
                ("locked", ct.POINTER(ct.c_uint)),
                ("epsilon", ct.c_longdouble),
                ("omega", ct.c_longdouble),
                ("currentIteration", ct.c_uint),
                ("d_m", ct.POINTER(ct.c_uint)),
                ("d_u", ct.POINTER(ct.c_longdouble)),
                ("d_locked", ct.POINTER(ct.c_uint)),
                ]


_inertia.harmonic_sor_2d_cpu.argtypes = tuple([ct.POINTER(InertiaHarmonic)])

#_inertia.harmonic_sor_2d_gpu.argtypes = (ct.POINTER(InertiaHarmonic),
#                                            ct.c_uint)     # numThreads
