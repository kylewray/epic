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
#    _epic
#except NameError:
_epic = None
if platform.system() == "Windows":
    _ = ct.CDLL(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    "..", "..", "lib", "libepic.dll"))
else:
    _epic = ct.CDLL(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    "..", "..", "lib", "libepic.so"))


class EpicHarmonic(ct.Structure):
    """ The C struct Harmonic object. """

    _fields_ = [("n", ct.c_uint),
                ("m", ct.POINTER(ct.c_uint)),
                ("u", ct.POINTER(ct.c_float)),
                ("locked", ct.POINTER(ct.c_uint)),
                ("epsilon", ct.c_float),
                ("delta", ct.c_float),
                ("numIterationsToStaggerCheck", ct.c_uint),
                ("currentIteration", ct.c_uint),
                ("d_m", ct.POINTER(ct.c_uint)),
                ("d_u", ct.POINTER(ct.c_float)),
                ("d_locked", ct.POINTER(ct.c_uint)),
                ("d_delta", ct.POINTER(ct.c_float)),
                ]


# Functions from 'harmonic_cpu.h'.
_epic.harmonic_complete_cpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])
_epic.harmonic_update_cpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])
_epic.harmonic_update_and_check_cpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])

# Functions from 'harmonic_gpu.h'.
_epic.harmonic_complete_gpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_uint)
_epic.harmonic_initialize_gpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_uint)
_epic.harmonic_execute_gpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_uint)
_epic.harmonic_uninitialize_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])
_epic.harmonic_update_gpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_uint)
_epic.harmonic_update_and_check_gpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_uint)
_epic.harmonic_get_potential_values_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])

# Functions from 'harmonic_model_gpu.h'.
_epic.harmonic_initialize_dimension_size_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])
_epic.harmonic_uninitialize_dimension_size_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])

_epic.harmonic_initialize_potential_values_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])
_epic.harmonic_uninitialize_potential_values_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])

_epic.harmonic_initialize_locked_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])
_epic.harmonic_uninitialize_locked_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])

_epic.harmonic_update_model_gpu.argtypes = tuple([ct.POINTER(EpicHarmonic)])

# Functions from 'harmonic_utilities_cpu.h'.
_epic.harmonic_utilities_set_cells_2d_cpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_uint,
                                                ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint))

# Functions from 'harmonic_utilities_gpu.h'.
_epic.harmonic_utilities_set_cells_2d_gpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_uint, ct.c_uint,
                                                ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint))

# Functions from 'harmonic_path_cpu.h'.
_epic.harmonic_compute_potential_2d_cpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_float, ct.c_float,
                                                    ct.POINTER(ct.c_float))
_epic.harmonic_compute_gradient_2d_cpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_float, ct.c_float, ct.c_float,
                                                    ct.POINTER(ct.c_float), ct.POINTER(ct.c_float))
_epic.harmonic_compute_path_2d_cpu.argtypes = (ct.POINTER(EpicHarmonic), ct.c_float, ct.c_float,
                                                    ct.c_float, ct.c_float, ct.c_uint,
                                                    ct.POINTER(ct.c_uint), ct.POINTER(ct.POINTER(ct.c_float)))
_epic.harmonic_free_path_cpu.argtypes = tuple([ct.POINTER(ct.POINTER(ct.c_float))])

# Functions from 'harmonic_legacy_cpu.h'.
_epic.harmonic_legacy_sor_2d_float_cpu.argtypes = (ct.c_uint, ct.c_uint, ct.c_float, ct.c_float,
                                                    ct.POINTER(ct.c_uint), ct.POINTER(ct.c_float),
                                                    ct.POINTER(ct.c_uint))
_epic.harmonic_legacy_sor_2d_double_cpu.argtypes = (ct.c_uint, ct.c_uint, ct.c_double, ct.c_double,
                                                    ct.POINTER(ct.c_uint), ct.POINTER(ct.c_double),
                                                    ct.POINTER(ct.c_uint))
_epic.harmonic_legacy_sor_2d_long_double_cpu.argtypes = (ct.c_uint, ct.c_uint, ct.c_longdouble, ct.c_longdouble,
                                                    ct.POINTER(ct.c_uint), ct.POINTER(ct.c_longdouble),
                                                    ct.POINTER(ct.c_uint))

# Functions from 'harmonic_legacy_path_cpu.h'.
_epic.harmonic_legacy_compute_potential_2d_cpu.argtypes = (ct.c_uint, ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_double),
                                                    ct.c_double, ct.c_double, ct.POINTER(ct.c_double))
_epic.harmonic_legacy_compute_gradient_2d_cpu.argtypes = (ct.c_uint, ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_double),
                                                    ct.c_double, ct.c_double, ct.c_double,
                                                    ct.POINTER(ct.c_double), ct.POINTER(ct.c_double))
_epic.harmonic_legacy_compute_path_2d_cpu.argtypes = (ct.c_uint, ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_double),
                                                    ct.c_double, ct.c_double, ct.c_double, ct.c_double, ct.c_uint, ct.c_int,
                                                    ct.POINTER(ct.c_uint), ct.POINTER(ct.POINTER(ct.c_double)))
_epic.harmonic_legacy_free_path_cpu.argtypes = tuple([ct.POINTER(ct.POINTER(ct.c_double))])

