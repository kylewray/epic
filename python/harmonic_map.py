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

import ctypes as ct
import numpy as np

import cv2

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__))))
import harmonic as harm


class HarmonicMap(harm.Harmonic):
    """ A HarmonicMap object that can be used to easily solve harmonic functions in 2d maps. """

    def __init__(self):
        """ The constructor for the HarmonicMap class. """

        super().__init__()

        self.image = None

    def load(self, filename):
        """ Load a map from a 2d grayscale image.

            Parameters:
                filename    --  The 2d grayscale image file to load.
        """

        # Load the image and handle errors.
        self.image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)

        if self.image is None:
            print("Failed to load image file '%s'." % (filename))
            raise Exception()

        # Convert the image into a 2d potential and set other corresponding variables.
        self.n = 2

        array_type_n_uint = ct.c_uint * (self.n)
        self.m = array_type_n_uint(*np.array([self.image.shape[0], self.image.shape[1]]))

        # TODO: Do the log conversion here.
        array_type_m_float = ct.c_float * (self.image.size)
        self.u = array_type_m_float(*np.array([[float(self.image[y, x] == 0) \
                                                for x in range(self.image.shape[1])] \
                                            for y in range(self.image.shape[0])]).flatten())

        array_type_m_uint = ct.c_uint * (self.image.size)
        self.locked = array_type_m_uint(*np.array([[int(self.image[y, x] == 0 or self.image[y, x] == 255) \
                                                for x in range(self.image.shape[1])] \
                                            for y in range(self.image.shape[0])]).flatten())

    def show(self):
        """ Render the current image to the screen; the escape key quits. """

        # TODO: Do the inverse log conversion here.
        # Convert the 2d potential back into an image.
        self.image = np.array([[int((1.0 - self.u[y * self.m[1] + x]) * 255.0) \
                                    for x in range(self.m[1])] \
                                for y in range(self.m[0])], dtype=np.uint8)

        print(self.image)
        # Show the image and wait for the exit key.
        cv2.imshow("Harmonic Map Log-Scale Image", self.image)
        #cv2.ResizeWindow("Harmonic Map Log-Scale Image", max(100, self.image.shape[0]), max(100, self.image.shape[1]))

        key = None
        while key != 27:
            key = cv2.waitKey(0)
        cv2.destroyAllWindows()

