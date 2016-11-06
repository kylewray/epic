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

import math

import cv2

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__))))
import harmonic_legacy as harm
import epic_harmonic as eh


class HarmonicLegacyMap(harm.HarmonicLegacy):
    """ A HarmonicLegacyMap object that can be used to easily solve harmonic functions in 2d maps. """

    def __init__(self):
        """ The constructor for the HarmonicLegacyMap class. """

        super().__init__()

        self.windowTitle = "Harmonic Legacy Map"

        self.originalImage = None
        self.image = None

        self.pxSize = 1.0
        self.hold = False

        self.flipped = False

    def load(self, filename):
        """ Load a map from a 2d grayscale image.

            Parameters:
                filename    --  The 2d grayscale image file to load.
        """

        # Load the image and handle errors.
        self.image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        self.originalImage = self.image.copy()

        if self.image is None:
            print("Failed to load image file '%s'." % (filename))
            raise Exception()

        # Convert the image into a 2d potential and set other corresponding variables.
        self.w = int(self.image.shape[1])
        self.h = int(self.image.shape[0])

        array_type_u_double = ct.c_double * (self.image.size)
        self.u = array_type_u_double(*np.array([[1.0 - float(self.image[y, x] == 255) \
                                                for x in range(self.image.shape[1])] \
                                            for y in range(self.image.shape[0])]).flatten())

        if self.flipped:
            self._flip_u_values()

        array_type_locked_uint = ct.c_uint * (self.image.size)
        self.locked = array_type_locked_uint(*np.array([[int(self.image[y, x] == 0 or self.image[y, x] == 255) \
                                                for x in range(self.image.shape[1])] \
                                            for y in range(self.image.shape[0])]).flatten())

    def _flip_u_values(self):
        """ Flip u-values. """

        for y in range(self.h):
            for x in range(self.w):
                self.u[y * self.w + x] = 1.0 - self.u[y * self.w + x]

    def _compute_streamline(self, x, y):
        """ Compute a streamline (series of points) starting from this initial (x, y) location.

            Parameters:
                x   --  The x "double pixel" location to start.
                y   --  The y "double pixel" location to start.

            Returns:
                The list of points from this starting location to a goal.
        """

        k = ct.c_uint(0)
        rawPath = ct.POINTER(ct.c_double)()

        result = eh._epic.harmonic_legacy_compute_path_2d_cpu(self.w, self.h, self.locked, self.u, x, y,
                                                       float(0.2), float(0.4), int(1e6), int(self.flipped),
                                                       ct.byref(k), ct.byref(rawPath))
        if result != 0:
            print("Failed to compute path using 'epic' library.")
            raise Exception()

        k = int(k.value)
        path = [(rawPath[2 * i + 0], rawPath[2 * i + 1]) for i in range(k)]

        result = eh._epic.harmonic_legacy_free_path_cpu(ct.byref(rawPath))
        if result != 0:
            print("Failed to free path using 'epic' library.")

        return path

    def _draw_image(self):
        """ Draw the image given the updated u values. """

        # Convert the 2d potential back into an image.
        self.image = self.originalImage.copy()

    def show(self):
        """ Render the current image to the screen; the escape key quits. """

        def mouse_clicked(event, x, y, flags, param):
            """ Handle mouse clicks in the cv window. This draws a rough streamline from the
                clicked position to the goal.

                Parameters:
                    event   --  The event object (button up, down, etc.).
                    x       --  The x location clicked.
                    y       --  The y location clicked.
                    flags   --  Extra flags about the mouse.
                    param   --  Extra params.
            """

            if event == cv2.EVENT_LBUTTONUP:
                # Repaint the image, but only if we are not holding.
                if not self.hold:
                    self._draw_image()

                # Compute the streamline and draw the points
                for p in self._compute_streamline(x, y):
                    self.image[p[1], p[0]] = 255

                # Update the image.
                cv2.imshow(self.windowTitle, self.image)

        self._draw_image()

        # Show the image and wait for the exit key.
        cv2.imshow(self.windowTitle, self.image)
        cv2.setMouseCallback(self.windowTitle, mouse_clicked)
        #cv2.ResizeWindow(self.windowTitle, max(100, self.image.shape[0]), max(100, self.image.shape[1]))

        key = None
        while key != 27:
            key = cv2.waitKey(0)
        cv2.destroyAllWindows()

