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

        self.windowTitle = "Harmonic Map"
        self.originalImage = None
        self.image = None

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
        self.n = 2

        array_type_n_uint = ct.c_uint * (self.n)
        self.m = array_type_n_uint(*np.array([self.image.shape[0], self.image.shape[1]]))

        # TODO: Do the log conversion here.
        array_type_m_longdouble = ct.c_longdouble * (self.image.size)
        self.u = array_type_m_longdouble(*np.array([[1.0 - float(self.image[y, x] == 255) \
                                                for x in range(self.image.shape[1])] \
                                            for y in range(self.image.shape[0])]).flatten())
        self._convert_u()

        array_type_m_uint = ct.c_uint * (self.image.size)
        self.locked = array_type_m_uint(*np.array([[int(self.image[y, x] == 0 or self.image[y, x] == 255) \
                                                for x in range(self.image.shape[1])] \
                                            for y in range(self.image.shape[0])]).flatten())

    def _convert_u(self):
        """ Convert u to v. """

        # Since this is supposed to get as close to 0 as possible, we have log(1-1e-13) ~= -4.3442952e-14
        # which is within double machine precision.
        #epsilon = 1e-130000

        for y in range(self.m[0]):
            for x in range(self.m[1]):
                # u[y * self.m[1] + x] = np.log((1.0 - self.u[y * self.m[1] + x]) * (1.0 - epsilon) + epsilon)
                if self.u[y * self.m[1] + x] == 1.0:
                    # log(epsilon) = -1e13  =>  epsilon = e^-1e13 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # For example, e^-1e1 = 0.00004539992, e^-1e2 = e^-1e2, etc.
                    # Note: This is within precision of doubles, which has issues around +/- 1e15.
                    self.u[y * self.m[1] + x] = -1e13
                else:
                    self.u[y * self.m[1] + x] = 0.0

        print("DONE...")
        sys.stdout.flush()

    def _compute_streamline(self, x, y):
        """ Compute a streamline (series of points) starting from this initial (x, y) location.

            Parameters:
                x   --  The x location to start.
                y   --  The y location to start.

            Returns:
                The list of points from this starting location to a goal.
        """

        maxPoints = self.image.size

        points = [(x, y)]

        while self.locked[y * self.m[1] + x] != 1 and len(points) < maxPoints:
            uStar = None
            coordStar = None

            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    if uStar is None or self.u[(y + j) * self.m[1] + (x + i)] > uStar:
                        uStar = self.u[(y + j) * self.m[1] + (x + i)]
                        coordStar = (x + i, y + j)

            if coordStar[0] == x and coordStar[1] == y:
                break

            x = coordStar[0]
            y = coordStar[1]

            points += [(coordStar[0], coordStar[1])]

            print(coordStar, uStar)

        return points

    def _draw_image(self):
        """ Draw the image given the updated u values. """

        #epsilon = 1e-30

        # Convert the 2d potential back into an image.
        self.image = self.originalImage.copy()
        #self.image = np.array([[int(np.log((self.u[y * self.m[1] + x]) * (1.0 - epsilon) + epsilon) / np.log(epsilon) * 255.0) \
        #self.image = np.array([[int((1.0 - self.u[y * self.m[1] + x]) * 255.0) \
        #                            for x in range(self.m[1])] \
        #                        for y in range(self.m[0])], dtype=np.uint8)

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
                # Repaint the image!
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

