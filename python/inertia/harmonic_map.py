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
import harmonic as harm


class HarmonicMap(harm.Harmonic):
    """ A HarmonicMap object that can be used to easily solve harmonic functions in 2d maps. """

    def __init__(self):
        """ The constructor for the HarmonicMap class. """

        super().__init__()

        self.windowTitle = "Harmonic Map"

        self.originalImage = None
        self.image = None

        self.pxSize = 1.0

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
                    # log(epsilon) = -1e10  =>  epsilon = e^-1e10 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # For example, e^-1e1 = 0.00004539992, e^-1e2 = 3.720076e-44, etc.
                    # Note: This is within precision of doubles, which has issues around +/- 1e15.
                    self.u[y * self.m[1] + x] = -1e15
                else:
                    self.u[y * self.m[1] + x] = 0.0

    def _compute_potential(self, x, y):
        """ Get the value of a true between pixel point (x, y) as linear combination of neighbor
            pixel values.
        
            Parameters:
                x   --  The x coordinate (between pixels).
                y   --  The y coordinate (between pixels).

            Returns:
                The linear combination of the neighboring pixel values of (x, y).
        """

        xCellIndex, yCellIndex = self._compute_cell_index(x, y)

        if self.locked[yCellIndex * self.m[1] + xCellIndex] == 1 and \
                self.u[yCellIndex * self.m[1] + xCellIndex] < 0.0:
            raise Exception()

        xtl = int(x - 0.5 * self.pxSize)
        ytl = int(y - 0.5 * self.pxSize)

        xtr = int(x + 0.5 * self.pxSize)
        ytr = int(y - 0.5 * self.pxSize)

        xbl = int(x - 0.5 * self.pxSize)
        ybl = int(y + 0.5 * self.pxSize)

        xbr = int(x + 0.5 * self.pxSize)
        ybr = int(y + 0.5 * self.pxSize)

        alpha = (x - xtl) / self.pxSize
        beta = (y - ytl) / self.pxSize

        one = (1.0 - alpha) * self.u[ytl * self.m[1] + xtl] + alpha * self.u[ytr * self.m[1] + xtr]
        two = (1.0 - alpha) * self.u[ybl * self.m[1] + xbl] + alpha * self.u[ybr * self.m[1] + xbr]
        three = (1.0 - beta) * one + beta * two

        return three

    def _compute_cell_index(self, x, y):
        """ Compute the offset to get the actual cell index from the subpixel coordinate.

            Parameters:
                x   --  The x coordinate (between pixels).
                y   --  The y coordinate (between pixels).

            Returns:
                xCellIndex  --  The x cell index.
                yCellIndex  --  The y cell index.
        """

        return int(x + 0.5 * self.pxSize), int(y + 0.5 * self.pxSize)

    def _compute_streamline(self, x, y):
        """ Compute a streamline (series of points) starting from this initial (x, y) location.

            Parameters:
                x   --  The x location to start.
                y   --  The y location to start.

            Returns:
                The list of points from this starting location to a goal.
        """

        points = [(x, y)]

        xCellIndex, yCellIndex = self._compute_cell_index(x, y)

        pathStepSize = 0.5
        centralDifferenceStepSize = 0.5

        maxPoints = self.image.size / pathStepSize

        while self.locked[yCellIndex * self.m[1] + xCellIndex] != 1 and len(points) < maxPoints:
            values = [self._compute_potential(x - centralDifferenceStepSize, y),
                      self._compute_potential(x + centralDifferenceStepSize, y),
                      self._compute_potential(x, y - centralDifferenceStepSize),
                      self._compute_potential(x, y + centralDifferenceStepSize)]

            partialx = (values[1] - values[0]) / (2.0 * centralDifferenceStepSize)
            partialy = (values[3] - values[2]) / (2.0 * centralDifferenceStepSize)

            denom = math.sqrt(pow(partialx, 2) + pow(partialy, 2))
            partialx /= denom
            partialy /= denom

            x += partialx * pathStepSize
            y += partialy * pathStepSize

            xCellIndex, yCellIndex = self._compute_cell_index(x, y)

            points += [(xCellIndex, yCellIndex)]

            print((x, y), (xCellIndex, yCellIndex), self._compute_potential(x, y))

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

        #self.image = np.array([[max(0.0, min(255, int(-self.u[y * self.m[1] + x]))) \
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

