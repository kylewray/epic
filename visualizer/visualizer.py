""" The MIT License (MIT)

    Copyright (c) 2014 Kyle Wray

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

import sys
import csv

import sdl2
import sdl2.ext
import sdl2.sdlgfx


class Visualizer(object):
    """ A visualizer for the harmonic functions. """

    def __init__(self, filename):
        """ The main constructor for the Visualizer class. It optionally takes in
            a filename to load.

            Parameters:
                filename -- Optionally, specify the file to load.
        """

        self.width = 1600
        self.height = 900

        self.data = list()

        if filename != None:
            self.load_data(filename)


    def load_data(self, filename):
        """ Load data from a file.
        
            Parameters:
                filename -- The file to load.
        """

        self.data = list()

        with open(filename, "r") as f:
            reader = csv.reader(f)

            for row in reader:
                self.data = self.data + [row]


    def execute(self):
        """ Execute the main loop of the visualizer. """

        sdl2.ext.init()

        window = sdl2.ext.Window("Nova Harmonic Visualizer", size=(WIDTH, HEIGHT))
        window.show()

        renderer = sdl2.ext.Renderer(window)

        running = True
        while running:
            events = sdl2.ext.get_events()

            for event in events:
                if event.type == sdl2.SDL_QUIT:
                    running = False
                    #break

                #self._check_keyboard(event)
                #self._check_mouse(event)

            #self._update_camera()

            renderer.color = sdl2.ext.Color(230, 230, 220)
            renderer.clear()

            #if self.fastRender:
            #    self._render_map_texture(renderer)
            #else:
            #    self._render_map(renderer)
            #    self._render_policy(renderer)

            renderer.present()

        sdl2.ext.quit()


if __name__ == "__main__":
    try:
        visualizer = Visualizer(sys.argv[1])
        visualizer.execute()
    except IndexError:
        print("Format: python visualizer.py <data file>")
    except IOError:
        print("Error: File '%s' not found." % sys.argv[1])


