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

thisFilePath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(thisFilePath, "..", "..", "python"))
from epic.harmonic import *
from epic.harmonic_map import *


images = [
          #{'name': "Basic", 'filename': os.path.join(thisFilePath, "basic.png")},
          #{'name': "C-Space", 'filename': os.path.join(thisFilePath, "c_space.png")},
          #{'name': "Maze 1", 'filename': os.path.join(thisFilePath, "maze_1.png")},
          {'name': "Maze 2", 'filename': os.path.join(thisFilePath, "maze_2.png")},
          {'name': "Maze 3", 'filename': os.path.join(thisFilePath, "maze_3.png")},
         ]

precisions = [1e-1, 1e-2, 1e-3]
processes = ['gpu', 'cpu']


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ['visual', 'batch']:
        print("Please specify either 'visual' or 'batch' as an argument.")
        sys.exit(0)

    for img in images:
        print("Image: %s." % (img['name']))

        if sys.argv[1] == 'visual':
            harmonicMap = HarmonicMap()
            harmonicMap.load(img['filename'])
            #harmonicMap.show()

            timing = harmonicMap.solve(process='gpu', epsilon=1e-2)
            harmonicMap.show()

        elif sys.argv[1] == 'batch':
            for prec in precisions:
                for proc in processes:
                    print("%s %s " % (prec, proc), end='')
                    sys.stdout.flush()

                    harmonicMap = HarmonicMap()
                    harmonicMap.load(img['filename'])
                    timing = harmonicMap.solve(process=proc, epsilon=prec)

                    print("%.2f" % (timing[0]))

    print("Done.")

