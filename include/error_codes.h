/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2015 Kyle Hollins Wray, University of Massachusetts
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the "Software"), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#ifndef ERROR_CODES_H
#define ERROR_CODES_H


#define EPIC_SUCCESS                         0
#define EPIC_SUCCESS_AND_CONVERGED           1

#define EPIC_ERROR_INVALID_DATA              2
#define EPIC_ERROR_INVALID_CUDA_PARAM        3
#define EPIC_ERROR_DEVICE_MALLOC             4
#define EPIC_ERROR_MEMCPY_TO_DEVICE          5
#define EPIC_ERROR_MEMCPY_TO_HOST            6
#define EPIC_ERROR_DEVICE_FREE               7
#define EPIC_ERROR_KERNEL_EXECUTION          8
#define EPIC_ERROR_DEVICE_SYNCHRONIZE        9


#endif // ERROR_CODES_H

