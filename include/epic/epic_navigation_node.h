/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2016 Kyle Hollins Wray, University of Massachusetts
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


#ifndef EPIC_NAVIGATION_NODE_H
#define EPIC_NAVIGATION_NODE_H


#include <ros/ros.h>

namespace epic {

class EpicNavigationNode {
public:
    /**
     *  The default constructor for the EpicNavigationNode class.
     *  @param nh       The node handle from main.
     */
    EpicNavigationNode(ros::NodeHandle &nh);

    /**
     *  The deconstructor for the EpicNavigationNode class.
     */
    virtual ~EpicNavigationNode();

    /**
     *  Initialize the services, messages, and algorithm variables. This must be implemented.
     *  @return True if successful in registering, subscribing, etc.; false otherwise.
     */
    virtual bool initialize() = 0;

    /**
     *  Update the harmonic function one step. This must be implemented.
     *  @param  num_steps   The number of steps to do.
     */
    virtual void update(unsigned int num_steps) = 0;

protected:
    // A private node handle; usually a reference to the one created in the node's "main" function.
    ros::NodeHandle private_node_handle;

};

};


#endif // EPIC_NAVIGATION_NODE_H

