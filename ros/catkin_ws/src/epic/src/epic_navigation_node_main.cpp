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


#include <ros/ros.h>

#include <epic/epic_navigation_node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "epic_navigation_node");

    ros::NodeHandle node_handle;

    // TODO: Read a ROS parameter for this node to assign the desired algorithm for EpicNavigation.
    epic::EpicNavigationNode epic_navigation_node;
    epic_navigation_node.initialize("epic_navigation_node");

    // TODO: Read a ROS parameter for the number of update steps of the algorithm at a time.
    unsigned int num_update_steps = 10;

    // TODO: Read a ROS parameter for how many updates should be called per second. (Default is 10Hz.)
    ros::Rate rate(10);

    while (ros::ok()) {
        // Perform an update of the harmonic function or other navigation planning algorithm.
        epic_navigation_node.update(num_update_steps);

        // Check for service calls.
        ros::spinOnce();

        // Sleep and let other processes think.
        rate.sleep();
    }

    return 0;
}
