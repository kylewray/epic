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
#include <epic/epic_navigation_node_harmonic.h>
#include <epic/epic_navigation_node_harmonic_rviz.h>
//#include <epic/epic_navigation_node_ompl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "epic_navigation_node");

    ros::NodeHandle node_handle("~");

    // Note: The extra "~" is not required in here because private_node_handle is initialized
    // with this relative namespace already. Basically, it is already in the string. This
    // differs from Python, which does not use a private node handle.

    // Read a ROS parameter for this node to assign the desired algorithm for EpicNavigationNode.
    std::string algorithm;
    node_handle.param<std::string>("algorithm", algorithm, "harmonic");

    // Read a ROS parameter for if Rviz message support should be included.
    bool rviz_support = false;
    node_handle.param<bool>("rviz_support", rviz_support, false);

    epic::EpicNavigationNode *epic_navigation_node;
    
    if (algorithm == "harmonic") {
        if (!rviz_support) {
            epic_navigation_node = new epic::EpicNavigationNodeHarmonic(node_handle);
        } else {
            epic_navigation_node = new epic::EpicNavigationNodeHarmonicRviz(node_handle);
        }
    }

    epic_navigation_node->initialize();

    // Read a ROS parameter for the number of update steps of the algorithm at a time.
    int steps_per_update = 50;
    node_handle.param<int>("steps_per_update", steps_per_update, 50);

    // Read a ROS parameter for how many updates should be called per second. (Default is 10Hz.)
    int update_rate = 10;
    node_handle.param<int>("update_rate", update_rate, 10);

    ros::Rate rate(update_rate);

    while (ros::ok()) {
        // Check for service calls.
        ros::spinOnce();

        // Perform an update of the harmonic function or other navigation planning algorithm.
        epic_navigation_node->update(steps_per_update);

        // Sleep and let other processes think.
        rate.sleep();
    }

    delete epic_navigation_node;

    return 0;
}

