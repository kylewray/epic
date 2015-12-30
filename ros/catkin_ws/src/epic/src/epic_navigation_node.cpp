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

#include <harmonic/harmonic_cpu.h>
#include <harmonic/harmonic_gpu.h>
#include <harmonic/harmonic_model_gpu.h>
#include <harmonic/harmonic_path_cpu.h>
#include <error_codes.h>

//using std::string;

namespace epic {

#define NUM_THREADS_GPU 1024

EpicNavigationNode::EpicNavigationNode()
{
    initialized = false;
    algorithm = EPIC_ALGORITHM_HARMONIC;
    gpu = true;

    harmonic.n = 0;
    harmonic.m = NULL;
    harmonic.u = NULL;
    harmonic.locked = NULL;

    harmonic.epsilon = 1e-3;
    harmonic.delta = 0.0f;
    harmonic.numIterationsToStaggerCheck = 100;

    harmonic.currentIteration = 0;

    harmonic.d_m = NULL;
    harmonic.d_u = NULL;
    harmonic.d_locked = NULL;
    harmonic.d_delta = NULL;
}


EpicNavigationNode::EpicNavigationNode(unsigned int alg)
{
    initialized = false;
    algorithm = alg;
    if (alg >= NUM_EPIC_ALGORITHMS) {
        alg = EPIC_ALGORITHM_HARMONIC;
    }
    gpu = true;

    harmonic.n = 0;
    harmonic.m = NULL;
    harmonic.u = NULL;
    harmonic.locked = NULL;

    harmonic.epsilon = 1e-3;
    harmonic.delta = 0.0f;
    harmonic.numIterationsToStaggerCheck = 100;

    harmonic.currentIteration = 0;

    harmonic.d_m = NULL;
    harmonic.d_u = NULL;
    harmonic.d_locked = NULL;
    harmonic.d_delta = NULL;
}


EpicNavigationNode::~EpicNavigationNode()
{

}


bool EpicNavigationNode::initialize(std::string name)
{
    if (initialized) {
        return false;
    }

    ros::NodeHandle private_node_handle("~/" + name);

    sub_occupancy_grid = private_node_handle.subscribe("epic_navigation_occupancy_grid", 100,
                                                        &EpicNavigationNode::subOccupancyGrid, this);

    srv_add_goals = private_node_handle.advertiseService("epic_navigation_add_goals",
                                                        &EpicNavigationNode::srvAddGoals, this);

    srv_remove_goals = private_node_handle.advertiseService("epic_navigation_remove_goals",
                                                        &EpicNavigationNode::srvRemoveGoals, this);

    srv_compute_path = private_node_handle.advertiseService("epic_navigation_compute_path",
                                                        &EpicNavigationNode::srvComputePath, this);

    initialized = true;

    return true;
}


void EpicNavigationNode::update(unsigned int num_steps)
{
    if (!initialized) {
        return;
    }

    // TODO: Check if width or height changed. If so, clear everything.
    // Note we don't care if the resolution or offsets change.

    // Depending on the algorithm, perform different update actions.
    if (algorithm == EPIC_ALGORITHM_HARMONIC) {
        // Perform steps of the harmonic function. Do not check for convergence.
        if (gpu) {
            for (unsigned int i = 0; i < num_steps; i++) {
                int result = harmonic_update_gpu(&harmonic, NUM_THREADS_GPU);
                if (result != EPIC_SUCCESS) {
                    ROS_ERROR("Error[EpicNavigationNode::update]: Failed to use GPU to relax harmonic function with 'epic' library.");
                    return;
                }
            }
        } else {
            for (unsigned int i = 0; i < num_steps; i++) {
                int result = harmonic_update_cpu(&harmonic);
                if (result != EPIC_SUCCESS) {
                    ROS_ERROR("Error[EpicNavigationNode::update]: Failed to use CPU to relax harmonic function with 'epic' library.");
                    return;
                }
            }
        }
    } else {
    }
}


void EpicNavigationNode::subOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // TODO: If the size changed, then free everything and start over.

    // TODO: If it fails to initialize the GPU model, then gpu is assigned to false. Otherwise, we will use it.
}


bool EpicNavigationNode::srvAddGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{

}


bool EpicNavigationNode::srvRemoveGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{

}


bool EpicNavigationNode::srvComputePath(epic::ComputePath::Request &req, epic::ComputePath::Response &res)
{

}


}; // namespace epic

