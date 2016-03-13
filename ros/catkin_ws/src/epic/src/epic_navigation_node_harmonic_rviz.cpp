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

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <epic/epic_navigation_node_harmonic_rviz.h>
#include <epic/epic_navigation_node_constants.h>

#include <epic/harmonic/harmonic_cpu.h>
#include <epic/harmonic/harmonic_gpu.h>
#include <epic/harmonic/harmonic_model_gpu.h>
#include <epic/harmonic/harmonic_path_cpu.h>
#include <epic/harmonic/harmonic_utilities_cpu.h>
#include <epic/harmonic/harmonic_utilities_gpu.h>
#include <epic/error_codes.h>
#include <epic/constants.h>

namespace epic {

EpicNavigationNodeHarmonicRviz::EpicNavigationNodeHarmonicRviz(ros::NodeHandle &nh) :
        EpicNavigationNodeHarmonic(nh)
        //private_node_handle(nh)
{
    goal_added = false;
}


EpicNavigationNodeHarmonicRviz::~EpicNavigationNodeHarmonicRviz()
{ }


bool EpicNavigationNodeHarmonicRviz::initialize()
{
    if (init_msgs) {
        return false;
    }

    EpicNavigationNodeHarmonic::initialize();

    // The follow subscribers/publishers are exclusively for simplified interaction with rviz.
    std::string sub_map_pose_estimate_topic;
    private_node_handle.param<std::string>("/epic_navigation_node/sub_map_pose_estimate",
                                            sub_map_pose_estimate_topic,
                                            "/initialpose");
    sub_map_pose_estimate = private_node_handle.subscribe(sub_map_pose_estimate_topic,
                                                        10,
                                                        &EpicNavigationNodeHarmonicRviz::subMapPoseEstimate,
                                                        this);

    std::string sub_map_nav_goal_topic;
    private_node_handle.param<std::string>("/epic_navigation_node/sub_map_nav_goal",
                                            sub_map_nav_goal_topic,
                                            "/move_base_simple/goal");
    sub_map_nav_goal = private_node_handle.subscribe(sub_map_nav_goal_topic,
                                                    10,
                                                    &EpicNavigationNodeHarmonicRviz::subMapNavGoal,
                                                    this);

    std::string sub_map_path_topic;
    private_node_handle.param<std::string>("/epic_navigation_node/sub_map_path",
                                            sub_map_path_topic,
                                            "path");
    pub_map_path = private_node_handle.advertise<nav_msgs::Path>(sub_map_path_topic, 1);

    init_msgs = true;

    return true;
}


void EpicNavigationNodeHarmonicRviz::subMapPoseEstimate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    current_pose.header.frame_id = msg->header.frame_id;
    current_pose.header.stamp = msg->header.stamp;

    current_pose.pose.position.x = msg->pose.pose.position.x;
    current_pose.pose.position.y = msg->pose.pose.position.y;
    current_pose.pose.position.z = msg->pose.pose.position.z;

    current_pose.pose.orientation.x = msg->pose.pose.orientation.x;
    current_pose.pose.orientation.y = msg->pose.pose.orientation.y;
    current_pose.pose.orientation.z = msg->pose.pose.orientation.z;
    current_pose.pose.orientation.w = msg->pose.pose.orientation.w;

    epic::ComputePath::Request req;
    epic::ComputePath::Response res;

    req.start = current_pose;

    req.step_size = 0.05;
    req.precision = 0.5;
    req.max_length = width * height / req.step_size;

    EpicNavigationNodeHarmonic::srvComputePath(req, res);

    pub_map_path.publish(res.path);
}


void EpicNavigationNodeHarmonicRviz::subMapNavGoal(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    epic::ModifyGoals::Request req;
    epic::ModifyGoals::Response res;

    if (goal_added) {
        req.goals.push_back(last_goal);
        EpicNavigationNodeHarmonic::srvRemoveGoals(req, res);
    }

    last_goal.header.frame_id = msg->header.frame_id;
    last_goal.header.stamp = msg->header.stamp;

    last_goal.pose.position.x = msg->pose.position.x;
    last_goal.pose.position.y = msg->pose.position.y;
    last_goal.pose.position.z = msg->pose.position.z;

    last_goal.pose.orientation.x = msg->pose.orientation.x;
    last_goal.pose.orientation.y = msg->pose.orientation.y;
    last_goal.pose.orientation.z = msg->pose.orientation.z;
    last_goal.pose.orientation.w = msg->pose.orientation.w;

    req.goals.clear();
    req.goals.push_back(last_goal);
    EpicNavigationNodeHarmonic::srvAddGoals(req, res);

    goal_added = true;
}

}; // namespace epic


