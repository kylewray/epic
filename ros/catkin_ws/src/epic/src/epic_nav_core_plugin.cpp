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


#include <epic/epic_nav_core_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>

#include <epic/harmonic/harmonic_cpu.h>
#include <epic/harmonic/harmonic_gpu.h>
#include <epic/harmonic/harmonic_model_gpu.h>
#include <epic/harmonic/harmonic_path_cpu.h>
#include <epic/constants.h>
#include <epic/error_codes.h>

#include <stdio.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(epic::EpicNavCorePlugin, nav_core::BaseGlobalPlanner)

namespace epic {

#define COSTMAP_OBSTACLE_THRESHOLD 250
#define NUM_THREADS_GPU 1024

EpicNavCorePlugin::EpicNavCorePlugin()
{
    costmap = nullptr;
    initialized = false;

    harmonic.n = 0;
    harmonic.m = nullptr;
    harmonic.u = nullptr;
    harmonic.locked = nullptr;

    harmonic.epsilon = 1e-3;
    harmonic.delta = 0.0f;
    harmonic.numIterationsToStaggerCheck = 100;

    harmonic.currentIteration = 0;

    harmonic.d_m = nullptr;
    harmonic.d_u = nullptr;
    harmonic.d_locked = nullptr;
    harmonic.d_delta = nullptr;
}


EpicNavCorePlugin::EpicNavCorePlugin(std::string name,
        costmap_2d::Costmap2DROS *costmap_ros)
{
    costmap = nullptr;
    initialized = false;

    harmonic.n = 0;
    harmonic.m = nullptr;
    harmonic.u = nullptr;
    harmonic.locked = nullptr;

    harmonic.epsilon = 1e-3;
    harmonic.delta = 0.0f;
    harmonic.numIterationsToStaggerCheck = 100;

    harmonic.currentIteration = 0;

    harmonic.d_m = nullptr;
    harmonic.d_u = nullptr;
    harmonic.d_locked = nullptr;
    harmonic.d_delta = nullptr;

    initialize(name, costmap_ros);
}


EpicNavCorePlugin::~EpicNavCorePlugin()
{
    uninitialize();

    costmap = nullptr;
    initialized = false;
}


void EpicNavCorePlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    if (name.length() == 0 || costmap_ros == nullptr) {
        ROS_ERROR("Error[EpicNavCorePlugin::initialize]: Costmap2DROS object is not initialized.");
        return;
    }

    uninitialize();

    costmap = costmap_ros->getCostmap();

    harmonic.n = 2;
    harmonic.m = new unsigned int[2];
    harmonic.m[0] = costmap->getSizeInCellsY();
    harmonic.m[1] = costmap->getSizeInCellsX();

    harmonic.u = new float[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];
    harmonic.locked = new unsigned int[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];

    setCellsFromCostmap();
    setBoundariesAsObstacles();

    ros::NodeHandle privateNodeHandle("~/" + name);
    pub_plan = privateNodeHandle.advertise<nav_msgs::Path>("plan", 1);
    //pub_potential = privateNodeHandle.advertise<nav_msgs::OccupancyGrid>("potential", 1);

    initialized = true;
}


void EpicNavCorePlugin::setCellsFromCostmap()
{
    if (costmap == nullptr) {
        ROS_ERROR("Error[EpicNavCorePlugin::setCellsFromCostmap]: Costmap2D object is not initialized.");
        return;
    }

    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr) {
        ROS_ERROR("Error[EpicNavCorePlugin::setCellsFromCostmap]: Harmonic object is not initialized.");
        return;
    }

    // Assign all obstacles and free space. `Goals' (cells with cost 0) become free space.
    // Note that this does not go over the boundary; it is guaranteed to be an obstacle.
    for (unsigned int y = 1; y < harmonic.m[0] - 1; y++) {
        for (unsigned int x = 1; x < harmonic.m[1] - 1; x++) {
            if (costmap->getCost(x, y) >= COSTMAP_OBSTACLE_THRESHOLD) {
                harmonic.u[y * harmonic.m[1] + x] = EPIC_LOG_SPACE_OBSTACLE;
                harmonic.locked[y * harmonic.m[1] + x] = 1;
            } else {
                harmonic.u[y * harmonic.m[1] + x] = EPIC_LOG_SPACE_FREE;
                harmonic.locked[y * harmonic.m[1] + x] = 0;
            }
        }
    }
}


void EpicNavCorePlugin::setBoundariesAsObstacles()
{
    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr) {
        ROS_ERROR("Error[EpicNavCorePlugin::setBoundariesAsObstacles]: Harmonic object is not initialized.");
        return;
    }

    for (unsigned int y = 0; y < harmonic.m[0]; y++) {
        harmonic.u[y * harmonic.m[1] + 0] = EPIC_LOG_SPACE_OBSTACLE;
        harmonic.locked[y * harmonic.m[1] + 0] = 1;
        harmonic.u[y * harmonic.m[1] + (harmonic.m[1] - 1)] = EPIC_LOG_SPACE_OBSTACLE;
        harmonic.locked[y * harmonic.m[1] + (harmonic.m[1] - 1)] = 1;
    }

    for (unsigned int x = 0; x < harmonic.m[1]; x++) {
        harmonic.u[0 * harmonic.m[1] + x] = EPIC_LOG_SPACE_OBSTACLE;
        harmonic.locked[0 * harmonic.m[1] + x] = 1;
        harmonic.u[(harmonic.m[0] - 1) * harmonic.m[1] + x] = EPIC_LOG_SPACE_OBSTACLE;
        harmonic.locked[(harmonic.m[0] - 1) * harmonic.m[1] + x] = 1;
    }
}


void EpicNavCorePlugin::uninitialize()
{
    if (harmonic.u != nullptr) {
        delete [] harmonic.u;
    }
    harmonic.u = nullptr;

    if (harmonic.locked != nullptr) {
        delete [] harmonic.locked;
    }
    harmonic.locked = nullptr;

    if (harmonic.m != nullptr) {
        delete [] harmonic.m;
    }
    harmonic.m = nullptr;

    harmonic.n = 0;
}


void EpicNavCorePlugin::mapToWorld(float mx, float my, float &wx, float &wy) const
{
    wx = costmap->getOriginX() + mx * costmap->getResolution();
    wy = costmap->getOriginY() + my * costmap->getResolution();
}


bool EpicNavCorePlugin::worldToMap(float wx, float wy, float &mx, float &my) const
{
    if (wx < costmap->getOriginX() || wy < costmap->getOriginY() ||
            wx >= costmap->getOriginX() + costmap->getSizeInMetersX() ||
            wy >= costmap->getOriginY() + costmap->getSizeInMetersY()) {
        ROS_WARN("Error[EpicNavCorePlugin::worldToMap]: World coordinates are outside map.");
        return false;
    }

    mx = (wx - costmap->getOriginX()) / costmap->getResolution();
    my = (wy - costmap->getOriginY()) / costmap->getResolution();

    return true;
}


bool EpicNavCorePlugin::makePlan(const geometry_msgs::PoseStamped &start,
        const geometry_msgs::PoseStamped &goal,
        std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized) {
        ROS_ERROR("Error[EpicNavCorePlugin::makePlan]: EpicNavCorePlugin has not been initialized yet.");
        return false;
    }

    // Set the goal location on the potential map.
    unsigned int x_coord = 0;
    unsigned int y_coord = 0;

    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, x_coord, y_coord)) {
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Could not convert goal to cost map location.");
        x_coord = 0;
        y_coord = 0;
    }

    setGoal(x_coord, y_coord);

    ROS_INFO("Information[EpicNavCorePlugin::makePlan]: Solving harmonic function...");
    int result = harmonic_complete_gpu(&harmonic, NUM_THREADS_GPU);

    if (result != EPIC_SUCCESS) {
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Could not execute GPU version of 'epic' library.");
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Trying CPU fallback...");

        result = harmonic_complete_cpu(&harmonic);
    }

    if (result == EPIC_SUCCESS) {
        ROS_INFO("Information[EpicNavCorePlugin::makePlan]: Successfully solved harmonic function!");
    } else {
        ROS_ERROR("Error[EpicNavCorePlugin::makePlan]: Failed to solve harmonic function.");
        return false;
    }

    //pub_potential.publish(harmonic.u);

    plan.clear();
    plan.push_back(start);

    /*
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, xCoord, yCoord)) {
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Could not convert start to cost map location.");
        xCoord = 0;
        yCoord = 0;
    }
    */

    float x = 0.0f;
    float y = 0.0f;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, x, y)) {
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Could not convert start to floating point cost map location.");
    }

    float step_size = 0.05f;
    float cd_precision = 0.5f;
    unsigned int max_length = harmonic.m[0] * harmonic.m[1] / step_size;

    unsigned int k = 0;
    float *raw_plan = nullptr;

    result = harmonic_compute_path_2d_cpu(&harmonic, x, y, step_size, cd_precision, max_length, k, raw_plan);

    if (result != EPIC_SUCCESS) {
        ROS_ERROR("Error[EpicNavCorePlugin::makePlan]: Failed to compute the path.");

        if (raw_plan != nullptr) {
            delete [] raw_plan;
        }

        return false;
    }

    geometry_msgs::PoseStamped new_goal = goal;

    for (unsigned int i = 1; i < k; i++) {
        float x = raw_plan[2 * i + 0];
        float y = raw_plan[2 * i + 1];

        float plan_theta = std::atan2(y - raw_plan[2 * (i - 1) + 1], x - raw_plan[2 * (i - 1) + 0]);

        float plan_x = 0.0f;
        float plan_y = 0.0f;

        mapToWorld(x, y, plan_x, plan_y);

        new_goal.pose.position.x = plan_x;
        new_goal.pose.position.y = plan_y;
        new_goal.pose.orientation = tf::createQuaternionMsgFromYaw(plan_theta);

        plan.push_back(new_goal);
    }

    delete [] raw_plan;
    raw_plan = nullptr;

    plan.push_back(goal);

    publishPlan(plan);

    return true;
}


void EpicNavCorePlugin::setGoal(unsigned int x_goal, unsigned int y_goal)
{
    if (!initialized) {
        ROS_ERROR("Error[EpicNavCorePlugin::setGoal]: EpicNavCorePlugin has not been initialized yet.");
        return;
    }

    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr) {
        ROS_ERROR("Error[EpicNavCorePlugin::setBoundariesAsObstacles]: Harmonic object is not initialized.");
        return;
    }

    // All current goals become free cells. Boundaries are always obstacles.
    for (unsigned int y = 1; y < harmonic.m[0] - 1; y++) {
        for (unsigned int x = 1; x < harmonic.m[1] - 1; x++) {
            if (harmonic.u[y * harmonic.m[1] + x] == EPIC_LOG_SPACE_GOAL) {
                harmonic.u[y * harmonic.m[1] + x] = EPIC_LOG_SPACE_FREE;
                harmonic.locked[y * harmonic.m[1] + x] = 0;
            }
        }
    }

    // The point provided is in cell-coordinates, so assign it!
    harmonic.u[y_goal * harmonic.m[1] + x_goal] = EPIC_LOG_SPACE_GOAL;
    harmonic.locked[y_goal * harmonic.m[1] + x_goal] = 1;
}


void EpicNavCorePlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized) {
        ROS_ERROR("Error[EpicNavCorePlugin::publishPlan]: The planner has not been initialized yet.");
        return;
    }

    nav_msgs::Path gui_plan;
    if (!plan.empty()) {
        gui_plan.header.frame_id = plan[0].header.frame_id;
        gui_plan.header.stamp = plan[0].header.stamp;
    }

    gui_plan.poses.resize(plan.size());
    for (unsigned int i = 0; i < plan.size(); i++) {
        gui_plan.poses[i] = plan[i];
    }

    pub_plan.publish(gui_plan);
}

}; // namespace epic_plan

