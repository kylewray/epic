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

#include <harmonic/harmonic_cpu.h>
#include <harmonic/harmonic_gpu.h>
#include <harmonic/harmonic_model_gpu.h>
#include <harmonic/harmonic_path_cpu.h>
#include <error_codes.h>

#include <stdio.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(epic::EpicNavCorePlugin, nav_core::BaseGlobalPlanner)

namespace epic {

#define COSTMAP_OBSTACLE_THRESHOLD 250
#define NUM_THREADS_GPU 1024

EpicNavCorePlugin::EpicNavCorePlugin()
{
    costmap = NULL;
    initialized = false;

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


EpicNavCorePlugin::EpicNavCorePlugin(std::string name,
        costmap_2d::Costmap2DROS *costmapROS)
{
    costmap = NULL;
    initialized = false;

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

    initialize(name, costmapROS);
}


EpicNavCorePlugin::~EpicNavCorePlugin()
{
    uninitialize();

    costmap = NULL;
    initialized = false;
}


void EpicNavCorePlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmapROS)
{
    if (name.length() == 0 || costmapROS == NULL) {
        ROS_ERROR("Error[EpicNavCorePlugin::initialize]: Costmap2DROS object is not initialized.");
        return;
    }

    uninitialize();

    costmap = costmapROS->getCostmap();

    harmonic.n = 2;
    harmonic.m = new unsigned int[2];
    harmonic.m[0] = costmap->getSizeInCellsY();
    harmonic.m[1] = costmap->getSizeInCellsX();

    harmonic.u = new float[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];
    harmonic.locked = new unsigned int[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];

    setCellsFromCostmap();
    setBoundariesAsObstacles();

    ros::NodeHandle privateNodeHandle("~/" + name);
    pubPlan = privateNodeHandle.advertise<nav_msgs::Path>("plan", 1);
    pubPotential = privateNodeHandle.advertise<nav_msgs::OccupancyGrid>("potential", 1);

    initialized = true;
}


void EpicNavCorePlugin::setCellsFromCostmap()
{
    if (costmap == NULL) {
        ROS_ERROR("Error[EpicNavCorePlugin::setCellsFromCostmap]: Costmap2D object is not initialized.");
        return;
    }

    if (harmonic.m == NULL || harmonic.u == NULL || harmonic.locked == NULL) {
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
    if (harmonic.m == NULL || harmonic.u == NULL || harmonic.locked == NULL) {
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
    if (harmonic.u != NULL) {
        delete [] harmonic.u;
    }
    harmonic.u = NULL;

    if (harmonic.locked != NULL) {
        delete [] harmonic.locked;
    }
    harmonic.locked = NULL;

    if (harmonic.m != NULL) {
        delete [] harmonic.m;
    }
    harmonic.m = NULL;

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
            wx >= costmap->getSizeInMetersX() || wy >= costmap->getSizeInMetersY()) {
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
    unsigned int xCoord = 0;
    unsigned int yCoord = 0;

    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, xCoord, yCoord)) {
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Could not convert goal to cost map location.");
        xCoord = 0;
        yCoord = 0;
    }

    setGoal(xCoord, yCoord);

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

    //pubPotential.publish(harmonic.u);

    plan.clear();
    plan.push_back(start);

    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, xCoord, yCoord)) {
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Could not convert start to cost map location.");
        xCoord = 0;
        yCoord = 0;
    }

    float x = 0.0f;
    float y = 0.0f;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, x, y)) {
        ROS_WARN("Warning[EpicNavCorePlugin::makePlan]: Could not convert start to floating point cost map location.");
    }

    float stepSize = 0.05f;
    float cdPrecision = 0.5f;
    unsigned int maxLength = harmonic.m[0] * harmonic.m[1] / stepSize;

    unsigned int k = 0;
    float *rawPlan = NULL;

    result = harmonic_compute_path_2d_cpu(&harmonic, x, y, stepSize, cdPrecision, maxLength, k, rawPlan);

    if (result != EPIC_SUCCESS) {
        ROS_ERROR("Error[EpicNavCorePlugin::makePlan]: Failed to compute the path.");

        if (rawPlan != NULL) {
            delete [] rawPlan;
        }

        return false;
    }

    geometry_msgs::PoseStamped newGoal = goal;

    for (unsigned int i = 1; i < k; i++) {
        float x = rawPlan[2 * i + 0];
        float y = rawPlan[2 * i + 1];

        float planTheta = std::atan2(y - rawPlan[2 * (i - 1) + 1], x - rawPlan[2 * (i - 1) + 0]);

        float planX = 0.0f;
        float planY = 0.0f;

        mapToWorld(x, y, planX, planY);

        newGoal.pose.position.x = planX;
        newGoal.pose.position.y = planY;
        newGoal.pose.orientation = tf::createQuaternionMsgFromYaw(planTheta);

        plan.push_back(newGoal);
    }

    delete [] rawPlan;

    plan.push_back(goal);

    publishPlan(plan);

    return true;
}


void EpicNavCorePlugin::setGoal(unsigned int xGoal, unsigned int yGoal)
{
    if (!initialized) {
        ROS_ERROR("Error[EpicNavCorePlugin::setGoal]: EpicNavCorePlugin has not been initialized yet.");
        return;
    }

    if (harmonic.m == NULL || harmonic.u == NULL || harmonic.locked == NULL) {
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
    harmonic.u[yGoal * harmonic.m[1] + xGoal] = EPIC_LOG_SPACE_GOAL;
    harmonic.locked[yGoal * harmonic.m[1] + xGoal] = 1;
}


void EpicNavCorePlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
    if (!initialized) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_plan;
    gui_plan.poses.resize(plan.size());

    if (!plan.empty()) {
        gui_plan.header.frame_id = plan[0].header.frame_id;
        gui_plan.header.stamp = plan[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the plan is all in the same frame
    for (unsigned int i = 0; i < plan.size(); i++) {
        gui_plan.poses[i] = plan[i];
    }

    pubPlan.publish(gui_plan);
}

}; // namespace epic_plan

