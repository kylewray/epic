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


#include <epic/epic_plan_nav_core.h>
#include <pluginlib/class_list_macros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>

#include <harmonic/harmonic_cpu.h>
#include <harmonic/harmonic_gpu.h>
#include <harmonic/harmonic_model_gpu.h>

#include <stdio.h>

PLUGINLIB_EXPORT_CLASS(epic_plan::EpicPlanNavCore, nav_core::BaseGlobalPlanner)


using namespace std;

namespace epic_plan {


#define COSTMAP_OBSTACLE_THRESHOLD 250

#define LOG_SPACE_GOAL 0.0
#define LOG_SPACE_OBSTACLE -1e6
#define LOG_SPACE_FREE -1e6


EpicPlanNavCore::EpicPlanNavCore()
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


EpicPlanNavCore::EpicPlanNavCore(std::string name,
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


EpicPlanNavCore::~EpicPlanNavCore()
{
    uninitialize();

    costmap = NULL;
    initialized = false;
}


void EpicPlanNavCore::initialize(std::string name, costmap_2d::Costmap2DROS *costmapROS)
{
    if (name.length() == 0 || costmapROS == NULL) {
        ROS_ERROR("Error[EpicPlanNavCore::initialize]: Costmap2DROS object is not initialized.");
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


void EpicPlanNavCore::setCellsFromCostmap()
{
    if (costmap == NULL) {
        ROS_ERROR("Error[EpicPlanNavCore::setCellsFromCostmap]: Costmap2D object is not initialized.");
        return;
    }

    if (harmonic.m == NULL || harmonic.u == NULL || harmonic.locked == NULL) {
        ROS_ERROR("Error[EpicPlanNavCore::setCellsFromCostmap]: Harmonic object is not initialized.");
        return;
    }

    // Assign all obstacles and free space. `Goals' (cells with cost 0) become free space.
    // Note that this does not go over the boundary; it is guaranteed to be an obstacle.
    for (unsigned int y = 1; y < harmonic.m[0] - 1; y++) {
        for (unsigned int x = 1; x < harmonic.m[1] - 1; x++) {
            if (costmap->getCost(x, y) >= COSTMAP_OBSTACLE_THRESHOLD) {
                harmonic.u[y * harmonic.m[1] + x] = LOG_SPACE_OBSTACLE;
                harmonic.locked[y * harmonic.m[1] + x] = 1;
            } else {
                harmonic.u[y * harmonic.m[1] + x] = LOG_SPACE_FREE;
                harmonic.locked[y * harmonic.m[1] + x] = 0;
            }
        }
    }
}


void EpicPlanNavCore::setBoundariesAsObstacles()
{
    if (harmonic.m == NULL || harmonic.u == NULL || harmonic.locked == NULL) {
        ROS_ERROR("Error[EpicPlanNavCore::setBoundariesAsObstacles]: Harmonic object is not initialized.");
        return;
    }

    for (unsigned int y = 0; y < harmonic.m[0]; y++) {
        harmonic.u[y * harmonic.m[1] + 0] = LOG_SPACE_OBSTACLE;
        harmonic.locked[y * harmonic.m[1] + 0] = 1;
        harmonic.u[y * harmonic.m[1] + (harmonic.m[1] - 1)] = LOG_SPACE_OBSTACLE;
        harmonic.locked[y * harmonic.m[1] + (harmonic.m[1] - 1)] = 1;
    }

    for (unsigned int x = 0; x < harmonic.m[1]; x++) {
        harmonic.u[0 * harmonic.m[1] + x] = LOG_SPACE_OBSTACLE;
        harmonic.locked[0 * harmonic.m[1] + x] = 1;
        harmonic.u[(harmonic.m[0] - 1) * harmonic.m[1] + x] = LOG_SPACE_OBSTACLE;
        harmonic.locked[(harmonic.m[0] - 1) * harmonic.m[1] + x] = 1;
    }
}


void EpicPlanNavCore::uninitialize()
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


float EpicPlanNavCore::computePotential(float x, float y)
{
    unsigned int xCellIndex = 0;
    unsigned int yCellIndex = 0;

    computeCellIndex(x, y, xCellIndex, yCellIndex);

    if (harmonic.locked[yCellIndex * harmonic.m[1] + xCellIndex] == 1 &&
            harmonic.u[yCellIndex * harmonic.m[1] + xCellIndex] < 0.0f) {
        ROS_ERROR("Error[EpicPlanNavCore::computePotential]: Badness in computing potential...");
        return 0.0f;
    }

    unsigned int xtl = (unsigned int)(x - 0.5f); // * costmap->getResolution());
    unsigned int ytl = (unsigned int)(y - 0.5f); // * costmap->getResolution());

    unsigned int xtr = (unsigned int)(x + 0.5f); // * costmap->getResolution());
    unsigned int ytr = (unsigned int)(y - 0.5f); // * costmap->getResolution());

    unsigned int xbl = (unsigned int)(x - 0.5f); // * costmap->getResolution());
    unsigned int ybl = (unsigned int)(y + 0.5f); // * costmap->getResolution());

    unsigned int xbr = (unsigned int)(x + 0.5f); // * costmap->getResolution());
    unsigned int ybr = (unsigned int)(y + 0.5f); // * costmap->getResolution());

    float alpha = (x - xtl); // / costmap->getResolution();
    float beta = (y - ytl); // / costmap->getResolution();

    float one = (1.0f - alpha) * harmonic.u[ytl * harmonic.m[1] + xtl] +
                alpha * harmonic.u[ytr * harmonic.m[1] + xtr];
    float two = (1.0f - alpha) * harmonic.u[ybl * harmonic.m[1] + xbl] +
                alpha * harmonic.u[ybr * harmonic.m[1] + xbr];
    float three = (1.0f - beta) * one + beta * two;

    return three;
}


void EpicPlanNavCore::computeCellIndex(float x, float y,
        unsigned int &xCellIndex, unsigned int &yCellIndex)
{
    xCellIndex = (unsigned int)(x + 0.5f * costmap->getResolution());
    yCellIndex = (unsigned int)(y + 0.5f * costmap->getResolution());
}


void EpicPlanNavCore::mapToWorld(float mx, float my, float &wx, float &wy) const
{
    wx = costmap->getOriginX() + mx * costmap->getResolution();
    wy = costmap->getOriginY() + my * costmap->getResolution();
}


bool EpicPlanNavCore::worldToMap(float wx, float wy, float &mx, float &my) const
{
    if (wx < costmap->getOriginX() || wy < costmap->getOriginY() ||
            wx >= costmap->getSizeInMetersX() || wy >= costmap->getSizeInMetersY()) {
        ROS_WARN("Error[EpicPlanNavCore::worldToMap]: World coordinates are outside map.");
        return true;
    }

    mx = (wx - costmap->getOriginX()) / costmap->getResolution();
    my = (wy - costmap->getOriginY()) / costmap->getResolution();

    return false;
}


bool EpicPlanNavCore::makePlan(const geometry_msgs::PoseStamped &start,
        const geometry_msgs::PoseStamped &goal,
        std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized) {
        ROS_ERROR("Error[EpicPlanNavCore::makePlan]: EpicPlanNavCore has not been initialized yet.");
        return false;
    }

    // Set the goal location on the potential map.
    unsigned int xCoord = 0;
    unsigned int yCoord = 0;

    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, xCoord, yCoord)) {
        ROS_WARN("Warning[EpicPlanNavCore::makePlan]: Could not convert goal to cost map location.");
        xCoord = 0;
        yCoord = 0;
    }

    setGoal(xCoord, yCoord);

    ROS_WARN("Solving harmonic function...");
    harmonic_complete_gpu(&harmonic, 1024);
    ROS_WARN("Done!");

    //pubPotential.publish(harmonic.u);

    plan.clear();
    plan.push_back(start);

    costmap->worldToMap(start.pose.position.x, start.pose.position.y, xCoord, yCoord);

    //xCellIndex, yCellIndex = self._compute_cell_index(x, y)

    unsigned int xCellIndex = xCoord;
    unsigned int yCellIndex = yCoord;

    float planStepSize = 0.05;
    float centralDifferenceStepSize = 0.5;
    unsigned int maxPoints = harmonic.m[0] * harmonic.m[1] / planStepSize;

    float x = 0.0f;
    float y = 0.0f;
    worldToMap(start.pose.position.x, start.pose.position.y, x, y);

    geometry_msgs::PoseStamped newGoal = goal;

    while (harmonic.locked[yCellIndex * harmonic.m[1] + xCellIndex] != 1 && plan.size() < maxPoints) {
        float value0 = computePotential(x - centralDifferenceStepSize, y);
        float value1 = computePotential(x + centralDifferenceStepSize, y);
        float value2 = computePotential(x, y - centralDifferenceStepSize);
        float value3 = computePotential(x, y + centralDifferenceStepSize);

        float partialX = (value1 - value0) / (2.0f * centralDifferenceStepSize);
        float partialY = (value3 - value2) / (2.0f * centralDifferenceStepSize);

        float denom = std::sqrt(std::pow(partialX, 2) + std::pow(partialY, 2));
        partialX /= denom;
        partialY /= denom;

        x += partialX * planStepSize;
        y += partialY * planStepSize;

        float planX = 0.0f;
        float planY = 0.0f;
        mapToWorld(x, y, planX, planY);

        float planTheta = std::atan2(partialY, partialX);

        computeCellIndex(x, y, xCellIndex, yCellIndex);

        newGoal.pose.position.x = planX;
        newGoal.pose.position.y = planY;
        newGoal.pose.orientation = tf::createQuaternionMsgFromYaw(planTheta);

        plan.push_back(newGoal);
    }

    plan.push_back(goal);

    publishPlan(plan);

    return true;
}


void EpicPlanNavCore::setGoal(unsigned int xGoal, unsigned int yGoal)
{
    if (!initialized) {
        ROS_ERROR("Error[EpicPlanNavCore::setGoal]: EpicPlanNavCore has not been initialized yet.");
        return;
    }

    if (harmonic.m == NULL || harmonic.u == NULL || harmonic.locked == NULL) {
        ROS_ERROR("Error[EpicPlanNavCore::setBoundariesAsObstacles]: Harmonic object is not initialized.");
        return;
    }

    // All current goals become free cells. Boundaries are always obstacles.
    for (unsigned int y = 1; y < harmonic.m[0] - 1; y++) {
        for (unsigned int x = 1; x < harmonic.m[1] - 1; x++) {
            if (harmonic.u[y * harmonic.m[1] + x] == LOG_SPACE_GOAL) {
                harmonic.u[y * harmonic.m[1] + x] = LOG_SPACE_FREE;
                harmonic.locked[y * harmonic.m[1] + x] = 0;
            }
        }
    }

    // The point provided is in cell-coordinates, so assign it!
    harmonic.u[yGoal * harmonic.m[1] + xGoal] = LOG_SPACE_GOAL;
    harmonic.locked[yGoal * harmonic.m[1] + xGoal] = 1;
}


void EpicPlanNavCore::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path) {
    if (!initialized) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    pubPlan.publish(gui_path);
}

}; // namespace epic_plan

