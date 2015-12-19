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


PLUGINLIB_EXPORT_CLASS(epic_plan::EpicPlanNavCore, nav_core::BaseGlobalPlanner)


using namespace std;

namespace epic_plan {


EpicPlanNavCore::EpicPlanNavCore()
{
    costmap = NULL;
    initialized = false;

    harmonic.n = 0;
    harmonic.m = NULL;
    harmonic.u = NULL;
    harmonic.locked = NULL;
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

    ROS_WARN("Initializing...");

    uninitialize();

    costmap = costmapROS->getCostmap();

    //ROS_WARN("%i", costmap->getSizeInCellsX());
    //ROS_WARN("%i", costmap->getSizeInCellsY());

    harmonic.n = 2;
    harmonic.m = new unsigned int[2];
    harmonic.m[0] = costmap->getSizeInCellsX();
    harmonic.m[1] = costmap->getSizeInCellsY();

    harmonic.u = new float[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];
    harmonic.locked = new unsigned int[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];

    setBoundariesAsObstacles();

    initialized = true;

    ROS_WARN("Done.");
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
    for (unsigned int x = 1; x < harmonic.m[0] - 1; x++) {
        for (unsigned int y = 1; y < harmonic.m[1] - 1; y++) {
            if (costmap->getCost(x, y) == 100) {
                harmonic.u[x * harmonic.m[1] + y] = 1.0f;
                harmonic.locked[x * harmonic.m[1] + y] = 1;
            } else {
                harmonic.u[x * harmonic.m[1] + y] = 1.0f;
                harmonic.locked[x * harmonic.m[1] + y] = 0;
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

    for (unsigned int x = 0; x < harmonic.m[0]; x++) {
        harmonic.u[x * harmonic.m[1] + 0] = 1.0f;
        harmonic.locked[x * harmonic.m[1] + 0] = 1;
        harmonic.u[x * harmonic.m[1] + (harmonic.m[1] - 1)] = 1.0f;
        harmonic.locked[x * harmonic.m[1] + (harmonic.m[1] - 1)] = 1;
    }

    for (unsigned int y = 0; y < harmonic.m[1]; y++) {
        harmonic.u[0 * harmonic.m[1] + y] = 1.0f;
        harmonic.locked[0 * harmonic.m[1] + y] = 1;
        harmonic.u[(harmonic.m[0] - 1) * harmonic.m[1] + y] = 1.0f;
        harmonic.locked[(harmonic.m[0] - 1) * harmonic.m[1] + y] = 1;
    }
}


void EpicPlanNavCore::uninitialize()
{
    ROS_WARN("Uninitializing...");

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

    ROS_WARN("Done.");
}


bool EpicPlanNavCore::makePlan(const geometry_msgs::PoseStamped &start,
        const geometry_msgs::PoseStamped &goal,
        std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_WARN("Making plan...");

    if (!initialized) {
        ROS_ERROR("Error[EpicPlanNavCore::makePlan]: EpicPathNavCore has not been initialized yet.");
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

    ROS_WARN("Set goal.");

    // Copy everything to the GPU's memory.
    harmonic_initialize_dimension_size_gpu(&harmonic);
    harmonic_initialize_potential_values_gpu(&harmonic);
    harmonic_initialize_locked_gpu(&harmonic);
    harmonic_initialize_gpu(&harmonic, 1024);

    ROS_WARN("Setup GPU.");

    ROS_WARN("STARTING HARMONIC EPIC!");
    harmonic_complete_gpu(&harmonic, 1024);
    ROS_WARN("DONE HARMONIC EPIC!");


    ROS_WARN("Solved!");

    // Free everything on the GPU's memory.
    harmonic_uninitialize_dimension_size_gpu(&harmonic);
    harmonic_uninitialize_potential_values_gpu(&harmonic);
    harmonic_uninitialize_locked_gpu(&harmonic);
    harmonic_uninitialize_gpu(&harmonic);

    ROS_WARN("Free.");

    plan.push_back(start);
       for (int i=0; i<20; i++){
            geometry_msgs::PoseStamped new_goal = goal;
                 tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

                       new_goal.pose.position.x = -2.5+(0.05*i);
                             new_goal.pose.position.y = -3.5+(0.05*i);

                                   new_goal.pose.orientation.x = goal_quat.x();
                                         new_goal.pose.orientation.y = goal_quat.y();
                                               new_goal.pose.orientation.z = goal_quat.z();
                                                     new_goal.pose.orientation.w = goal_quat.w();

                                                        plan.push_back(new_goal);
                                                           }
                                                              plan.push_back(goal);
                                                              ROS_WARN("Done.");
                                                                return true;
}


void EpicPlanNavCore::setGoal(unsigned int xGoal, unsigned int yGoal)
{
    if (!initialized) {
        ROS_ERROR("Error[EpicPlanNavCore::setGoal]: EpicPathNavCore has not been initialized yet.");
        return;
    }

    if (harmonic.m == NULL || harmonic.u == NULL || harmonic.locked == NULL) {
        ROS_ERROR("Error[EpicPlanNavCore::setBoundariesAsObstacles]: Harmonic object is not initialized.");
        return;
    }

    // All current goals become free cells. Boundaries are always obstacles.
    for (unsigned int x = 1; x < harmonic.m[0] - 1; x++) {
        for (unsigned int y = 1; y < harmonic.m[1] - 1; y++) {
            if (harmonic.u[x * harmonic.m[1] + y] == 0.0f) {
                harmonic.u[x * harmonic.m[1] + y] = 1.0f;
                harmonic.locked[x * harmonic.m[1] + y] = 0;
            }
        }
    }

    // The point provided is in cell-coordinates, so assign it!
    harmonic.u[xGoal * harmonic.m[1] + yGoal] = 0.0f;
    harmonic.locked[xGoal * harmonic.m[1] + yGoal] = 1;
}


}; // namespace epic_plan

