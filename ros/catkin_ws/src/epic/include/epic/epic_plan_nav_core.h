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


#ifndef EPIC_PLAN_NAV_CORE_H
#define EPIC_PLAN_NAV_CORE_H


#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <angles/angles.h>

#include <harmonic/harmonic.h>


using std::string;


namespace epic_plan {

    class EpicPlanNavCore : public nav_core::BaseGlobalPlanner {
    public:
        /**
         *  The default constructor for the EpicPlanNavCore.
         */
        EpicPlanNavCore();

        /**
         *  The constructor for the EpicPlanNavCore used by the plugin.
         *  @param  name    The name of the planner.
         *  @param  costmap The cost map to solve.
         */
        EpicPlanNavCore(std::string name, costmap_2d::Costmap2DROS *costmapROS);


        /**
         *  The deconstructor for the EpicPlanNavCore used by the plugin.
         */
        virtual ~EpicPlanNavCore();

        /**
         *  Initialize the plugin class EpicPlanNavCore. Overloads from nav_core::BaseGlobalPlanner.
         *  @param  name    The name of the planner.
         *  @param  costmap The cost map to solve.
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmapROS);

        /**
         *  Uninitialize the plugin class EpicPlanNavCore.
         */
        void uninitialize();

        /**
         *  Make the plan for plugin EpicPlanNavCore. Overloads from nav_core::BaseGlobalPlanner.
         *  @param  start   The starting (x, y) location in the cost map.
         *  @param  goal    The goal (x, y) location in the cost map.
         *  @param  plan    The resulting plan as a sequence of (x, y) waypoints.
         *  @return True if a plan was found, false otherwise.
         */
        bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         *  Set the goal given an (x, y) location on the cell map.
         *  @param  xGoal   The x location.
         *  @param  yGoal   The y location.
         */
        void setGoal(unsigned int xGoal, unsigned int yGoal);

    private:
        /**
         *  Set harmonic cell potential values from the cost map, assuming both have been defined.
         */
        void setCellsFromCostmap();

        /**
         *  Set the boundaries as obstacles, provided the harmonic function has been defined.
         */
        void setBoundariesAsObstacles();

        /**
         *  Compute the cell indexes given coordinates in float-
         *  @param  x           The x coordinate in float-index form (completely adjusted).
         *  @param  y           The y coordinate in float-index form (completely adjusted).
         *  @param  xCellIndex  The resultant x cell index. This will be modified.
         *  @param  yCellIndex  The resultant y cell index. This will be modified.
         */
        void computeCellIndex(float x, float y,
                unsigned int &xCellIndex, unsigned int &yCellIndex);

        /**
         *  Convert the float-index map coordinate to the world coordinate.
         *  @param  mx      The float-index map x coordinate.
         *  @param  my      The float-index map y coordinate.
         *  @param  wx      The resultant world x coordinate. This will be modified.
         *  @param  wy      The resultant world y coordinate. This will be modified.
         */
        void mapToWorld(float mx, float my, float &wx, float &wy) const;

        /**
         *  Covert the world coordinate to the float-index map coordinate.
         *  @param  wx      The world x coordinate.
         *  @param  wy      The world y coordinate.
         *  @param  mx      The resultant float-index map x coordinate. This will be modified.
         *  @param  my      The resultant float-index map y coordinate. This will be modified.
         *  @return True if the world coordinate is outside the map, false otherwise.
         */
        bool worldToMap(float wx, float wy, float &mx, float &my) const;

        /**
         *  Compute the potential at the float-index map coordinate provided.
         *  @param  x       The float-index map x coordinate.
         *  @param  y       The float-index map y coordinate.
         *  @return The potential at this float-index map coordinate.
         */
        float computePotential(float x, float y);

        /**
         *  Publish the plan as a path for use in path followers and visualization.
         *  @param  path    The path to be published.
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);

        // The Harmonic struct which holds the potential values for the 'epic' library.
        Harmonic harmonic;

        // The costmap object.
        costmap_2d::Costmap2D *costmap;

        // We will publish the plan so that the stupid nav_core doesn't truncate it.
        ros::Publisher pubPlan;

        // Also publish the potential.
        ros::Publisher pubPotential;

        // If this object has been properly initialized or not.
        bool initialized;
    };
};


#endif // EPIC_PLAN_NAV_CORE_H

