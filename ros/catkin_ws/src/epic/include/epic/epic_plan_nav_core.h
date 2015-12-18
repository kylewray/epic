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

//#include <base_local_planner/world_model.h>
//#include <base_local_planner/costmap_model.h>


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
        EpicPlanNavCore(std::string name, costmap_2d::Costmap2DROS *costmap);

        /**
         *  Initialize the plugin class EpicPlanNavCore. Overloads from nav_core::BaseGlobalPlanner.
         *  @param  name    The name of the planner.
         *  @param  costmap The cost map to solve.
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap);

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

    };
};


#endif // EPIC_PLAN_NAV_CORE_H

