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


EpicPlanNavCore::EpicPlanNavCore() //: costmap_ros_(NULL), initialized_(false)
{ }


EpicPlanNavCore::EpicPlanNavCore(std::string name,
    costmap_2d::Costmap2DROS *costmap)
    //: costmap_ros_(NULL), initialized_(false)
{
    initialize(name, costmap);
}


void EpicPlanNavCore::initialize(std::string name, costmap_2d::Costmap2DROS *costmap)
{ }

bool EpicPlanNavCore::makePlan(const geometry_msgs::PoseStamped &start,
        const geometry_msgs::PoseStamped &goal,
        std::vector<geometry_msgs::PoseStamped> &plan)
{

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
                                                                return true;
}


}; // namespace epic_plan

