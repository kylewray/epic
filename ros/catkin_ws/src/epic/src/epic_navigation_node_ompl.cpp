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

#include <epic/epic_navigation_node_ompl.h>
#include <epic/epic_navigation_node_constants.h>

namespace epic {

EpicNavigationNodeOMPL::EpicNavigationNodeOMPL(ros::NodeHandle &nh, unsigned int alg) :
        private_node_handle(nh)
{
    init_msgs = false;
    init_alg = false;
    algorithm = alg;
    if (alg >= NUM_EPIC_ALGORITHMS) {
        algorithm = EPIC_ALGORITHM_RRT_CONNECT;
    }

    width = 0;
    height = 0;
    occupancy_grid = nullptr;
    resolution = 0.0f;
    x_origin = 0.0f;
    y_origin = 0.0f;

    goal_added = false;

    start_assigned = false;
    goal_assigned = false;
}


EpicNavigationNodeOMPL::~EpicNavigationNodeOMPL()
{
    uninitAlg();

    init_msgs = false;
    init_alg = false;

    if (occupancy_grid != nullptr) {
        delete [] occupancy_grid;
    }
    occupancy_grid = nullptr;
}


bool EpicNavigationNodeOMPL::initMsgs()
{
    if (init_msgs) {
        return false;
    }

    // TODO: Figure out the "remap" problem.
    sub_occupancy_grid = private_node_handle.subscribe("/map", 10, &EpicNavigationNodeOMPL::subOccupancyGrid, this);
    //sub_occupancy_grid = private_node_handle.subscribe("occupancy_grid", 10, &EpicNavigationNodeOMPL::subOccupancyGrid, this);
    srv_add_goals = private_node_handle.advertiseService("add_goals", &EpicNavigationNodeOMPL::srvAddGoals, this);
    srv_remove_goals = private_node_handle.advertiseService("remove_goals", &EpicNavigationNodeOMPL::srvRemoveGoals, this);
    srv_set_cells = private_node_handle.advertiseService("set_cells", &EpicNavigationNodeOMPL::srvSetCells, this);
    srv_compute_path = private_node_handle.advertiseService("compute_path", &EpicNavigationNodeOMPL::srvComputePath, this);

    // Exclusively for simplified interaction with rviz.
    // TODO: Figure out the "remap" problem.
    sub_map_pose_estimate = private_node_handle.subscribe("/initialpose", 10, &EpicNavigationNodeOMPL::subMapPoseEstimate, this);
    sub_map_nav_goal = private_node_handle.subscribe("/move_base_simple/goal", 10, &EpicNavigationNodeOMPL::subMapNavGoal, this);
    pub_map_path = private_node_handle.advertise<nav_msgs::Path>("path", 1);

    init_msgs = true;

    return true;
}


void EpicNavigationNodeOMPL::update(float t)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::update]: Algorithm has not been initialized yet.");
        return;
    }

    // Depending on the algorithm, perform different update actions.
    ompl_planner_status = ompl_planner->solve(t);
}


ompl::base::OptimizationObjectivePtr omplGetPathLengthObjective(const ompl::base::SpaceInformationPtr &ompl_space_info)
{
    return ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(ompl_space_info));
}


void EpicNavigationNodeOMPL::initAlg()
{
    // Only initialize the algorithm if all necessary variables have been defined.
    if (!goal_assigned || !start_assigned || occupancy_grid == nullptr) {
        return;
    }

    // Setup the OMPL state space.
    ompl::base::RealVectorBounds ompl_bounds(2);
    ompl_bounds.setLow(0, 0.0);
    ompl_bounds.setHigh(0, height);
    ompl_bounds.setLow(1, 0.0);
    ompl_bounds.setHigh(1, width);

    ompl::base::StateSpacePtr ompl_space(new ompl::base::RealVectorStateSpace(2));
    ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds);

    // The state validity checker holds the map representation so we can check if a cell is valid or not.
    ompl::base::SpaceInformationPtr ompl_space_info(new ompl::base::SpaceInformation(ompl_space));
    ompl_space_info->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
                            new EpicNavigationNodeOMPLStateValidityChecker(ompl_space_info, occupancy_grid)));
    ompl_space_info->setup();

    // We relinquish control of the occupancy_grid. It will be managed by the EpicNavigatioNodeOMPLStateValidityChecker object.
    occupancy_grid = nullptr;

    // Create the problem definition.
    ompl::base::ScopedState<> start_state(ompl_space);
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start_location.second;
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = start_location.first;

    ompl::base::ScopedState<> goal_state(ompl_space);
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = goal_location.second;
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = goal_location.first;

    ompl::base::ProblemDefinitionPtr ompl_problem_def(new ompl::base::ProblemDefinition(ompl_space_info));
    ompl_problem_def->setOptimizationObjective(omplGetPathLengthObjective(ompl_space_info));

    if (algorithm == EPIC_ALGORITHM_RRT_CONNECT) {
        ompl_planner = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(ompl_space_info));
    } else {
    }

    init_alg = true;
}


void EpicNavigationNodeOMPL::uninitAlg()
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::uninitAlg]: Algorithm has not been initialized yet.");
        return;
    }

    init_alg = false;
}


void EpicNavigationNodeOMPL::setBoundariesAsObstacles()
{
    if (occupancy_grid == nullptr) {
        ROS_ERROR("Error[EpicNavigationNodeOMPL::setBoundariesAsObstacles]: Occupancy grid object is not initialized.");
        return;
    }

    for (unsigned int y = 0; y < height; y++) {
        occupancy_grid[y * width + 0] = EPIC_CELL_TYPE_OBSTACLE;
        occupancy_grid[y * width + (width - 1)] = EPIC_CELL_TYPE_OBSTACLE;
    }

    for (unsigned int x = 0; x < width; x++) {
        occupancy_grid[0 * width + x] = EPIC_CELL_TYPE_OBSTACLE;
        occupancy_grid[(height - 1) * width + x] = EPIC_CELL_TYPE_OBSTACLE;
    }
}


void EpicNavigationNodeOMPL::mapToWorld(float mx, float my, float &wx, float &wy) const
{
    wx = x_origin + mx * resolution;
    wy = y_origin + my * resolution;
}


bool EpicNavigationNodeOMPL::worldToMap(float wx, float wy, float &mx, float &my) const
{
    if (wx < x_origin || wy < y_origin ||
            wx >= x_origin + width * resolution ||
            wy >= y_origin + height * resolution) {
        ROS_WARN("Error[EpicNavigationNodeOMPL::worldToMap]: World coordinates are outside map.");
        return false;
    }

    mx = (wx - x_origin) / resolution;
    my = (wy - y_origin) / resolution;

    return true;
}


bool EpicNavigationNodeOMPL::isCellObstacle(unsigned int x, unsigned int y)
{
    if (occupancy_grid == nullptr || x >= width || y >= height) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::isCellGoal]: Location provided is outside of the map; it is obviously an obstacle.");
        return true;
    }

    return (occupancy_grid[y * width + x] == EPIC_CELL_TYPE_OBSTACLE);
}


bool EpicNavigationNodeOMPL::isCellGoal(unsigned int x, unsigned int y)
{
    if (occupancy_grid == nullptr || x >= width || y >= height) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::isCellGoal]: Location provided is outside of the map; it is obviously not a goal.");
        return false;
    }

    return (occupancy_grid[y * width + x] == EPIC_CELL_TYPE_GOAL);
}


void EpicNavigationNodeOMPL::subOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // These only affect the final path, not the path computation.
    width = msg->info.width;
    height = msg->info.height;

    x_origin = msg->info.origin.position.x;
    y_origin = msg->info.origin.position.y;

    resolution = msg->info.resolution;

    // We need to reset everything for OMPL if the map changes.
    init_alg = false;

    if (occupancy_grid != nullptr) {
        delete [] occupancy_grid;
    }

    occupancy_grid = new int[width * height];

    // Copy the map data to this state space representation. This map will be passed into the state validity checker object, and freed in there.
    for (unsigned int y = 1; y < height - 1; y++) {
        for (unsigned int x = 1; x < width - 1; x++) {
            if (msg->data[y * width + x] == EPIC_OCCUPANCY_GRID_NO_CHANGE || isCellGoal(x, y)) {
                // Do nothing.
            } else if (msg->data[y * width + x] >= EPIC_OCCUPANCY_GRID_OBSTACLE_THRESHOLD) {
                occupancy_grid[y * width + x] = EPIC_CELL_TYPE_OBSTACLE;
            } else {
                occupancy_grid[y * width + x] = EPIC_CELL_TYPE_FREE;
            }
        }
    }

    setBoundariesAsObstacles();

    // TODO: Create the EpicNavigationNodeOMPLStateValidityChecker class in the header.

    // Note: Do *not* try to create the planner algorithm now. The process should be:
    // 1. Service call to create map.
    // 2. Service call(s) to assign goals, or change the map. Check if algorithm valid. If so, begin updating.
    // 3. Service call (after a few) to get the current path.
    //initAlg();
}


bool EpicNavigationNodeOMPL::srvAddGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{
    if (occupancy_grid == nullptr) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::srvAddGoals]: Occupancy grid was not initialized.");
        res.success = false;
        return false;
    }

    if (req.goals.size() != 1) {
        ROS_ERROR("Warning[EpicNavigationNodeOMPL::srvAddGoals]: Invalid number of goal locations.");
        res.success = false;
        return false;
    }

    float x = 0.0f;
    float y = 0.0f;

    worldToMap(req.goals[0].pose.position.x, req.goals[0].pose.position.y, x, y);

    // If the goal location is an obstacle, then do not let it add a goal here.
    if (isCellObstacle((unsigned int)(x + 0.5f), (unsigned int)(y + 0.5f))) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::srvAddGoals]: Attempted to add a goal at an obstacle.");
        res.success = false;
        return false;
    }

    goal_location.first = x;
    goal_location.second = y;
    goal_assigned = true;

    occupancy_grid[(unsigned int)(y + 0.5f) * width + (unsigned int)(x + 0.5f)] = EPIC_CELL_TYPE_OBSTACLE;

    // Try to create the planner algorithm now.
    initAlg();

    res.success = true;

    return true;
}


bool EpicNavigationNodeOMPL::srvRemoveGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{
    if (occupancy_grid == nullptr) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::srvRemoveGoals]: Occupancy grid was not initialized.");
        res.success = false;
        return false;
    }

    if (req.goals.size() != 1) {
        ROS_ERROR("Warning[EpicNavigationNodeOMPL::srvRemoveGoals]: Invalid number of goal locations.");
        res.success = false;
        return false;
    }

    float x = 0.0f;
    float y = 0.0f;

    worldToMap(req.goals[0].pose.position.x, req.goals[0].pose.position.y, x, y);

    // If the goal location is not a goal, then we can just return without changing anything.
    if (!isCellGoal((unsigned int)(x + 0.5f), (unsigned int)(y + 0.5f))) {
        return ;
    }

    occupancy_grid[(unsigned int)(y + 0.5f) * width + (unsigned int)(x + 0.5f)] = EPIC_CELL_TYPE_FREE;
    goal_assigned = false;

    // Try to create the planner algorithm now.
    initAlg();

    res.success = true;

    return true;
}


bool EpicNavigationNodeOMPL::srvSetCells(epic::SetCells::Request &req, epic::SetCells::Response &res)
{
    if (occupancy_grid == nullptr) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::srvSetCells]: Occupancy grid was not initialized.");
        res.success = false;
        return false;
    }

    for (unsigned int i = 0; i < req.types.size(); i++) {
        // Note: For the SetCells service, these are assigning the raw cells, not picking world coordinates.
        // Thus, no worldToMap translation is required.
        unsigned int x = req.v[2 * i + 0];
        unsigned int y = req.v[2 * i + 1];

        if (x >= width || y >= height) {
            continue;
        }

        if (req.types[i] == EPIC_CELL_TYPE_GOAL || req.types[i] == EPIC_CELL_TYPE_OBSTACLE || req.types[i] == EPIC_CELL_TYPE_FREE) {
            occupancy_grid[(unsigned int)(y + 0.5f) * width + (unsigned int)(x + 0.5f)] = req.types[i];
        }

        if (req.types[i] == EPIC_CELL_TYPE_GOAL) {
            goal_location.first = x;
            goal_location.second = y;
            goal_assigned = true;
        }
    }

    // Try to create the planner algorithm now.
    initAlg();

    res.success = true;

    return true;
}


bool EpicNavigationNodeOMPL::srvComputePath(epic::ComputePath::Request &req, epic::ComputePath::Response &res)
{
    // First, determine and assign the starting location.
    float x = 0.0f;
    float y = 0.0f;

    if (!worldToMap(req.start.pose.position.x, req.start.pose.position.y, x, y)) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::srvComputePath]: Could not convert start to floating point map location.");
    }

    start_location.first = x;
    start_location.second = y;
    start_assigned = true;

    // Try to create the planner algorithm now that we have an initial starting state.
    initAlg();

    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeOMPL::srvComputePath]: Algorithm was not initialized.");
        return false;
    }

    unsigned int k = 0;
    float *raw_path = nullptr;

    if ((bool)ompl_planner_status) {
        // TODO: If an approximate or exact solution was found, then populate raw_path. Otherwise, leave it empty.
        //k = ???
        //raw_path = new float[2 * k];
        //for (...
        // ompl_planner.get_path...???
    }

    res.path.header.frame_id = req.start.header.frame_id;
    res.path.header.stamp = req.start.header.stamp;

    res.path.poses.resize(k);
    res.path.poses[0] = req.start;

    for (unsigned int i = 1; i < k; i++) {
        x = raw_path[2 * i + 0];
        y = raw_path[2 * i + 1];
        float path_theta = std::atan2(y - raw_path[2 * (i - 1) + 1], x - raw_path[2 * (i - 1) + 0]);

        float path_x = 0.0f;
        float path_y = 0.0f;
        mapToWorld(x, y, path_x, path_y);

        res.path.poses[i] = req.start;
        res.path.poses[i].pose.position.x = path_x;
        res.path.poses[i].pose.position.y = path_y;
        res.path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_theta);
    }

    delete [] raw_path;
    raw_path = nullptr;

    return true;
}


void EpicNavigationNodeOMPL::subMapPoseEstimate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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

    srvComputePath(req, res);

    pub_map_path.publish(res.path);
}


void EpicNavigationNodeOMPL::subMapNavGoal(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    epic::ModifyGoals::Request req;
    epic::ModifyGoals::Response res;

    if (goal_added) {
        req.goals.push_back(last_goal);
        srvRemoveGoals(req, res);
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
    srvAddGoals(req, res);

    goal_added = true;
}

}; // namespace epic


