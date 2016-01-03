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

#include <epic/epic_navigation_node.h>

#include <harmonic/harmonic_cpu.h>
#include <harmonic/harmonic_gpu.h>
#include <harmonic/harmonic_model_gpu.h>
#include <harmonic/harmonic_path_cpu.h>
#include <harmonic/harmonic_utilities_cpu.h>
#include <harmonic/harmonic_utilities_gpu.h>
#include <error_codes.h>
#include <constants.h>

//using std::string;

namespace epic {

EpicNavigationNode::EpicNavigationNode(ros::NodeHandle &nh) :
        private_node_handle(nh)
{
    init_msgs = false;
    init_alg = false;
    algorithm = EPIC_ALGORITHM_HARMONIC;
    gpu = true;

    width = 0;
    height = 0;
    resolution = 0.0f;
    x_origin = 0.0f;
    y_origin = 0.0f;

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

    num_gpu_threads = 1024;

    goal_added = false;
}


EpicNavigationNode::EpicNavigationNode(ros::NodeHandle &nh, unsigned int alg) :
        private_node_handle(nh)
{
    init_msgs = false;
    init_alg = false;
    algorithm = alg;
    if (alg >= NUM_EPIC_ALGORITHMS) {
        alg = EPIC_ALGORITHM_HARMONIC;
    }
    gpu = true;

    width = 0;
    height = 0;
    resolution = 0.0f;
    x_origin = 0.0f;
    y_origin = 0.0f;

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

    num_gpu_threads = 1024;

    goal_added = false;
}


EpicNavigationNode::~EpicNavigationNode()
{
    uninitAlg();

    init_msgs = false;
    init_alg = false;
}


bool EpicNavigationNode::initMsgs()
{
    if (init_msgs) {
        return false;
    }

    // TODO: Figure out the "remap" problem.
    sub_occupancy_grid = private_node_handle.subscribe("/map", 10, &EpicNavigationNode::subOccupancyGrid, this);
    //sub_occupancy_grid = private_node_handle.subscribe("occupancy_grid", 10, &EpicNavigationNode::subOccupancyGrid, this);
    srv_add_goals = private_node_handle.advertiseService("add_goals", &EpicNavigationNode::srvAddGoals, this);
    srv_remove_goals = private_node_handle.advertiseService("remove_goals", &EpicNavigationNode::srvRemoveGoals, this);
    srv_set_cells = private_node_handle.advertiseService("set_cells", &EpicNavigationNode::srvSetCells, this);
    srv_compute_path = private_node_handle.advertiseService("compute_path", &EpicNavigationNode::srvComputePath, this);

    // Exclusively for simplified interaction with rviz.
    // TODO: Figure out the "remap" problem.
    sub_map_pose_estimate = private_node_handle.subscribe("/initialpose", 10, &EpicNavigationNode::subMapPoseEstimate, this);
    sub_map_nav_goal = private_node_handle.subscribe("/move_base_simple/goal", 10, &EpicNavigationNode::subMapNavGoal, this);
    pub_map_path = private_node_handle.advertise<nav_msgs::Path>("path", 1);

    init_msgs = true;

    return true;
}


void EpicNavigationNode::update(unsigned int num_steps)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::update]: Algorithm has not been initialized yet.");
        return;
    }

    // Depending on the algorithm, perform different update actions.
    if (algorithm == EPIC_ALGORITHM_HARMONIC) {
        // Perform steps of the harmonic function. Do not check for convergence.
        if (gpu) {
            int result = harmonic_update_and_check_gpu(&harmonic, num_gpu_threads);
            if (result == EPIC_SUCCESS) {
                for (unsigned int i = 0; i < num_steps - 1; i++) {
                    if (harmonic_update_gpu(&harmonic, num_gpu_threads) != EPIC_SUCCESS) {
                        ROS_ERROR("Error[EpicNavigationNode::update]: Failed to use GPU to relax harmonic function with 'epic' library.");
                        return;
                    }
                }
            } else if (result != EPIC_SUCCESS_AND_CONVERGED) {
                ROS_ERROR("Error[EpicNavigationNode::update]: Failed to use GPU to relax harmonic function (and check convergence) with 'epic' library.");
                return;
            }
        } else {
            int result = harmonic_update_and_check_cpu(&harmonic);
            if (result == EPIC_SUCCESS) {
                for (unsigned int i = 0; i < num_steps - 1; i++) {
                    if (harmonic_update_cpu(&harmonic) != EPIC_SUCCESS) {
                        ROS_ERROR("Error[EpicNavigationNode::update]: Failed to use CPU to relax harmonic function with 'epic' library.");
                        return;
                    }
                }
            } else if (result != EPIC_SUCCESS_AND_CONVERGED) {
                ROS_ERROR("Error[EpicNavigationNode::update]: Failed to use CPU to relax harmonic function (and check convergence) with 'epic' library.");
                return;
            }
        }
    } else {
    }
}


void EpicNavigationNode::initAlg(unsigned int w, unsigned int h)
{
    if (harmonic.n > 0 || harmonic.m != nullptr || harmonic.u != nullptr || harmonic.locked != nullptr) {
        ROS_ERROR("Error[EpicNavigationNode::initAlg]: Harmonic object is already initialized and must be freed first.");
        return;
    }

    width = w;
    height = h;

    init_alg = true;

    if (algorithm == EPIC_ALGORITHM_HARMONIC) {
        harmonic.n = 2;
        harmonic.m = new unsigned int[2];
        harmonic.m[0] = h;
        harmonic.m[1] = w;

        harmonic.u = new float[w * h];
        harmonic.locked = new unsigned int[w * h];
        for (unsigned int i = 0; i < w * h; i++) {
            harmonic.u[i] = 0.0f;
            harmonic.locked[i] = 0;
        }

        setBoundariesAsObstacles();

        int result = harmonic_initialize_dimension_size_gpu(&harmonic);
        result += harmonic_initialize_potential_values_gpu(&harmonic);
        result += harmonic_initialize_locked_gpu(&harmonic);
        result += harmonic_initialize_gpu(&harmonic, num_gpu_threads);

        if (result != EPIC_SUCCESS) {
            ROS_WARN("Warning[EpicNavigationNode::createHarmonic]: Could not initialize GPU. Defaulting to CPU version.");
            gpu = false;
        } else {
            gpu = true;
        }
    }
}


void EpicNavigationNode::uninitAlg()
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::uninitAlg]: Algorithm has not been initialized yet.");
        return;
    }

    if (algorithm == EPIC_ALGORITHM_HARMONIC) {
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

        if (gpu) {
            int result = harmonic_uninitialize_dimension_size_gpu(&harmonic);
            result += harmonic_uninitialize_potential_values_gpu(&harmonic);
            result += harmonic_uninitialize_locked_gpu(&harmonic);
            result += harmonic_uninitialize_gpu(&harmonic);
            if (result != EPIC_SUCCESS) {
                ROS_WARN("Warning[EpicNavigationNode::uninitAlg]: Failed to uninitialize GPU variables.");
            }
        }
    } else {
    }

    init_alg = false;
}


void EpicNavigationNode::setBoundariesAsObstacles()
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::setBoundariesAsObstacles]: Algorithm has not been initialized yet.");
        return;
    }

    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr) {
        ROS_ERROR("Error[EpicNavigationNode::setBoundariesAsObstacles]: Harmonic object is not initialized.");
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


void EpicNavigationNode::mapToWorld(float mx, float my, float &wx, float &wy) const
{
    wx = x_origin + mx * resolution;
    wy = y_origin + my * resolution;
}


bool EpicNavigationNode::worldToMap(float wx, float wy, float &mx, float &my) const
{
    if (wx < x_origin || wy < y_origin ||
            wx >= x_origin + width * resolution ||
            wy >= y_origin + height * resolution) {
        ROS_WARN("Error[EpicNavigationNode::worldToMap]: World coordinates are outside map.");
        return false;
    }

    mx = (wx - x_origin) / resolution;
    my = (wy - y_origin) / resolution;

    return true;
}


bool EpicNavigationNode::isCellGoal(unsigned int x, unsigned int y)
{
    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr ||
            x >= harmonic.m[1] || y >= harmonic.m[0]) {
        ROS_WARN("Warning[EpicNavigationNode::isCellGoal]: Location provided is outside of the map; it is obviously not a goal.");
        return false;
    }

    return (harmonic.u[y * harmonic.m[1] + x] == EPIC_LOG_SPACE_GOAL && harmonic.locked[y * harmonic.m[1] + x] == 1);
}


void EpicNavigationNode::subOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // If the size changed, then free everything and start over.
    // Note we don't care if the resolution or offsets change.
    if (msg->info.width != width || msg->info.height != height) {
        uninitAlg();
        initAlg(msg->info.width, msg->info.height);
    }

    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::subOccupancyGrid]: Algorithm was not initialized.");
        return;
    }

    // These only affect the final path, not the path computation.
    x_origin = msg->info.origin.position.x;
    y_origin = msg->info.origin.position.y;
    resolution = msg->info.resolution;

    // Copy the data to the respective algorithm.
    if (algorithm == EPIC_ALGORITHM_HARMONIC) {
        std::vector<unsigned int> v;
        std::vector<unsigned int> types;

        for (unsigned int y = 1; y < height - 1; y++) {
            for (unsigned int x = 1; x < width - 1; x++) {
                if (msg->data[y * width + x] == EPIC_OCCUPANCY_GRID_NO_CHANGE || isCellGoal(x, y)) {
                    // Do nothing.
                } else if (msg->data[y * width + x] >= EPIC_OCCUPANCY_GRID_OBSTACLE_THRESHOLD) {
                    v.push_back(x);
                    v.push_back(y);
                    types.push_back(EPIC_CELL_TYPE_OBSTACLE);
                } else {
                    v.push_back(x);
                    v.push_back(y);
                    types.push_back(EPIC_CELL_TYPE_FREE);
                }
            }
        }

        // Note: This trick with vectors only works for C++11 or greater; the spec updated to guarantee
        // that vector objects store contiguously in memory.
        int result = 0;
        if (gpu) {
            result = harmonic_utilities_set_cells_2d_gpu(&harmonic, num_gpu_threads, types.size(), &v[0], &types[0]);
            if (result != EPIC_SUCCESS) {
                ROS_ERROR("Error[EpicNavigationNode::subOccupancyGrid]: Failed to set the cells on the GPU.");
            }
        } else {
            result = harmonic_utilities_set_cells_2d_cpu(&harmonic, types.size(), &v[0], &types[0]);
            if (result != EPIC_SUCCESS) {
                ROS_ERROR("Error[EpicNavigationNode::subOccupancyGrid]: Failed to set the cells on the CPU.");
            }
        }

        v.clear();
        types.clear();
    } else {
    }
}


bool EpicNavigationNode::srvAddGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::srvAddGoals]: Algorithm was not initialized.");
        return false;
    }

    std::vector<unsigned int> v;
    std::vector<unsigned int> types;

    for (unsigned int i = 0; i < req.goals.size(); i++) {
        float x = 0.0f;
        float y = 0.0f;

        worldToMap(req.goals[i].pose.position.x, req.goals[i].pose.position.y, x, y);

        // If the goal location is an obstacle, then do not let it add a goal here.
        if (isCellGoal((unsigned int)(x + 0.5f), (unsigned int)(y + 0.5f))) {
            continue;
        }

        v.push_back((unsigned int) x);
        v.push_back((unsigned int) y);
        types.push_back(EPIC_CELL_TYPE_GOAL);
    }

    if (v.size() == 0) {
        ROS_WARN("Warning[EpicNavigationNode::srvAddGoals]: Attempted to add goal(s) inside obstacles. No goals added.");
        res.success = false;
        return false;
    }

    // Note: This trick with vectors only works for C++11 or greater; the spec updated to guarantee
    // that vector objects store contiguously in memory.
    if (gpu) {
        if (harmonic_utilities_set_cells_2d_gpu(&harmonic, num_gpu_threads, types.size(), &v[0], &types[0]) != EPIC_SUCCESS) {
            ROS_ERROR("Error[EpicNavigationNode::srvAddGoals]: Failed to set the cells on the GPU.");
            res.success = false;
            return false;
        }
    } else {
        if (harmonic_utilities_set_cells_2d_cpu(&harmonic, types.size(), &v[0], &types[0]) != EPIC_SUCCESS) {
            ROS_ERROR("Error[EpicNavigationNode::srvAddGoals]: Failed to set the cells on the CPU.");
            res.success = false;
            return false;
        }
    }

    v.clear();
    types.clear();

    res.success = true;

    return true;
}


bool EpicNavigationNode::srvRemoveGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::srvRemoveGoals]: Algorithm was not initialized.");
        return false;
    }

    std::vector<unsigned int> v;
    std::vector<unsigned int> types;

    // Note: Removing goals turns them into free space. Recall, however, that goals can
    // only be added on free space (above).
    for (unsigned int i = 0; i < req.goals.size(); i++) {
        float x = 0.0f;
        float y = 0.0f;

        worldToMap(req.goals[i].pose.position.x, req.goals[i].pose.position.y, x, y);

        v.push_back((unsigned int) x);
        v.push_back((unsigned int) y);
        types.push_back(EPIC_CELL_TYPE_FREE);
    }

    // Note: This trick with vectors only works for C++11 or greater; the spec updated to guarantee
    // that vector objects store contiguously in memory.
    if (gpu) {
        if (harmonic_utilities_set_cells_2d_gpu(&harmonic, num_gpu_threads, types.size(), &v[0], &types[0]) != EPIC_SUCCESS) {
            ROS_ERROR("Error[EpicNavigationNode::srvRemoveGoals]: Failed to set the cells on the GPU.");
            res.success = false;
            return false;
        }
    } else {
        if (harmonic_utilities_set_cells_2d_cpu(&harmonic, types.size(), &v[0], &types[0]) != EPIC_SUCCESS) {
            ROS_ERROR("Error[EpicNavigationNode::srvRemoveGoals]: Failed to set the cells on the CPU.");
            res.success = false;
            return false;
        }
    }

    v.clear();
    types.clear();

    res.success = true;

    return true;
}


bool EpicNavigationNode::srvSetCells(epic::SetCells::Request &req, epic::SetCells::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::srvSetCells]: Algorithm was not initialized.");
        return false;
    }

    std::vector<unsigned int> v;
    std::vector<unsigned int> types;

    for (unsigned int i = 0; i < req.v.size(); i++) {
        v.push_back((unsigned int)req.v[i]);
    }
    for (unsigned int i = 0; i < req.types.size(); i++) {
        types.push_back((unsigned int)req.types[i]);
    }

    // Note: This trick with vectors only works for C++11 or greater; the spec updated to guarantee
    // that vector objects store contiguously in memory.
    if (gpu) {
        if (harmonic_utilities_set_cells_2d_gpu(&harmonic, num_gpu_threads, types.size(), &v[0], &types[0]) != EPIC_SUCCESS) {
            ROS_ERROR("Error[EpicNavigationNode::srvSetCells]: Failed to set the cells on the GPU.");
            res.success = false;
            return false;
        }
    } else {
        if (harmonic_utilities_set_cells_2d_cpu(&harmonic, types.size(), &v[0], &types[0]) != EPIC_SUCCESS) {
            ROS_ERROR("Error[EpicNavigationNode::srvSetCells]: Failed to set the cells on the CPU.");
            res.success = false;
            return false;
        }
    }

    v.clear();
    types.clear();

    res.success = true;

    return true;
}


bool EpicNavigationNode::srvComputePath(epic::ComputePath::Request &req, epic::ComputePath::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNode::srvComputePath]: Algorithm was not initialized.");
        return false;
    }

    if (gpu) {
        if (harmonic_get_potential_values_gpu(&harmonic) != EPIC_SUCCESS) {
            ROS_WARN("Error[EpicNavigationNode::srvComputePath]: Failed to get the potential values from the GPU.");
            return false;
        }
    }

    float x = 0.0f;
    float y = 0.0f;
    if (!worldToMap(req.start.pose.position.x, req.start.pose.position.y, x, y)) {
        ROS_WARN("Warning[EpicNavigationNode::srvComputePath]: Could not convert start to floating point map location.");
    }

    unsigned int k = 0;
    float *raw_path = nullptr;

    int result = harmonic_compute_path_2d_cpu(&harmonic, x, y, req.step_size, req.precision, req.max_length, k, raw_path);

    if (result != EPIC_SUCCESS) {
        ROS_ERROR("Error[EpicNavigationNode::srvComputePath]: Failed to compute the path.");

        if (raw_path != nullptr) {
            delete [] raw_path;
        }

        return false;
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


void EpicNavigationNode::subMapPoseEstimate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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


void EpicNavigationNode::subMapNavGoal(const geometry_msgs::PoseStamped::ConstPtr &msg)
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


