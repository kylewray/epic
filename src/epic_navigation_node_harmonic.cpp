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

#include <epic/epic_navigation_node_harmonic.h>
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

EpicNavigationNodeHarmonic::EpicNavigationNodeHarmonic(ros::NodeHandle &nh) :
        EpicNavigationNode(nh)
        //private_node_handle(nh)
{
    init_msgs = false;
    init_alg = false;
    paused = false;
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
}


EpicNavigationNodeHarmonic::~EpicNavigationNodeHarmonic()
{
    uninitAlg();

    init_msgs = false;
    init_alg = false;
}


bool EpicNavigationNodeHarmonic::initialize()
{
    if (init_msgs) {
        return false;
    }

    std::string sub_occupancy_grid_topic;
    private_node_handle.param<std::string>("sub_occupancy_grid",
                                            sub_occupancy_grid_topic,
                                            "/map");
    sub_occupancy_grid = private_node_handle.subscribe(sub_occupancy_grid_topic,
                                                        10,
                                                        &EpicNavigationNodeHarmonic::subOccupancyGrid,
                                                        this);

    std::string srv_set_status_topic;
    private_node_handle.param<std::string>("srv_set_status",
                                            srv_set_status_topic,
                                            "set_status");
    srv_set_status = private_node_handle.advertiseService(srv_set_status_topic,
                                                          &EpicNavigationNodeHarmonic::srvSetStatus,
                                                          this);

    std::string srv_add_goals_topic;
    private_node_handle.param<std::string>("srv_add_goals",
                                            srv_add_goals_topic,
                                            "add_goals");
    srv_add_goals = private_node_handle.advertiseService(srv_add_goals_topic,
                                                        &EpicNavigationNodeHarmonic::srvAddGoals,
                                                        this);

    std::string srv_remove_goals_topic;
    private_node_handle.param<std::string>("srv_remove_goals",
                                            srv_remove_goals_topic,
                                            "remove_goals");
    srv_remove_goals = private_node_handle.advertiseService(srv_remove_goals_topic,
                                                            &EpicNavigationNodeHarmonic::srvRemoveGoals,
                                                            this);

    std::string srv_get_cell_topic;
    private_node_handle.param<std::string>("srv_get_cell",
                                            srv_get_cell_topic,
                                            "get_cell");
    srv_get_cell = private_node_handle.advertiseService(srv_get_cell_topic,
                                                        &EpicNavigationNodeHarmonic::srvGetCell,
                                                        this);

    std::string srv_set_cells_topic;
    private_node_handle.param<std::string>("srv_set_cells",
                                            srv_set_cells_topic,
                                            "set_cells");
    srv_set_cells = private_node_handle.advertiseService(srv_set_cells_topic,
                                                        &EpicNavigationNodeHarmonic::srvSetCells,
                                                        this);

    std::string srv_reset_free_cells_topic;
    private_node_handle.param<std::string>("srv_reset_free_cells",
                                            srv_reset_free_cells_topic,
                                            "reset_free_cells");
    srv_reset_free_cells = private_node_handle.advertiseService(srv_reset_free_cells_topic,
                                                                &EpicNavigationNodeHarmonic::srvResetFreeCells,
                                                                this);

    std::string srv_compute_path_topic;
    private_node_handle.param<std::string>("srv_compute_path",
                                            srv_compute_path_topic,
                                            "compute_path");
    srv_compute_path = private_node_handle.advertiseService(srv_compute_path_topic,
                                                            &EpicNavigationNodeHarmonic::srvComputePath,
                                                            this);

    init_msgs = true;

    return true;
}


void EpicNavigationNodeHarmonic::update(unsigned int num_steps)
{
    if (!init_alg) {
        //ROS_WARN("Warning[EpicNavigationNodeHarmonic::update]: Algorithm has not been initialized yet.");
        return;
    }

    if (paused) {
        return;
    }

    // Perform steps of the harmonic function. Do not check for convergence.
    if (gpu) {
        int result = harmonic_update_and_check_gpu(&harmonic, num_gpu_threads);
        if (result == EPIC_SUCCESS) {
            for (unsigned int i = 0; i < num_steps - 1; i++) {
                if (harmonic_update_gpu(&harmonic, num_gpu_threads) != EPIC_SUCCESS) {
                    ROS_ERROR("Error[EpicNavigationNodeHarmonic::update]: Failed to use GPU to relax harmonic function with 'epic' library.");
                    return;
                }
            }
        } else if (result != EPIC_SUCCESS_AND_CONVERGED) {
            ROS_ERROR("Error[EpicNavigationNodeHarmonic::update]: Failed to use GPU to relax harmonic function (and check convergence) with 'epic' library.");
            return;
        }
    } else {
        int result = harmonic_update_and_check_cpu(&harmonic);
        if (result == EPIC_SUCCESS) {
            for (unsigned int i = 0; i < num_steps - 1; i++) {
                if (harmonic_update_cpu(&harmonic) != EPIC_SUCCESS) {
                    ROS_ERROR("Error[EpicNavigationNodeHarmonic::update]: Failed to use CPU to relax harmonic function with 'epic' library.");
                    return;
                }
            }
        } else if (result != EPIC_SUCCESS_AND_CONVERGED) {
            ROS_ERROR("Error[EpicNavigationNodeHarmonic::update]: Failed to use CPU to relax harmonic function (and check convergence) with 'epic' library.");
            return;
        }
    }
}


void EpicNavigationNodeHarmonic::initAlg(unsigned int w, unsigned int h)
{
    if (harmonic.n > 0 || harmonic.m != nullptr || harmonic.u != nullptr || harmonic.locked != nullptr) {
        ROS_ERROR("Error[EpicNavigationNodeHarmonic::initAlg]: Harmonic object is already initialized and must be freed first.");
        return;
    }

    width = w;
    height = h;

    init_alg = true;

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
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::createHarmonic]: Could not initialize GPU. Defaulting to CPU version.");
        gpu = false;
    } else {
        gpu = true;
    }
}


void EpicNavigationNodeHarmonic::uninitAlg()
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::uninitAlg]: Algorithm has not been initialized yet.");
        return;
    }

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
            ROS_WARN("Warning[EpicNavigationNodeHarmonic::uninitAlg]: Failed to uninitialize GPU variables.");
        }
    }

    init_alg = false;
}


void EpicNavigationNodeHarmonic::setBoundariesAsObstacles()
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::setBoundariesAsObstacles]: Algorithm has not been initialized yet.");
        return;
    }

    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr) {
        ROS_ERROR("Error[EpicNavigationNodeHarmonic::setBoundariesAsObstacles]: Harmonic object is not initialized.");
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


void EpicNavigationNodeHarmonic::mapToWorld(float mx, float my, float &wx, float &wy) const
{
    wx = x_origin + mx * resolution;
    wy = y_origin + my * resolution;
}


bool EpicNavigationNodeHarmonic::worldToMap(float wx, float wy, float &mx, float &my) const
{
    if (wx < x_origin || wy < y_origin ||
            wx >= x_origin + width * resolution ||
            wy >= y_origin + height * resolution) {
        ROS_WARN("Error[EpicNavigationNodeHarmonic::worldToMap]: World coordinates are outside map.");
        return false;
    }

    mx = (wx - x_origin) / resolution;
    my = (wy - y_origin) / resolution;

    return true;
}


bool EpicNavigationNodeHarmonic::isCellObstacle(unsigned int x, unsigned int y)
{
    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr ||
            x >= harmonic.m[1] || y >= harmonic.m[0]) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::isCellGoal]: Location provided is outside of the map; it is obviously an obstacle.");
        return true;
    }

    return (harmonic.u[y * harmonic.m[1] + x] == EPIC_LOG_SPACE_OBSTACLE && harmonic.locked[y * harmonic.m[1] + x] == 1);
}


bool EpicNavigationNodeHarmonic::isCellGoal(unsigned int x, unsigned int y)
{
    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr ||
            x >= harmonic.m[1] || y >= harmonic.m[0]) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::isCellGoal]: Location provided is outside of the map; it is obviously not a goal.");
        return false;
    }

    return (harmonic.u[y * harmonic.m[1] + x] == EPIC_LOG_SPACE_GOAL && harmonic.locked[y * harmonic.m[1] + x] == 1);
}


bool EpicNavigationNodeHarmonic::setCells(std::vector<unsigned int> &v, std::vector<unsigned int> &types)
{
    // Note: This trick with vectors only works for C++11 or greater; the spec updated to guarantee
    // that vector objects store contiguously in memory.
    int result = 0;

    // We always update the CPU map here. Namely for the locked variable.
    result = harmonic_utilities_set_cells_2d_cpu(&harmonic, types.size(), &v[0], &types[0]);
    if (result != EPIC_SUCCESS) {
        ROS_ERROR("Error[EpicNavigationNodeHarmonic::setCells]: Failed to set the cells on the CPU.");
        return false;
    }

    // Optionally, we update the GPU side if it is being used.
    if (gpu) {
        result = harmonic_utilities_set_cells_2d_gpu(&harmonic, num_gpu_threads, types.size(), &v[0], &types[0]);
        if (result != EPIC_SUCCESS) {
            ROS_ERROR("Error[EpicNavigationNodeHarmonic::setCells]: Failed to set the cells on the GPU.");
            return false;
        }
    }

    return true;
}


void EpicNavigationNodeHarmonic::subOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // If the size changed, then free everything and start over.
    // Note we don't care if the resolution or offsets change.
    if (msg->info.width != width || msg->info.height != height) {
        uninitAlg();
        initAlg(msg->info.width, msg->info.height);
    }

    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::subOccupancyGrid]: Algorithm was not initialized.");
        return;
    }

    // These only affect the final path, not the path computation.
    x_origin = msg->info.origin.position.x;
    y_origin = msg->info.origin.position.y;
    resolution = msg->info.resolution;

    // Copy the data to the Harmonic object.
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

    setCells(v, types);

    v.clear();
    types.clear();
}


bool EpicNavigationNodeHarmonic::srvSetStatus(epic::SetStatus::Request &req, epic::SetStatus::Response &res)
{
    paused = (bool)req.paused;

    // TODO: Set gpu assignment. Requires initialize and uninitialize code too.

    res.success = true;

    return true;
}


bool EpicNavigationNodeHarmonic::srvAddGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::srvAddGoals]: Algorithm was not initialized.");
        res.success = false;
        return false;
    }

    std::vector<unsigned int> v;
    std::vector<unsigned int> types;

    for (unsigned int i = 0; i < req.goals.size(); i++) {
        float x = 0.0f;
        float y = 0.0f;

        worldToMap(req.goals[i].pose.position.x, req.goals[i].pose.position.y, x, y);

        // If the goal location is an obstacle, then do not let it add a goal here.
        if (isCellObstacle((unsigned int)(x + 0.5f), (unsigned int)(y + 0.5f))) {
            continue;
        }

        v.push_back((unsigned int) x);
        v.push_back((unsigned int) y);
        types.push_back(EPIC_CELL_TYPE_GOAL);
    }

    if (v.size() == 0) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::srvAddGoals]: Attempted to add goal(s) inside obstacles. No goals added.");
        res.success = false;
        return false;
    }

    setCells(v, types);

    v.clear();
    types.clear();

    res.success = true;

    return true;
}


bool EpicNavigationNodeHarmonic::srvRemoveGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::srvRemoveGoals]: Algorithm was not initialized.");
        res.success = false;
        return false;
    }

    std::vector<unsigned int> v;
    std::vector<unsigned int> types;

    // Note: Removing goals turns them into free space. Recall, however, that goals can
    // only be added on free space (above).
    for (unsigned int i = 0; i < req.goals.size(); i++) {
        float x = 0.0f;
        float y = 0.0f;

        if (!worldToMap(req.goals[i].pose.position.x, req.goals[i].pose.position.y, x, y)) {
            continue;
        }

        v.push_back((unsigned int) x);
        v.push_back((unsigned int) y);
        types.push_back(EPIC_CELL_TYPE_FREE);
    }

    setCells(v, types);

    v.clear();
    types.clear();

    res.success = true;

    return true;
}


bool EpicNavigationNodeHarmonic::srvGetCell(epic::GetCell::Request &req, epic::GetCell::Response &res)
{
    if (harmonic.m == nullptr || harmonic.u == nullptr || harmonic.locked == nullptr ||
            req.x >= harmonic.m[1] || req.y >= harmonic.m[0]) {
        ROS_WARN("Error[EpicNavigationNodeHarmonic::srvGetCell]: Location provided is outside of the map.");
        return false;
    }

    if (gpu) {
        if (harmonic_get_potential_values_gpu(&harmonic) != EPIC_SUCCESS) {
            ROS_WARN("Error[EpicNavigationNodeHarmonic::srvGetCell]: Failed to get the potential values from the GPU.");
            res.success = false;
            return false;
        }
    }

    res.value = harmonic.u[req.y * harmonic.m[1] + req.x];
    res.success = true;

    return true;
}


bool EpicNavigationNodeHarmonic::srvSetCells(epic::SetCells::Request &req, epic::SetCells::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::srvSetCells]: Algorithm was not initialized.");
        res.success = false;
        return false;
    }

    std::vector<unsigned int> v;
    std::vector<unsigned int> types;

    for (unsigned int i = 0; i < req.types.size(); i++) {
        // Note: For the SetCells service, these are assigning the raw cells, not picking world coordinates.
        // Thus, no worldToMap translation is required.
        unsigned int x = req.v[2 * i + 0];
        unsigned int y = req.v[2 * i + 1];

        if (x >= width || y >= height) {
            continue;
        }

        v.push_back((unsigned int)req.v[2 * i + 0]);
        v.push_back((unsigned int)req.v[2 * i + 1]);
        types.push_back((unsigned int)req.types[i]);
    }

    setCells(v, types);

    v.clear();
    types.clear();

    res.success = true;

    return true;
}


bool EpicNavigationNodeHarmonic::srvResetFreeCells(epic::ResetFreeCells::Request &req, epic::ResetFreeCells::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::srvResetFreeCells]: Algorithm was not initialized.");
        res.success = false;
        return false;
    }

    std::vector<unsigned int> v;
    std::vector<unsigned int> types;

    for (unsigned int y = 1; y < height - 1; y++) {
        for (unsigned int x = 1; x < width - 1; x++) {
            if (harmonic.locked[y * width + x] == 0) {
                v.push_back(x);
                v.push_back(y);
                types.push_back(EPIC_CELL_TYPE_FREE);
            }
        }
    }

    setCells(v, types);

    v.clear();
    types.clear();

    res.success = true;

    return true;
}


bool EpicNavigationNodeHarmonic::srvComputePath(epic::ComputePath::Request &req, epic::ComputePath::Response &res)
{
    if (!init_alg) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::srvComputePath]: Algorithm was not initialized.");
        return false;
    }

    if (gpu) {
        if (harmonic_get_potential_values_gpu(&harmonic) != EPIC_SUCCESS) {
            ROS_WARN("Error[EpicNavigationNodeHarmonic::srvComputePath]: Failed to get the potential values from the GPU.");
            return false;
        }
    }

    float x = 0.0f;
    float y = 0.0f;
    if (!worldToMap(req.start.pose.position.x, req.start.pose.position.y, x, y)) {
        ROS_WARN("Warning[EpicNavigationNodeHarmonic::srvComputePath]: Could not convert start to floating point map location.");
    }

    unsigned int k = 0;
    float *raw_path = nullptr;

    int result = harmonic_compute_path_2d_cpu(&harmonic, x, y, req.step_size, req.precision, req.max_length, k, raw_path);

    if (result != EPIC_SUCCESS) {
        ROS_ERROR("Error[EpicNavigationNodeHarmonic::srvComputePath]: Failed to compute the path.");

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

}; // namespace epic


