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


#ifndef EPIC_NAVIGATION_NODE_HARMONIC_H
#define EPIC_NAVIGATION_NODE_HARMONIC_H


#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <epic/harmonic/harmonic.h>

#include <epic/epic_navigation_node.h>

// Generated by having the .srv files defined.
#include <epic/SetStatus.h>
#include <epic/ModifyGoals.h>
#include <epic/GetCell.h>
#include <epic/SetCells.h>
#include <epic/ResetFreeCells.h>
#include <epic/ComputePath.h>

namespace epic {

class EpicNavigationNodeHarmonic : public EpicNavigationNode {
public:
    /**
     *  The default constructor for the EpicNavigationNodeHarmonic.
     *  @param  nh      The node handle from main.
     */
    EpicNavigationNodeHarmonic(ros::NodeHandle &nh);

    /**
     *  The deconstructor for the EpicNavigationNodeHarmonic.
     */
    virtual ~EpicNavigationNodeHarmonic();

    /**
     *  Initialize the services, messages, and algorithm variables. Implemented for EpicNavigationNode.
     *  @return True if successful in registering, subscribing, etc.; false otherwise.
     */
    bool initialize();

    /**
     *  Update the harmonic function one step. Implemented for EpicNavigationNode.
     *  @param  num_steps   The number of steps to do.
     */
    void update(unsigned int num_steps);

protected:
    /**
     *  Initialize the algorithm.
     *  @param  w   The new width.
     *  @param  h   The new height.
     */
    void initAlg(unsigned int w, unsigned int h);

    /**
     *  Uninitialize the algorithm.
     */
    void uninitAlg();

    /**
     *  Set the boundaries as obstacles for the occupancy grid.
     */
    void setBoundariesAsObstacles();

    /**
     *  Convert a "float pixel" map coordinate to a world coordinate.
     *  @param  mx  The "float pixel" map x coordinate.
     *  @param  my  The "float pixel" map y coordinate.
     *  @param  wx  The world x coordinate. This will be modified.
     *  @param  wy  The world y coordinate. This will be modified.
     */
    void mapToWorld(float mx, float my, float &wx, float &wy) const;

    /**
     *  Convert a "float pixel" map coordinate to a world coordinate.
     *  @param  wx  The world x coordinate.
     *  @param  wy  The world y coordinate.
     *  @param  mx  The "float pixel" map x coordinate. This will be modified.
     *  @param  my  The "float pixel" map y coordinate. This will be modified.
     */
    bool worldToMap(float wx, float wy, float &mx, float &my) const;

    /**
     *  Check if a cell is an obstacle or not.
     *  @param  x   The x cell index.
     *  @param  y   The y cell index.
     *  @return True if it is an obstacle, false otherwise.
     */
    bool isCellObstacle(unsigned int x, unsigned int y);

    /**
     *  Check if a cell is a goal or not.
     *  @param  x   The x cell index.
     *  @param  y   The y cell index.
     *  @return True if it is a goal, false otherwise.
     */
    bool isCellGoal(unsigned int x, unsigned int y);

    /**
     *  Set the cells based on the values and types provided. This handles CPU vs. GPU assignment.
     *  @param  v       The n locations (2-n array) of each cell as indexes.
     *  @param  types   The n types of each cell to set.
     *  @return True if successful, false otherwise.
     */
    bool setCells(std::vector<unsigned int> &v, std::vector<unsigned int> &types);

    /**
     *  Handler for receiving OccupancyGrid messages (see defines above; values differ here).
     *  @param  msg     The OccupancyGrid message.
     */
    void subOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    /**
     *  Handler for receiving set status service request.
     *  @param  req     The SetStatus service request containing the goal location(s) to add.
     *  @param  res     The SetStatus service response containing if it was successful or not. 
     *  @return Returns true if successful, false otherwise.
     */
    bool srvSetStatus(epic::SetStatus::Request &req, epic::SetStatus::Response &res);

    /**
     *  Handler for receiving add goal(s) service request. Assumption: These are not initially obstacles.
     *  @param  req     The AddGoal service request containing the goal location(s) to add.
     *  @param  res     The AddGoal service response containing if it was successful or not. 
     *  @return Returns true if successful, false otherwise.
     */
    bool srvAddGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res);

    /**
     *  Handler for receiving remove goal(s) messages. Assumption: Removed goals become free space.
     *  @param  req     The RemoveGoal service request containing the goal location(s) to remove.
     *  @param  res     The RemoveGoal service response containing if it was successful or not.
     *  @return Returns true if successful, false otherwise.
     */
    bool srvRemoveGoals(epic::ModifyGoals::Request &req, epic::ModifyGoals::Response &res);

    /**
     *  Handler for receiving get cell messages. This allows us to get the log hitting probability of a cell.
     *  @param  req     The GetCell service request containing the cell value.
     *  @param  res     The GetCell service response containing if it was successful or not.
     *  @return Returns true if successful, false otherwise.
     */
    bool srvGetCell(epic::GetCell::Request &req, epic::GetCell::Response &res);

    /**
     *  Handler for receiving set cells messages. This allows for quick updates to the occupancy grid.
     *  @param  req     The SetCells service request containing the cells to set and resulting desired type.
     *  @param  res     The SetCells service response containing if it was successful or not.
     *  @return Returns true if successful, false otherwise.
     */
    bool srvSetCells(epic::SetCells::Request &req, epic::SetCells::Response &res);

    /**
     *  Handler for receiving reset messages. This completely resets the map (e.g., for drastic goal changes).
     *  @param  req     The ResetFreeCells service request.
     *  @param  res     The ResetFreeCells service response containing if it was successful or not.
     *  @return Returns true if successful, false otherwise.
     */
    bool srvResetFreeCells(epic::ResetFreeCells::Request &req, epic::ResetFreeCells::Response &res);

    /**
     *  Handler for service requests for generating Path messages.
     *  @param  req     The ComputePath service request containing the start location, max length, precision, etc.
     *  @param  res     The ComputePath service response containing the resulting path.
     *  @return Returns true if successful, false otherwise.
     */
    bool srvComputePath(epic::ComputePath::Request &req, epic::ComputePath::Response &res);

    // The subscriber for the OccupancyGrid message.
    ros::Subscriber sub_occupancy_grid;

    // The service for SetStatus.
    ros::ServiceServer srv_set_status;

    // The service for AddGoal.
    ros::ServiceServer srv_add_goals;

    // The service for RemoveGoal.
    ros::ServiceServer srv_remove_goals;

    // The service for GetCell.
    ros::ServiceServer srv_get_cell;

    // The service for SetCells.
    ros::ServiceServer srv_set_cells;

    // The service for ResetFreeCells.
    ros::ServiceServer srv_reset_free_cells;

    // The service for ComputePath.
    ros::ServiceServer srv_compute_path;

    // Current width of the map.
    unsigned int width;

    // Current height of the map.
    unsigned int height;

    // Current resolution of the map.
    float resolution;

    // Current x origin offset of the map.
    float x_origin;

    // Current y origin offset of the map.
    float y_origin;

    // The Harmonic object.
    Harmonic harmonic;

    // The number of GPU threads for the Harmonic function.
    unsigned int num_gpu_threads;

    // If this is paused (i.e., basically do not run update), or not.
    bool paused;

    // If this is GPU-capable, or not.
    bool gpu;

    // If this class' services and subscriptions have been properly initialized or not.
    bool init_msgs;

    // If this class' algorithm variables have been properly initialized or not.
    bool init_alg;

};

};


#endif // EPIC_NAVIGATION_NODE_HARMONIC_H

