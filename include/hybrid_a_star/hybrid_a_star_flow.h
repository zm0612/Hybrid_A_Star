/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
#define HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H

#include "hybrid_a_star.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class HybridAStarFlow {
public:
    HybridAStarFlow() = default;

    explicit HybridAStarFlow(ros::NodeHandle &nh);

    void Run();

private:
    void InitPoseData();

    void CallbackCostMap(const nav_msgs::OccupancyGridConstPtr &msg_costmap);

    void CallbackInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg_initpose);

    void CallbackGoalPose(const geometry_msgs::PoseStampedConstPtr &msg_goalpose);

    void PublishPath(const VectorVec3d &path);

    void PublishSearchedTree(const VectorVec4d &searched_tree);

    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);

private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
    ros::Subscriber costmap_sub;
    ros::Subscriber init_pose_sub;
    ros::Subscriber goal_pose_sub;
    bool is_costmap_init;
    bool is_costmap_get;
    bool is_init_pose_get;
    bool is_goal_pose_get;

    ros::Publisher path_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;

    geometry_msgs::PoseWithCovarianceStamped current_init_pose;
    geometry_msgs::PoseStamped               current_goal_pose;
    nav_msgs::OccupancyGrid                  current_costmap;

    ros::Time timestamp_;
};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
