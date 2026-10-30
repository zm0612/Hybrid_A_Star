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

#include "hybrid_a_star/hybrid_a_star_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <functional>
#include <algorithm>
#include <cmath>
#include <vector>

__attribute__((unused)) static double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    double steering_angle = nh.param("planner/steering_angle", 10);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    double wheel_base = nh.param("planner/wheel_base", 1.0);
    double segment_length = nh.param("planner/segment_length", 1.6);
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    double steering_penalty = nh.param("planner/steering_penalty", 1.05);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    double shot_distance = nh.param("planner/shot_distance", 5.0);

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
        steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
        steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    optimized_path_pub_ = nh.advertise<nav_msgs::Path>("optimized_path", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);

    // ---- Post-processing / back-end optimization configuration ----
    enable_post_process_ = nh.param("planner/enable_post_process", true);

    const double move_step_size = segment_length / segment_length_discrete_num;
    resample_spacing_ = move_step_size;

    // Redundant-reversal removal.
    path_processor_.set_min_segment_length(nh.param("planner/opt_min_segment_length", 3.0));
    path_processor_.set_heading_tol(nh.param("planner/opt_heading_tol", 0.6));
    path_processor_.set_step_size(move_step_size);

    // Ceres smoother.
    TrajectoryOptimizer::Config opt_cfg;
    opt_cfg.w_smooth = nh.param("planner/opt_w_smooth", 1.0);
    opt_cfg.w_reference = nh.param("planner/opt_w_reference", 0.1);
    opt_cfg.w_obstacle = nh.param("planner/opt_w_obstacle", 2.0);
    opt_cfg.w_curvature = nh.param("planner/opt_w_curvature", 1.0);
    opt_cfg.w_length = nh.param("planner/opt_w_length", 0.1);
    opt_cfg.safety_margin = nh.param("planner/opt_safety_margin", 0.2);
    opt_cfg.num_circles = nh.param("planner/opt_num_circles", 3);
    opt_cfg.obstacle_eps = nh.param("planner/opt_obstacle_eps", 0.05);
    opt_cfg.max_iterations = nh.param("planner/opt_max_iterations", 100);
    opt_cfg.cusp_taper_dist = nh.param("planner/opt_cusp_taper_dist", 2.0);
    opt_cfg.cusp_taper_min_ratio = nh.param("planner/opt_cusp_taper_min_ratio", 0.1);
    opt_cfg.min_opt_segment_length = nh.param("planner/opt_min_opt_segment_length", 2.0);
    opt_cfg.cusp_fix_dist = nh.param("planner/opt_cusp_fix_dist", 0.0);
    opt_cfg.endpoint_fix_dist = nh.param("planner/opt_endpoint_fix_dist", 0.0);
    opt_num_passes_ = nh.param("planner/opt_num_passes", 3);
    opt_cfg.vehicle_length = vehicle_length_;
    opt_cfg.vehicle_width = vehicle_width_;
    opt_cfg.rear_axle_dist = rear_axle_dist_;
    trajectory_optimizer_ = TrajectoryOptimizer(opt_cfg);
    trajectory_optimizer_.SetKinematics(wheel_base, steering_angle * M_PI / 180.0);

    // Clearance (circle radius) used by the reverse-removal collision check,
    // matching the covering-circle model of the optimizer.
    {
        const double front = vehicle_length_ - rear_axle_dist_;
        const double rear = -rear_axle_dist_;
        const int num_circles = std::max(1, opt_cfg.num_circles);
        const double spacing = num_circles > 1 ? (front - rear) / (num_circles - 1) : vehicle_length_;
        collision_clearance_ = std::hypot(spacing * 0.5, vehicle_width_ * 0.5);
    }

    has_map_ = false;
}

void HybridAStarFlow::Run() {
    ReadData();

    if (!has_map_) {
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = static_cast<float>(current_costmap_ptr_->info.resolution);
        kinodynamic_astar_searcher_ptr_->Init(
            current_costmap_ptr_->info.origin.position.x,
            1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
            current_costmap_ptr_->info.origin.position.y,
            1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
            1.0, map_resolution
        );

        unsigned int map_w = std::floor(current_costmap_ptr_->info.width);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height);
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                if (current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w]) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }

        BuildDistanceField();
        has_map_ = true;
    }
    costmap_deque_.clear();

    while (HasStartPose() && HasGoalPose()) {
        InitPoseData();

        double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);

        Vec3d start_state = Vec3d(
            current_init_pose_ptr_->pose.pose.position.x,
            current_init_pose_ptr_->pose.pose.position.y,
            start_yaw
        );
        Vec3d goal_state = Vec3d(
            current_goal_pose_ptr_->pose.position.x,
            current_goal_pose_ptr_->pose.position.y,
            goal_yaw
        );

        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();

            // Front-end (raw) path, published for comparison in rviz.
            PublishPath(path);

            // Back-end: remove redundant reversals + Ceres smoothing.
            auto optimized_path = enable_post_process_ ? PostProcess(path) : path;

            PublishOptimizedPath(optimized_path);
            PublishVehiclePath(optimized_path, 4.0, 2.0, 5u);
            PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());

            nav_msgs::Path path_ros;
            geometry_msgs::PoseStamped pose_stamped;

            for (const auto &pose: optimized_path) {
                pose_stamped.header.frame_id = "world";
                pose_stamped.pose.position.x = pose.x();
                pose_stamped.pose.position.y = pose.y();
                pose_stamped.pose.position.z = 0.0;

                pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

                path_ros.poses.emplace_back(pose_stamped);
            }

            path_ros.header.frame_id = "world";
            path_ros.header.stamp = ros::Time::now();
            static tf::TransformBroadcaster transform_broadcaster;
            for (const auto &pose: path_ros.poses) {
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

                tf::Quaternion q;
                q.setX(pose.pose.orientation.x);
                q.setY(pose.pose.orientation.y);
                q.setZ(pose.pose.orientation.z);
                q.setW(pose.pose.orientation.w);
                transform.setRotation(q);

                transform_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                                         ros::Time::now(), "world",
                                                                         "ground_link")
                );

                ros::Duration(0.05).sleep();
            }
        }

        // debug
        //        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
        kinodynamic_astar_searcher_ptr_->Reset();
    }
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool HybridAStarFlow::HasGoalPose() const {
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasStartPose() const {
    return !init_pose_deque_.empty();
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path) const {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void HybridAStarFlow::PublishOptimizedPath(const VectorVec3d &path) const {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();

    optimized_path_pub_.publish(nav_path);
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, const double width,
                                         const double length, const unsigned int vehicle_interval = 5u) const {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.03;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) const {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}

void HybridAStarFlow::BuildDistanceField() {
    if (!current_costmap_ptr_) {
        return;
    }

    const int width = static_cast<int>(current_costmap_ptr_->info.width);
    const int height = static_cast<int>(current_costmap_ptr_->info.height);
    const double resolution = current_costmap_ptr_->info.resolution;
    const double origin_x = current_costmap_ptr_->info.origin.position.x;
    const double origin_y = current_costmap_ptr_->info.origin.position.y;

    // Treat any non-free cell (occupied or unknown) as an obstacle, matching the
    // front-end occupancy handling above.
    std::vector<uint8_t> occupancy(static_cast<std::size_t>(width) * height, 0u);
    for (std::size_t i = 0; i < occupancy.size() && i < current_costmap_ptr_->data.size(); ++i) {
        occupancy[i] = current_costmap_ptr_->data[i] != 0 ? 1u : 0u;
    }

    esdf_.Build(occupancy, width, height, resolution, origin_x, origin_y, 1u);
    trajectory_optimizer_.SetEsdf(&esdf_);

    ROS_INFO("\033[1;32m[PostProcess] ESDF built: %d x %d, resolution %.3f \033[0m",
             width, height, resolution);
}

VectorVec3d HybridAStarFlow::PostProcess(const VectorVec3d &raw_path) const {
    if (raw_path.size() < 2 || !esdf_.IsValid()) {
        return raw_path;
    }

    // 1. Re-derive the per-point driving direction (forward / backward).
    Trajectory traj = InferGears(raw_path);
    const int cusps_before = PathProcessor::CountCusps(traj);

    // 2. Remove redundant reversals. The collision predicate uses the covering
    //    circles of the vehicle footprint queried against the ESDF.
    const double length = vehicle_length_;
    const double rear_axle = rear_axle_dist_;
    const double clearance = collision_clearance_;
    const ESDF *esdf = &esdf_;

    PathProcessor::CollisionChecker collision_free =
            [esdf, length, rear_axle, clearance](double x, double y, double theta) -> bool {
        const double front = length - rear_axle;
        const double rear = -rear_axle;
        const double offsets[3] = {rear, 0.5 * (rear + front), front};
        for (const double d: offsets) {
            const double cx = x + d * std::cos(theta);
            const double cy = y + d * std::sin(theta);
            if (esdf->Distance(cx, cy) <= clearance) {
                return false;
            }
        }
        return true;
    };

    traj = path_processor_.RemoveRedundantReversals(traj, collision_free);
    const int cusps_after = PathProcessor::CountCusps(traj);

    // 3. Ceres back-end smoothing (smoothness + reference + obstacle + curvature +
    //    length), run over several passes. A single solve flattens high-frequency
    //    wiggles but leaves long-wavelength meanders behind: low-frequency modes
    //    converge much more slowly under a second-difference (curvature) smoother.
    //    Re-sampling between passes re-discretizes the path and re-anchors the
    //    obstacle/reference terms to the progressively smoother geometry, so each
    //    pass straightens the path a little further while staying collision-free.
    for (int pass = 0; pass < opt_num_passes_; ++pass) {
        traj = trajectory_optimizer_.Optimize(traj);
        traj = PathProcessor::Resample(traj, resample_spacing_);
    }

    ROS_INFO("\033[1;32m[PostProcess] cusps: %d -> %d, points: %zu \033[0m",
             cusps_before, cusps_after, traj.size());

    return ToVectorVec3d(traj);
}
