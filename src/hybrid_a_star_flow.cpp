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
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    double steering_angle = nh.param("planner/steering_angle", 60);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1); //1
    double wheel_base = nh.param("planner/wheel_base", 2); 
    double segment_length = nh.param("planner/segment_length", 2); //1.6
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8); //8
    double steering_penalty = nh.param("planner/steering_penalty", 2); //2
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 2); //2
    double reversing_penalty = nh.param("planner/reversing_penalty", 2); //2.0
    double shot_distance = nh.param("planner/shot_distance", 10.0); //5.0

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );

    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);

    ROS_INFO("HYBRID ASTAR STARTED...");

    has_map_ = false;
}

HybridAStarFlow::~HybridAStarFlow()
	{
	}

void HybridAStarFlow::Run() {
    if (!has_map_) {
        
        while (costmap_deque_.empty()) {
            costmap_sub_ptr_->ParseData(costmap_deque_);
         }
    
        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();
        const double map_resolution = 0.025;
        kinodynamic_astar_searcher_ptr_->Init(
                current_costmap_ptr_->info.origin.position.x,
                1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.origin.position.y,
                1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.resolution,
                map_resolution
        );
        unsigned int map_w = std::floor(current_costmap_ptr_->info.width);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height);
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);
                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);
               if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x] > 0) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }
        has_map_ = true;
    }
    costmap_deque_.clear();
    
        double start_yaw = tf::getYaw(init_pose_.pose.orientation);
        double goal_yaw = tf::getYaw(move_base_goal_.pose.orientation);

        Vec3d start_state = Vec3d(
                 init_pose_.pose.position.x,
                 init_pose_.pose.position.y,
                 start_yaw
         );
        Vec3d goal_state = Vec3d(
                move_base_goal_.pose.position.x,
                move_base_goal_.pose.position.y,
                goal_yaw
        );


        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();
            PublishPath(path);

            // nav_msgs::Path path_ros;
            // geometry_msgs::PoseStamped pose_stamped;

            // for (const auto &pose: path) {
            //     pose_stamped.header.frame_id = "map";
            //     pose_stamped.pose.position.x = pose.x();
            //     pose_stamped.pose.position.y = pose.y();
            //     pose_stamped.pose.position.z = 0.0;

            //     pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

            //     path_ros.poses.emplace_back(pose_stamped);
            // }

            // path_ros.header.frame_id = "map";
            // path_ros.header.stamp = ros::Time::now();
            // static tf::TransformBroadcaster transform_broadcaster;
            // for (const auto &pose: path_ros.poses) {
            //     tf::Transform transform;
            //     transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

            //     tf::Quaternion q;
            //     q.setX(pose.pose.orientation.x);
            //     q.setY(pose.pose.orientation.y);
            //     q.setZ(pose.pose.orientation.z);
            //     q.setW(pose.pose.orientation.w);
            //     transform.setRotation(q);

            //     transform_broadcaster.sendTransform(tf::StampedTransform(transform,
            //                                                              ros::Time::now(), "world",
            //                                                              "ground_link")
            //     );

            //     ros::Duration(0.05).sleep();
            // }
        }


        // debug
//        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
        kinodynamic_astar_searcher_ptr_->Reset();
    }





void HybridAStarFlow::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;
    ros::Time plan_time = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = plan_time;
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);

        plan_gp_ros.push_back(pose_stamped);
    }

    nav_path.header.frame_id = "map";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
    
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "map";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

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

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
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