#include "hybrid_a_star/hybrid_astar_global_planner.h"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(hybrid_astar_global_planner::HybridAstarGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_global_planner

{
    HybridAstarGlobalPlanner::HybridAstarGlobalPlanner() :
        costmap_(NULL),initialized_(false) {
	}

	HybridAstarGlobalPlanner::HybridAstarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
	  initialize(name, costmap_ros);
	}

	HybridAstarGlobalPlanner::~HybridAstarGlobalPlanner()
	{
	}

    void HybridAstarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (!initialized_) {
        ros::NodeHandle node_handle("~/" + name);
        kino = std::make_shared<HybridAStarFlow>(node_handle);
        name_ = name;
        costmap_ = costmap_ros;
        initialized_ = true;
    }
    }

    bool HybridAstarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
  		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
          {   
               plan.clear();
               kino->plan_gp_ros.clear();
               kino->init_pose_ = start_pose;
               kino->move_base_goal_ = goal;
               ROS_INFO("NEW GOAL : %f", kino->move_base_goal_.pose.position.x);
               kino->Run();
               plan = kino->plan_gp_ros;

              return true;
          }


}