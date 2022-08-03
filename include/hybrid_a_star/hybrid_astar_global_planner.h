#ifndef HYBRID_ASTAR_GLOBAL_PLANNER_H
#define HYBRID_ASTAR_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include "hybrid_a_star/hybrid_a_star_flow.h"

namespace hybrid_astar_global_planner {


class HybridAstarGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        
    HybridAstarGlobalPlanner();

    HybridAstarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap);

    ~HybridAstarGlobalPlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);


    private:
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_;
    std::string name_;
    std::shared_ptr<HybridAStarFlow> kino;

    };
    

}
#endif