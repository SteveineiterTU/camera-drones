/*
 * Octomap Path Planner ROS - octomap_path_planner_ros.h
 *
 *  Author: Stefan Schorkmeier <s.schoerkmeier@student.tugraz.at>
 *  Created on: November 25, 2023
 *
 */

#ifndef OCTOMAP_PATH_PLANNER_H_
#define OCTOMAP_PATH_PLANNER_H_

#include "ros/ros.h"
#include <octomap_path_planner/octomap_helper_functions.h>
// #include <geometry_msgs/Point.h>
#include <mav_planning_msgs/Point2D.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
// 2D Version
//#include <mav_planning_msgs/Point2D.h>
// 3D Version
#include <geometry_msgs/Point.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>
// #include <any> // NOT working becuase we don't have C++17 and higher -.- man i wanted to use a map so badly for the informaion
#include <chrono>
#include <list>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

class OctomapPathPlanner
{
public:
    OctomapPathPlanner(ros::NodeHandle &n, ros::NodeHandle &pn, int argc, char** argv);
    ~OctomapPathPlanner();

private:
    ros::NodeHandle &pnode_;
    ros::NodeHandle &node_;

    // parsed arguments
    double runTime;
    optimalPlanner plannerType;
    planningObjective objectiveType;
    std::string outputFile;
    std::string octomapFile;

    void plan();

    // ROS Topics
    ros::Subscriber current_position_sub;
    ros::Subscriber goal_position_sub;
    ros::Publisher trajectory_pub;
    void currentPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg);
    void goalPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg);
    void convertOMPLPathToMsg();
    void printRelevantInformation();
    double calculateAverageClearance();
    geometry_msgs::Point current_position, goal_position;
    bool traj_planning_successful;
    std::shared_ptr<ompl::geometric::PathGeometric> p_last_traj_ompl;
    mav_planning_msgs::PolynomialTrajectory4D last_traj_msg;

    // Our Variables
    // std::map<std::string, std::variant<int, double>> relevant_path_information;
    double path_length;
    double path_cost;
    std::chrono::milliseconds path_calculation_time;
    std::list<double> clearence_list; 

    DynamicEDTOctomap *distmap;
    octomap::OcTree *tree;

    // Our functions
 
};

#endif // OCTOMAP_PATH_PLANNER_H_
