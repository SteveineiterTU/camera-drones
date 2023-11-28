/*
 * Octomap Path Planner ROS - octomap_path_planner_ros.cpp
 *
 *  Author: Stefan Schorkmeier <s.schoerkmeier@student.tugraz.at>
 *  Created on: November 25, 2023
 *
 */

#include <octomap_path_planner/octomap_path_planner_ros.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <cmath>

OctomapPathPlanner::OctomapPathPlanner(ros::NodeHandle &n, ros::NodeHandle &pn, int argc, char** argv) :
    pnode_(pn),
    node_(n),
    traj_planning_successful(false)
{   
    // Parse the arguments, returns true if successful, false otherwise
    if (argParse(argc, argv, &runTime, &plannerType, &objectiveType, &outputFile, &octomapFile))
    {
        // Return with success
        ROS_INFO("OctomapPathPlanner::OctomapPathPlanner(...) argParse success!");
    } else {
        // Return with error - Modified argParse to make this equivalent to giving no arguments.
        ROS_INFO("OctomapPathPlanner::OctomapPathPlanner(...) argParse error!");
    }

    // ROS topics
    current_position_sub = pnode_.subscribe("current_position", 10, &OctomapPathPlanner::currentPositionCallback, this);
    goal_position_sub = pnode_.subscribe("goal_position", 10, &OctomapPathPlanner::goalPositionCallback, this);
    trajectory_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory", 1);

    current_position.x = 0.; current_position.y = 0.; current_position.z = 0.;
    goal_position.x = 1.; goal_position.y = 1.; goal_position.z = 1.;

    tree = new octomap::OcTree(0.05);
    tree->readBinary(octomapFile);
    ROS_INFO_STREAM("read in tree, " << tree->getNumLeafNodes() << " leaves ");

    // Setting up the dist map. I assume we have a static octomap aka do not have to change this map. 
    // Also by having this in the clearance function for example it resulted in weird errors aka i could
    // not calculate any path (eg from 0 0 0 to 1 0 0)
    double a,b,c;
    tree->getMetricMin(a,b,c);
    octomap::point3d min(a,b,c);
    tree->getMetricMax(a,b,c);
    octomap::point3d max(a,b,c);

    bool unknownAsOccupied = false;
    float maxDist = 50.0;
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    //The constructor copies data but does not yet compute the distance map
    distmap = new DynamicEDTOctomap(maxDist, tree, min, max, unknownAsOccupied);

    //This computes the distance map
    distmap->update(); 
}

OctomapPathPlanner::~OctomapPathPlanner() {

}

void OctomapPathPlanner::currentPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    current_position = *p_msg;
    ROS_INFO_STREAM("New current position, x: " << current_position.x << "; y: " << current_position.y << "; z: " << current_position.z);
}

void OctomapPathPlanner::goalPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    goal_position = *p_msg;
    ROS_INFO_STREAM("New goal position, x: " << goal_position.x << "; y: " << goal_position.y << "; z: " << goal_position.z);

    plan();

    if (traj_planning_successful) {
        convertOMPLPathToMsg();
        mav_planning_msgs::PolynomialTrajectory4D::Ptr p_traj_msg = 
            mav_planning_msgs::PolynomialTrajectory4D::Ptr( new mav_planning_msgs::PolynomialTrajectory4D( last_traj_msg ) );
        trajectory_pub.publish(last_traj_msg);
    }
    printRelevantInformation();
}

void OctomapPathPlanner::convertOMPLPathToMsg() {
    mav_planning_msgs::PolynomialTrajectory4D &msg = last_traj_msg;
    msg.segments.clear();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "odom"

    std::vector<ompl::base::State *> &states = p_last_traj_ompl->getStates();
    size_t N = states.size();
    for (size_t i = 0; i<N; i++) {
        ompl::base::State *p_s = states[i];
        const double &x_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &y_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        double &z_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
        double &yaw_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[3];
        ROS_INFO_STREAM("states["<< i <<"], x_s: " << x_s << "; y_s: " << y_s << "; z_s: " << z_s << "; yaw_s: " << yaw_s);

        // Calculate clearance here and push it on a list
        clearence_list.push_back(sqrt((x_s-0.5)*(x_s-0.5) + (y_s-0.5)*(y_s-0.5) + (z_s-0.5)*(z_s-0.5)) - 0.25);

        mav_planning_msgs::PolynomialSegment4D segment;
        segment.header = msg.header;
        segment.num_coeffs = 0;
        segment.segment_time = ros::Duration(0.);
        
        segment.x.push_back(x_s);
        segment.y.push_back(y_s);
        segment.z.push_back(z_s);
        segment.yaw.push_back(yaw_s);
        msg.segments.push_back(segment);
    }

}

void OctomapPathPlanner::printRelevantInformation() {
    if (!traj_planning_successful) {
        ROS_INFO_STREAM(
        "Path planning was NOT successful."
        << "\n  path_calculation_time: " << path_calculation_time.count() << "ms"
        ); 
        return;
    }
    auto minimum_clearance = std::min_element(clearence_list.begin(), clearence_list.end());
    auto maximum_clearance = std::max_element(clearence_list.begin(), clearence_list.end());
    double average_clearance = calculateAverageClearance();
    
    const ompl::base::State* last_state = p_last_traj_ompl->getState(p_last_traj_ompl->getStateCount() - 1);
    double* my_values = last_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    bool point_reached = (
        my_values[0] == goal_position.x
        && my_values[1] == goal_position.y
        && my_values[2] == goal_position.z
    );
    
    ROS_INFO_STREAM(
        "Path planning was successful." 
        << "\n  path_calculation_time: " << path_calculation_time.count() << "ms"
        << "\n  point_reached: " << point_reached
        << "\n  length: " << path_length
        << "\n  cost: " << path_cost 
        << "\n  minimum_clearance: " << *minimum_clearance
        << "\n  maximum_clearance: " << *maximum_clearance
        << "\n  average_clearance: " << average_clearance
        );
}


double OctomapPathPlanner::calculateAverageClearance() {
    double sum = 0.0;
    for (const auto& value : clearence_list) {
        sum += value;
    }

    if (!clearence_list.empty()) {
        return  sum / clearence_list.size();
    } else {
        return 9999.99;
    }
}


void OctomapPathPlanner::plan()
{
    // Used to determine planing duration
    auto start_time = std::chrono::high_resolution_clock::now();

    // Construct the robot state space in which we're planning. We're
    // planning in [min, max]x[min, max]x[min, max], a subset of R^3.
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // Set the bounds of space to be in [min, max].
    // double a, b, c;
    // tree->getMetricMin(a,b,c);
    // double minimum_value = std::min({a, b, c});
    // std::cout<<"Metric min: "<<a<<","<<b<<","<<c<<std::endl;
    // tree->getMetricMax(a,b,c);
    // double maximum_value = std::max({a, b, c});
    // std::cout<<"Metric max: "<<a<<","<<b<<","<<c<<std::endl;
    // std::cout << minimum_value << " and max: " << maximum_value;

    space->setBounds(-100, 100); // TODO i guess we have to change those values?

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si, distmap));
    si->setMotionValidator(std::make_shared<LocalMotionValidator>(si, tree));
    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or [0, 0, 0], if not specified by the user.
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_position.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_position.y;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = current_position.z;

    // Set our robot's goal state to be the one entered by the user.
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_position.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_position.y;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_position.z;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    ob::PlannerStatus solved = ob::PlannerStatus::UNKNOWN;
    if (runTime > 0) {
        solved = optimizingPlanner->solve(runTime);
    } else {
        solved = optimizingPlanner->solve(ompl::base::IterationTerminationCondition(1000));
    }
    
    // Used to determine planing duration
    auto end_time = std::chrono::high_resolution_clock::now(); 

    if (solved)
    {
        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

        path_length = pdef->getSolutionPath()->length();
        path_cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        if (!outputFile.empty())
        {
            std::ofstream outFile(outputFile.c_str());
            std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                printAsMatrix(outFile);
            outFile.close();
        }

        p_last_traj_ompl =  std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
        traj_planning_successful = true;
    } else {
        std::cout << "No solution found." << std::endl;
        traj_planning_successful = false;
    }
    path_calculation_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
}
