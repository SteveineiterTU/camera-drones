/*
 * Octomap Path Planner ROS - octomap_path_planner_ros.cpp
 *
 *  Authors: Stefan Schorkmeier <s.schoerkmeier@student.tugraz.at>
 *           Florian Werkl <florian.werkl@student.tugraz.at>
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
    traj_planning_successful(false),
    dt_(0.01),
    current_sample_time_(0.0) { 
    
    pn.param("dt", dt_, dt_);

    // Parse the arguments, returns true if successful, false otherwise
    if (argParse(argc, argv, &runTime, &optimizingPlannerMaxIterations, &plannerType, &objectiveType, &outputFile, &octomapFile))
    {
        // Return with success
        ROS_INFO("OctomapPathPlanner::OctomapPathPlanner(...) argParse success!");
    } else {
        // Return with error - Modified argParse to make this equivalent to giving no arguments.
        ROS_INFO_STREAM("OctomapPathPlanner::OctomapPathPlanner(...) argParse error!"
        << "\n  ===================================="
        << "\n  Please EXIT the terminal (ctrl + C)."
        << "\n  ====================================");
        return; 
    }

    // ROS topics
    current_position_sub = pnode_.subscribe("current_position", 10, &OctomapPathPlanner::currentPositionCallback, this);
    goal_position_sub = pnode_.subscribe("goal_position", 10, &OctomapPathPlanner::goalPositionCallback, this);
    trajectory_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory", 1);

  smooth_trajectory4d_sub = pnode_.subscribe<mav_planning_msgs::PolynomialTrajectory4D>(
    "smooth_trajectory4d", 10,
    &OctomapPathPlanner::smoothTrajectory4DCallback, this);
    const bool oneshot = false;
    const bool autostart = false;
    publish_timer_ = node_.createTimer(ros::Duration(dt_),
                                   &OctomapPathPlanner::commandTimerCallback,
                                   this, oneshot, autostart);
    command_pub_ = pnode_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);

    current_position.x = 0.; current_position.y = 0.; current_position.z = 0.;
    goal_position.x = 1.; goal_position.y = 1.; goal_position.z = 1.;

    tree = new octomap::OcTree(0.05);
    tree->readBinary(octomapFile);
    ROS_INFO_STREAM("read in tree, " << tree->getNumLeafNodes() << " leaves ");

    // Setting up the dist map. I assume we have a static octomap aka we do not have to change this map. 
    // Also by having this in the clearance function for example it resulted in weird errors aka i could
    // not calculate any path (eg from 0 0 0 to 1 0 0)
    double x, y, z;
    tree->getMetricMin(x, y, z);
    octomap::point3d min(x, y, z);
    tree->getMetricMax(x, y, z);
    octomap::point3d max(x, y, z);

    bool unknownAsOccupied = false;
    float maxDist = 50.0;
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    // The constructor copies data but does not yet compute the distance map
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

    plan(goal_position);

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
    
std::string additional_info = point_reached ? "" : " Altough we have NOT reached the given point.";
    ROS_INFO_STREAM(
        "Path planning was successful." << additional_info 
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


void OctomapPathPlanner::plan(const geometry_msgs::Point &goal_pos)
{
    // Used to determine planing duration
    auto start_time = std::chrono::high_resolution_clock::now();

    // Construct the robot state space in which we're planning. We're
    // planning in (TODO CHANGE as we change the part below) [min, max]x[min, max]x[min, max], a subset of R^3.
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // Set the bounds of space to be in [min, max] of the passed octomap.
    ompl::base::RealVectorBounds bounds(3);
    double x, y, z;

    tree->getMetricMin(x, y, z);
    // std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    bounds.setLow(0, x);
    bounds.setLow(1, y);
    bounds.setLow(2, z);
    
    tree->getMetricMax(x, y, z);
    // std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;
    bounds.setHigh(0, x);
    bounds.setHigh(1, y);
    bounds.setHigh(2, z);

    space->setBounds(bounds);


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
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_pos.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_pos.y;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_pos.z;

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
    // One of those two variables has to be set.
    if (runTime > 0) {
        solved = optimizingPlanner->solve(runTime);
    } 
    else if (optimizingPlannerMaxIterations > 0){
        solved = optimizingPlanner->solve(ompl::base::IterationTerminationCondition(optimizingPlannerMaxIterations));
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

        simplifyPath(p_last_traj_ompl,20);

        traj_planning_successful = true;
    } else {
        std::cout << "No solution found." << std::endl;
        traj_planning_successful = false;
    }
    path_calculation_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
}

void OctomapPathPlanner::smoothTrajectory4DCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr& p_msg) {
  ROS_INFO("BEGIN: smoothTrajectory4DCallback(...)");
  const mav_planning_msgs::PolynomialTrajectory4D &msg = *p_msg;

  if (msg.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu waypoints", msg.segments.size());
  }

  ROS_INFO("Received trajectory print-out...");
  std::cout << "header:" << msg.header << std::endl;
  { // print-out trajectory, I do this because rostopic echo breaks when receiving a message of this type
    int i = 0;
    for (auto it=msg.segments.begin(); it!=msg.segments.end(); it++) {
      std::cout <<
        "segment["<<i<<"]"<<
        *it << std::endl;
      i++;
    }
  }

  bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory( msg, &trajectory_);
  if (!success) {
    return;
  }
  processTrajectory();

  ROS_INFO("END:   smoothTrajectory4DCallback(...)");
}

void OctomapPathPlanner::processTrajectory() {
  publish_timer_.stop();
  publish_timer_.start();
  current_sample_time_ = 0.0;
  start_time_ = ros::Time::now();
}

void OctomapPathPlanner::commandTimerCallback(const ros::TimerEvent&) {
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
    if (!success) {
      publish_timer_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    ros::Time current_ideal_time_ = start_time_ + ros::Duration(current_sample_time_);
    msg.header.frame_id = "world";
    std::string child_frame_id("state_ref");
    msg.header.stamp = current_ideal_time_;
    command_pub_.publish(msg);
    current_sample_time_ += dt_;

    {
      tf::Transform transform;
      const geometry_msgs::Vector3 &t = msg.points[0].transforms[0].translation;
      const geometry_msgs::Quaternion &q = msg.points[0].transforms[0].rotation;
      transform.setOrigin(tf::Vector3( t.x, t.y, t.z));
      transform.setRotation(tf::Quaternion( q.x, q.y, q.z, q.w));
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(
              transform, current_ideal_time_,
              msg.header.frame_id, child_frame_id));
    }

  } else {
    publish_timer_.stop();
  }
}

void OctomapPathPlanner::simplifyPath(std::shared_ptr<ompl::geometric::PathGeometric> path, int runs=20)
{
    ROS_INFO_STREAM(
    "Simplification Started" 
    );
    std::cout << "Simplification started." << std::endl;
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si_));
//    auto simplified = std::make_shared<ompl::geometric::PathGeometric>(ompl::geometric::PathGeometric(*path));
    *p_simplified = *path;

    ompl::geometric::PathSimplifier simplifier(si_, ompl::base::GoalPtr(), obj);
    
    double avg_costs = 0.0;
    ompl::base::Cost original_cost = path->cost(obj);
    for (int i = 0; i < runs; i++)
    {
        simplifier.shortcutPath(*p_simplified, 100, 100, 0.33, 0.005);
        avg_costs += p_simplified->cost(obj).value();
    }

    avg_costs /= runs;
    printf("Average cost: %f, original cost: %f\n", avg_costs, original_cost.value());

}