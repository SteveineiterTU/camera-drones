/*
 * Octomap Path Planner ROS - octomap_helper_functions.h
 *
 *  Author: Stefan Schorkmeier <s.schoerkmeier@student.tugraz.at>
 *  Created on: November 25, 2023
 *
 */


#ifndef OCTOMAP_PATH_PLANNER_HELPER_FUNCTIONS_H_
#define OCTOMAP_PATH_PLANNER_HELPER_FUNCTIONS_H_


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
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
#include <dynamicEDT3D/dynamicEDTOctomap.h>



namespace ob = ompl::base;
namespace og = ompl::geometric;

// An enum of supported optimal planners, alphabetical order
enum optimalPlanner
{
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
    OBJECTIVE_PATHCLEARANCE,
    OBJECTIVE_PATHLENGTH,
    OBJECTIVE_THRESHOLDPATHLENGTH,
    OBJECTIVE_WEIGHTEDCOMBO
};

// Parse the command-line arguments
bool argParse(int argc, char** argv, double *runTimePtr, int *optimizingPlannerMaxIterationsPtr, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string *outputFilePtr, std::string *octomap);


class ValidityChecker : public ob::StateValidityChecker
{
public:
    DynamicEDTOctomap *_distmap;

    ValidityChecker(const ob::SpaceInformationPtr& si, DynamicEDTOctomap* distmap) :
        ob::StateValidityChecker(si),  
        _distmap(distmap) {}


    bool isValid(const ob::State* state) const override
    {
        // return this->clearance(state) > 2.0;
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to octomap things.
    double clearance(const ob::State* state) const override
    {
        const auto* _state =
            state->as<ob::RealVectorStateSpace::StateType>();

        double x = _state->values[0];
        double y = _state->values[1];
        double z = _state->values[2];

        octomap::point3d p(x, y, z);
        octomap::point3d closestObst;
        float distance;

        _distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
        // std::cout << "[DEBUG STUDENT] distance: " << distance << "\n";
        // std::cout << "[DEBUG STUDENT] x: " << x << " y: " << y << " z: " << z << "\n";
        return distance != -1 ? distance : 100; 
    }
};

// Other approach with keys: 
// 1. call tree.computeRayKeys(origin, end, keys) where origin is our current state and end is the closest object i guess? 
// 2. After that we can call tree.search(keys) to check if something exists on this ray i guess.
// 3. If something exsits on ray return false, else true.
class LocalMotionValidator : public ob::MotionValidator
{
public:
    octomap::OcTree *_tree;

    LocalMotionValidator(const ob::SpaceInformationPtr& si, octomap::OcTree* tree) : 
        ob::MotionValidator(si),
        _tree(tree) {}


    bool checkMotion(const ob::State *s1, const ob::State *s2) const override {
        return motionClearance(s1, s2);            
    }

     bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override {
        return motionClearance(s1, s2);
    }

    bool motionClearance(const ob::State *s1, const ob::State *s2) const {
            const auto* state_1 = s1->as<ob::RealVectorStateSpace::StateType>();
            const auto* state_2 = s2->as<ob::RealVectorStateSpace::StateType>();

            octomap::point3d start(state_1->values[0], state_1->values[1], state_1->values[2]);
            octomap::point3d end(state_2->values[0], state_2->values[1], state_2->values[2]);
            octomap::point3d direction(
                state_2->values[0] - state_1->values[0], 
                state_2->values[1] - state_1->values[1], 
                state_2->values[2] - state_1->values[2]
            );
                        octomap::point3d last_cell;
            bool has_ray_hit_an_occupied_cell = _tree->castRay(start, direction, last_cell, false, 0.1); // TODO ponder if maxRange should be changed tho
            
            // std::cout << "[DEBUG STUDENT] start: " << start << " end: " << end << " direction: " << direction << "\n";
            // std::cout << "[DEBUG STUDENT] Ray return value: " << foo << "\n";
            
            // has_ray_hit_an_occupied_cell = True if somehting has been hit. Therefore we need to negate it.
            return !has_ray_hit_an_occupied_cell;
    }
};

class PathLengthOptimizationObjectiveZPenalized : public ompl::base::OptimizationObjective
{
public:
    PathLengthOptimizationObjectiveZPenalized(const ompl::base::SpaceInformationPtr &si) : 
        ompl::base::OptimizationObjective(si) {
        description_ = "Path Length - Z Penalized";
        // Setup a default cost-to-go heuristics:
        setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
    }

    ompl::base::Cost stateCost(const ompl::base::State *s) const override {
        return identityCost();
    }

    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override {
        const auto* states1 = s1->as<ob::RealVectorStateSpace::StateType>();
        const auto* states2 = s2->as<ob::RealVectorStateSpace::StateType>();

        double dx = (states1->values[0] - states2->values[0]) * (states1->values[0] - states2->values[0]);
        double dy = (states1->values[1] - states2->values[1]) * (states1->values[1] - states2->values[1]);
        double dz = (states1->values[2] - states2->values[2]) * (states1->values[2] - states2->values[2]);
        double distance = sqrt(dx + dy + 100*dz);  

        return ompl::base::Cost(distance);
        // return ompl::base::Cost(si_->distance(s1, s2));
    }

    ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const override  {
        return motionCost(s1, s2);
    }

    ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
                                                unsigned int maxNumberCalls) const override
    {
        // Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is
        // available, if not a rejection-based technique can be used
        return std::make_shared<ompl::base::PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
		case PLANNER_BFMTSTAR:
        {
            return std::make_shared<og::BFMT>(si);
            break;
        }
        case PLANNER_BITSTAR:
        {
            return std::make_shared<og::BITstar>(si);
            break;
        }
        case PLANNER_CFOREST:
        {
            return std::make_shared<og::CForest>(si);
            break;
        }
        case PLANNER_FMTSTAR:
        {
            return std::make_shared<og::FMT>(si);
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            return std::make_shared<og::InformedRRTstar>(si);
            break;
        }
        case PLANNER_PRMSTAR:
        {
            return std::make_shared<og::PRMstar>(si);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return std::make_shared<og::RRTstar>(si);
            break;
        }
        case PLANNER_SORRTSTAR:
        {
            return std::make_shared<og::SORRTstar>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        case OBJECTIVE_PATHCLEARANCE:
            return getClearanceObjective(si);
            break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        case OBJECTIVE_THRESHOLDPATHLENGTH:
            return getThresholdPathLengthObj(si);
            break;
        case OBJECTIVE_WEIGHTEDCOMBO:
            return getBalancedObjective2(si);
            break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}


/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

/** Returns an optimization objective which attempts to minimize path
    length that is satisfied when a path of length shorter than 1.51
    is found. */
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

/** Defines an optimization objective which attempts to steer the
    robot away from obstacles. To formulate this objective as a
    minimization of path cost, we can define the cost of a path as a
    summation of the costs of each of the states along the path, where
    each state cost is a function of that state's clearance from
    obstacles.

    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
            std::numeric_limits<double>::min()));
    }
};

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);
}

/** Create an optimization objective which attempts to optimize both
    path length and clearance. We do this by defining our individual
    objectives, then adding them to a MultiOptimizationObjective
    object. This results in an optimization objective where path cost
    is equivalent to adding up each of the individual objectives' path
    costs.

    When adding objectives, we can also optionally specify each
    objective's weighting factor to signify how important it is in
    optimal planning. If no weight is specified, the weight defaults to
    1.0.
*/
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    // opt->addObjective(lengthObj, 10.0);
    // opt->addObjective(clearObj, 1.0);
    opt->addObjective(lengthObj, 1.0);
    opt->addObjective(clearObj, 10.0);

    return ob::OptimizationObjectivePtr(opt);
}

/** Create an optimization objective equivalent to the one returned by
    getBalancedObjective1(), but use an alternate syntax.
 */
ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));

    //return 10.0*lengthObj + clearObj;
    return lengthObj + 10.0*clearObj;
}

/** Create an optimization objective for minimizing path length, and
    specify a cost-to-go heuristic suitable for this optimal planning
    problem. */
ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

/** Parse the command line arguments into a string for an output file and the planner/optimization types */
bool argParse(int argc, char** argv, double* runTimePtr, int* optimizingPlannerMaxIterationsPtr, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string *outputFilePtr, std::string *octomapFile)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("octomap", bpo::value<std::string>()->required(), "Specify the path/filename of the octomap to use.")
        ("runtime,t", bpo::value<double>()->default_value(1.5), "(Optional) Specify the runtime in seconds. Must be greater than 0. Default value 1.5 seconds.")
        ("maxiterations,m", bpo::value<int>()->default_value(0), "(Optional) Specify the maximal iterations for early stopping. Must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are BFMTstar, BITstar, CForest, FMTstar, InformedRRTstar, PRMstar, RRTstar, and SORRTstar.") //Alphabetical order
        ("objective,o", bpo::value<std::string>()->default_value("PathLength"), "(Optional) Specify the optimization objective, defaults to PathLength if not given. Valid options are PathClearance, PathLength, ThresholdPathLength, and WeightedLengthAndClearanceCombo.") //Alphabetical order
        ("file,f", bpo::value<std::string>()->default_value(""), "(Optional) Specify an output path for the found solution path.")
        ("info,i", bpo::value<unsigned int>()->default_value(0u), "(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to WARN.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        // return false; // JP: Commented to achieve default behavior
    }

    // Set the log-level
    unsigned int logLevel = vm["info"].as<unsigned int>();

    // Switch to setting the log level:
    if (logLevel == 0u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    }
    else if (logLevel == 1u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    }
    else if (logLevel == 2u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    }
    else
    {
        std::cout << "Invalid log-level integer." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the runtime as a double
    *runTimePtr = vm["runtime"].as<double>();
    *optimizingPlannerMaxIterationsPtr = vm["maxiterations"].as<int>();
    if (*runTimePtr == 1.5 && *optimizingPlannerMaxIterationsPtr > 0) {
        *runTimePtr = 0.0;
    }

    // Sanity check
    if ( !((*runTimePtr > 0.0) ^ (*optimizingPlannerMaxIterationsPtr > 0)) )
    {
        std::cout << "Invalid runtime or maxiterations. Only ONE of those values has to be greater then 0." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("BFMTstar", plannerStr))
    {
        *plannerPtr = PLANNER_BFMTSTAR;
    }
    else if (boost::iequals("BITstar", plannerStr))
    {
        *plannerPtr = PLANNER_BITSTAR;
    }
    else if (boost::iequals("CForest", plannerStr))
    {
        *plannerPtr = PLANNER_CFOREST;
    }
    else if (boost::iequals("FMTstar", plannerStr))
    {
        *plannerPtr = PLANNER_FMTSTAR;
    }
    else if (boost::iequals("InformedRRTstar", plannerStr))
    {
        *plannerPtr = PLANNER_INF_RRTSTAR;
    }
    else if (boost::iequals("PRMstar", plannerStr))
    {
        *plannerPtr = PLANNER_PRMSTAR;
    }
    else if (boost::iequals("RRTstar", plannerStr))
    {
        *plannerPtr = PLANNER_RRTSTAR;
    }
    else if (boost::iequals("SORRTstar", plannerStr))
    {
        *plannerPtr = PLANNER_SORRTSTAR;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified objective as a string
    std::string objectiveStr = vm["objective"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("PathClearance", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHCLEARANCE;
    }
    else if (boost::iequals("PathLength", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHLENGTH;
    }
    else if (boost::iequals("ThresholdPathLength", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_THRESHOLDPATHLENGTH;
    }
    else if (boost::iequals("WeightedLengthAndClearanceCombo", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_WEIGHTEDCOMBO;
    }
    else
    {
        std::cout << "Invalid objective string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the output file string and store it in the return pointer
    *outputFilePtr = vm["file"].as<std::string>();

    // Get the boolean if we have a 3d or 2d calculation and store it in the return pointer.
    *octomapFile = vm["octomap"].as<std::string>();
    // Looks like we parsed the arguments successfully
    return true;
}

#endif // OCTOMAP_PATH_PLANNER_HELPER_FUNCTIONS_H_
