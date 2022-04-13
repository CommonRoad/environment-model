//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/commonroad_container.h"
#include <pybind11/pybind11.h>

#include "commonroad_cpp/predicates/braking/safe_distance_predicate.h"
#include "commonroad_cpp/predicates/braking/unnecessary_braking_predicate.h"
#include "commonroad_cpp/predicates/general/in_congestion_predicate.h"
#include "commonroad_cpp/predicates/general/in_queue_of_vehicles_predicate.h"
#include "commonroad_cpp/predicates/general/in_slow_moving_traffic_predicate.h"
#include "commonroad_cpp/predicates/general/interstate_broad_enough_predicate.h"
#include "commonroad_cpp/predicates/general/lane_based_orientation_similar_predicate.h"
#include "commonroad_cpp/predicates/general/makes_u_turn_predicate.h"
#include "commonroad_cpp/predicates/general/orientation_towards_predicate.h"
#include "commonroad_cpp/predicates/position/drives_leftmost_predicate.h"
#include "commonroad_cpp/predicates/position/drives_rightmost_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_intersection_main_area_predicate.h"
#include "commonroad_cpp/predicates/position/in_leftmost_lane_predicate.h"
#include "commonroad_cpp/predicates/position/in_rightmost_lane_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "commonroad_cpp/predicates/position/in_single_lane_predicate.h"
#include "commonroad_cpp/predicates/position/left_of_broad_lane_marking_predicate.h"
#include "commonroad_cpp/predicates/position/left_of_predicate.h"
#include "commonroad_cpp/predicates/position/on_lanelet_with_type_predicate.h"
#include "commonroad_cpp/predicates/position/right_of_broad_lane_marking_predicate.h"
#include "commonroad_cpp/predicates/regulatory/at_red_left_traffic_light_predicate.h"
#include "commonroad_cpp/predicates/regulatory/at_red_right_traffic_light_predicate.h"
#include "commonroad_cpp/predicates/regulatory/at_red_straight_traffic_light_predicate.h"
#include "commonroad_cpp/predicates/regulatory/at_red_traffic_light_predicate.h"
#include "commonroad_cpp/predicates/velocity/drives_faster_predicate.h"
#include "commonroad_cpp/predicates/velocity/drives_with_slightly_higher_speed_predicate.h"
#include "commonroad_cpp/predicates/velocity/exist_standing_leading_vehicle_predicate.h"
#include "commonroad_cpp/predicates/velocity/in_standstill_predicate.h"
#include "commonroad_cpp/predicates/velocity/reverses_predicate.h"

namespace py = pybind11;

void py_registerScenario(size_t scenarioId, size_t timeStep, double dt, const std::string &country,
                         const py::handle &py_laneletNetwork, const py::list &py_egoVehicles,
                         const py::list &py_obstacles);

void py_removeScenario(size_t scenarioId);

template <typename T>
bool py_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId);

template <typename T>
bool py_boolean_single_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId = 0);

template <typename T>
double py_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId);

template <typename T> double py_robust_single_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                                         size_t py_obstacleId = 0);

bool py_safe_distance_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                         double velocityP, double minAccelerationK,
                                                         double minAccelerationP, double tReact, double lengthK,
                                                         double lengthP);

double py_safe_distance_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                          double velocityP, double minAccelerationK,
                                                          double minAccelerationP, double tReact, double lengthK,
                                                          double lengthP);

double py_safe_distance(double velocityK, double velocityP, double minAccelerationK, double minAccelerationP,
                        double tReact);

bool py_in_front_of_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK, double lengthP);

double py_in_front_of_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK, double lengthP);

PYBIND11_MODULE(crcpp, m) {
    m.doc() = "CommonRoad Python/C++ Interface";
    m.def("register_scenario", &py_registerScenario, "Add new scenario to C++ environment model", py::arg("scenarioId"),
          py::arg("timeStep"), py::arg("dt"), py::arg("country"), py::arg("py_lanelets"), py::arg("py_egoVehicles"),
          py::arg("py_obstacles"));

    m.def("remove_scenario", &py_removeScenario, "Remove scenario to C++ environment model", py::arg("scenarioId"));

    m.def("safe_distance_boolean_evaluation", &py_boolean_evaluation<SafeDistancePredicate>,
          "Boolean evaluation of safe distance predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("safe_distance_boolean_evaluation", &py_safe_distance_boolean_evaluation_with_parameters,
          "Boolean evaluation of safe distance predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("velocityK"), py::arg("velocityP"), py::arg("minAccelerationK"),
          py::arg("minAccelerationP"), py::arg("tReact"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("safe_distance_robust_evaluation", &py_robust_evaluation<SafeDistancePredicate>,
          "Robust evaluation of safe distance predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("safe_distance_robust_evaluation", &py_safe_distance_robust_evaluation_with_parameters,
          "Robust evaluation of safe distance predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("velocityK"), py::arg("velocityP"), py::arg("minAccelerationK"),
          py::arg("minAccelerationP"), py::arg("tReact"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("safe_distance", &py_safe_distance, "Calculation of safe distance", py::arg("velocityK"),
          py::arg("velocityP"), py::arg("minAccelerationK"), py::arg("minAccelerationP"), py::arg("tReact"));

    m.def("in_front_of_boolean_evaluation", &py_boolean_evaluation<InFrontOfPredicate>,
          "Boolean evaluation of in front of predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("in_front_of_boolean_evaluation", &py_in_front_of_boolean_evaluation_with_parameters,
          "Boolean evaluation of in front of predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("in_front_of_robust_evaluation", &py_robust_evaluation<InFrontOfPredicate>,
          "Robust evaluation of in front of predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("in_front_of_robust_evaluation", &py_in_front_of_robust_evaluation_with_parameters,
          "Robust evaluation of in front of predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("in_same_lane_boolean_evaluation", &py_boolean_evaluation<InSameLanePredicate>,
          "Boolean evaluation of in same lane predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("in_single_lane_boolean_evaluation", &py_boolean_single_evaluation<InSingleLanePredicate>,
          "Boolean evaluation of in single lane predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);
    m.def("unnecessary_braking_boolean_evaluation", &py_boolean_single_evaluation<UnnecessaryBrakingPredicate>,
          "Boolean evaluation of unnecessary braking predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("unnecessary_braking_robust_evaluation", &py_robust_single_evaluation<UnnecessaryBrakingPredicate>,
          "Robust evaluation of unnecessary braking predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("in_congestion_boolean_evaluation", &py_boolean_evaluation<InCongestionPredicate>,
          "Boolean evaluation of in congestion predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("exist_standing_leading_vehicle_evaluation", &py_boolean_evaluation<ExistStandingLeadingVehiclePredicate>,
          "Boolean evaluation of exist standing leading vehicle predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("in_standstill_evaluation", &py_boolean_evaluation<InStandstillPredicate>,
          "Boolean evaluation of in standstill predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("left_of_evaluation", &py_boolean_evaluation<LeftOfPredicate>, "Boolean evaluation of left of predicate",
          py::arg("scenarioId"), py::arg("time_step"), py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("drives_faster_evaluation", &py_boolean_evaluation<DrivesFasterPredicate>,
          "Boolean evaluation of drives faster predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("in_slow_moving_traffic_evaluation", &py_boolean_evaluation<InSlowMovingTrafficPredicate>,
          "Boolean evaluation of in slow moving traffic predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("in_queue_of_vehicles_evaluation", &py_boolean_evaluation<InQueueOfVehiclesPredicate>,
          "Boolean evaluation of in queue of vehicles predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("drives_with_slightly_higher_speed_evaluation",
          &py_boolean_evaluation<DrivesWithSlightlyHigherSpeedPredicate>,
          "Boolean evaluation of drives with slightly higher speed predicate", py::arg("scenarioId"),
          py::arg("time_step"), py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("right_of_broad_lane_marking_evaluation", &py_boolean_evaluation<RightOfBroadLaneMarkingPredicate>,
          "Boolean evaluation of right of broad lane marking predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("left_of_broad_lane_marking_evaluation", &py_boolean_evaluation<LeftOfBroadLaneMarkingPredicate>,
          "Boolean evaluation of left of broad lane marking predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("makes_u_turn_evaluation", &py_boolean_evaluation<MakesUTurnPredicate>,
          "Boolean evaluation of makes u turn predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("reverses_evaluation", &py_boolean_evaluation<ReversesPredicate>, "Boolean evaluation of reverses predicate",
          py::arg("scenarioId"), py::arg("time_step"), py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("interstate_broad_enough_evaluation", &py_boolean_evaluation<InterstateBroadEnoughPredicate>,
          "Boolean evaluation of interstate broad enough predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("in_leftmost_lane_evaluation", &py_boolean_evaluation<InLeftmostLanePredicate>,
          "Boolean evaluation of in leftmost lane predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("in_rightmost_lane_evaluation", &py_boolean_evaluation<InRightmostLanePredicate>,
          "Boolean evaluation of in rightmost lane predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("drives_leftmost_evaluation", &py_boolean_evaluation<DrivesLeftmostPredicate>,
          "Boolean evaluation of drives leftmost predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("drives_rightmost_evaluation", &py_boolean_evaluation<DrivesRightmostPredicate>,
          "Boolean evaluation of drives rightmost predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("in_single_lane_evaluation", &py_boolean_evaluation<InSingleLanePredicate>,
          "Boolean evaluation of in single lane predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("orientation_towards_boolean_evaluation", &py_boolean_evaluation<OrientationTowardsPredicate>,
          "Boolean evaluation of orientation towards predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("lane_based_orientation_similar_boolean_evaluation",
          &py_boolean_evaluation<LaneBasedOrientationSimilarPredicate>,
          "Boolean evaluation of orientation towards predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("in_intersection_main_area_boolean_evaluation",
          &py_boolean_single_evaluation<InIntersectionMainAreaPredicate>,
          "Boolean evaluation of intersection-main-area predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("at_red_right_traffic_light_boolean_evaluation",
          &py_boolean_single_evaluation<AtRedRightTrafficLightPredicate>,
          "Boolean evaluation of at red right traffic light predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("at_red_straight_traffic_light_boolean_evaluation",
          &py_boolean_single_evaluation<AtRedStraightTrafficLightPredicate>,
          "Boolean evaluation of at red straight traffic light predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("at_red_traffic_light_boolean_evaluation", &py_boolean_single_evaluation<AtRedTrafficLightPredicate>,
          "Boolean evaluation of at red traffic light predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);

    m.def("at_red_left_traffic_light_boolean_evaluation", &py_boolean_single_evaluation<AtRedLeftTrafficLightPredicate>,
          "Boolean evaluation of at red left traffic light predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 0);
}