//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_predicate.h"
#include "braking/safe_distance_predicate.h"
#include "braking/unnecessary_braking_predicate.h"
#include "general/lane_based_orientation_similar_predicate.h"
#include "general/orientation_towards_predicate.h"
#include "position/in_front_of_predicate.h"
#include "position/in_intersection_main_area_predicate.h"
#include "position/in_same_lane_predicate.h"
#include "position/in_single_lane_predicate.h"
#include "position/passes_stop_line_predicate.h"
#include "position/stop_line_in_front_predicate.h"
#include "regulatory/at_red_left_traffic_light_predicate.h"
#include "regulatory/at_red_right_traffic_light_predicate.h"
#include "regulatory/at_red_straight_traffic_light_predicate.h"
#include "regulatory/at_red_traffic_light_predicate.h"
#include "velocity/in_standstill_predicate.h"
#include "velocity/keeps_lane_speed_limit_predicate.h"
#include "velocity/preserves_traffic_flow_predicate.h"
#include "velocity/required_speed_predicate.h"
#include "velocity/slow_leading_vehicle_predicate.h"

bool CommonRoadPredicate::statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    statistics.numExecutions++;
    auto startTime{Timer::start()};
    bool result{booleanEvaluation(timeStep, world, obstacleK, obstacleP)};
    long compTime{evaluationTimer.stop(startTime)};
    statistics.totalComputationTime += compTime;
    if (compTime > statistics.maxComputationTime)
        statistics.maxComputationTime = compTime;
    if (compTime < statistics.minComputationTime)
        statistics.minComputationTime = compTime;
    if (result)
        statistics.numSatisfaction++;
    return result;
}

const PredicateParameters &CommonRoadPredicate::getParameters() const { return parameters; }

void CommonRoadPredicate::setParameters(const PredicateParameters &params) { parameters = params; }

CommonRoadPredicate::CommonRoadPredicate(const PredicateParameters &parameters, bool vehicleDependent)
    : parameters(parameters), vehicleDependent(vehicleDependent) {}

const PredicateStatistics &CommonRoadPredicate::getStatistics() const { return statistics; }

const Timer &CommonRoadPredicate::getEvaluationTimer() const { return evaluationTimer; }

CommonRoadPredicate::CommonRoadPredicate(bool vehicleDependent) : vehicleDependent(vehicleDependent) {}

bool CommonRoadPredicate::isVehicleDependent() const { return vehicleDependent; }

std::map<std::string, std::shared_ptr<CommonRoadPredicate>> predicates{
    {"keeps_safe_distance_prec", std::make_shared<SafeDistancePredicate>()},
    {"unnecessary_braking", std::make_shared<UnnecessaryBrakingPredicate>()},
    {"in_front_of", std::make_shared<InFrontOfPredicate>()},
    {"in_same_lane", std::make_shared<InSameLanePredicate>()},
    {"in_intersection_main_area", std::make_shared<InIntersectionMainAreaPredicate>()},
    {"keeps_lane_speed_limit", std::make_shared<KeepsLaneSpeedLimitPredicate>()},
    {"preserves_traffic_flow", std::make_shared<PreservesTrafficFlowPredicate>()},
    {"keeps_sign_min_speed_limit", std::make_shared<RequiredSpeedPredicate>()},
    {"slow_leading_vehicle", std::make_shared<SlowLeadingVehiclePredicate>()},
    {"at_red_traffic_light", std::make_shared<AtRedTrafficLightPredicate>()},
    {"at_red_straight_traffic_light", std::make_shared<AtRedStraightTrafficLightPredicate>()},
    {"at_red_left_traffic_light", std::make_shared<AtRedLeftTrafficLightPredicate>()},
    {"at_red_right_traffic_light", std::make_shared<AtRedRightTrafficLightPredicate>()},
    {"orientation_towards", std::make_shared<OrientationTowardsPredicate>()},
    {"lane_based_orientation_similar", std::make_shared<LaneBasedOrientationSimilarPredicate>()},
    {"in_single_lane", std::make_shared<InSingleLanePredicate>()},
    {"in_standstill", std::make_shared<InStandstillPredicate>()},
    {"stop_line_in_front", std::make_shared<StopLineInFrontPredicate>()},
    {"passes_stop_line", std::make_shared<PassesStopLinePredicate>()},
};
