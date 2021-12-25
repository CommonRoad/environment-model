//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_predicate.h"
#include "braking/safe_distance_predicate.h"
#include "braking/unnecessary_braking_predicate.h"
#include "general/in_congestion_predicate.h"
#include "general/in_queue_of_vehicles_predicate.h"
#include "general/in_slow_moving_traffic_predicate.h"
#include "general/interstate_broad_enough_predicate.h"
#include "general/lane_based_orientation_similar_predicate.h"
#include "general/makes_u_turn_predicate.h"
#include "general/orientation_towards_predicate.h"
#include "position/drives_leftmost_predicate.h"
#include "position/drives_rightmost_predicate.h"
#include "position/in_front_of_predicate.h"
#include "position/in_intersection_main_area_predicate.h"
#include "position/in_leftmost_lane_predicate.h"
#include "position/in_rightmost_lane_predicate.h"
#include "position/in_same_lane_predicate.h"
#include "position/in_single_lane_predicate.h"
#include "position/left_of_broad_lane_marking_predicate.h"
#include "position/left_of_predicate.h"
#include "position/on_access_ramp_predicate.h"
#include "position/on_main_carriage_way_predicate.h"
#include "position/on_shoulder_predicate.h"
#include "position/passes_stop_line_predicate.h"
#include "position/right_of_broad_lane_marking_predicate.h"
#include "position/stop_line_in_front_predicate.h"
#include "regulatory/at_red_left_traffic_light_predicate.h"
#include "regulatory/at_red_right_traffic_light_predicate.h"
#include "regulatory/at_red_straight_traffic_light_predicate.h"
#include "regulatory/at_red_traffic_light_predicate.h"
#include "velocity/drives_faster_predicate.h"
#include "velocity/drives_with_slightly_higher_speed_predicate.h"
#include "velocity/exist_standing_leading_vehicle_predicate.h"
#include "velocity/in_standstill_predicate.h"
#include "velocity/keeps_lane_speed_limit_predicate.h"
#include "velocity/preserves_traffic_flow_predicate.h"
#include "velocity/required_speed_predicate.h"
#include "velocity/reverses_predicate.h"
#include "velocity/slow_leading_vehicle_predicate.h"

bool CommonRoadPredicate::statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    auto startTime{Timer::start()};
    bool result{booleanEvaluation(timeStep, world, obstacleK, obstacleP)};
    long compTime{evaluationTimer.stop(startTime)};
#pragma omp critical(statistics)
    {
        statistics.numExecutions++;
        statistics.totalComputationTime += compTime;
        if (compTime > statistics.maxComputationTime)
            statistics.maxComputationTime = compTime;
        if (compTime < statistics.minComputationTime)
            statistics.minComputationTime = compTime;
        if (result)
            statistics.numSatisfaction++;
    }
    return result;
}

const PredicateParameters &CommonRoadPredicate::getParameters() const { return parameters; }

void CommonRoadPredicate::setParameters(const PredicateParameters &params) { parameters = params; }

CommonRoadPredicate::CommonRoadPredicate(const PredicateParameters &parameters, bool vehicleDependent)
    : parameters(parameters), vehicleDependent(vehicleDependent) {}

const PredicateStatistics &CommonRoadPredicate::getStatistics() const { return statistics; }

void CommonRoadPredicate::resetStatistics() {
    statistics.minComputationTime = LONG_MAX;
    statistics.maxComputationTime = LONG_MIN;
    statistics.totalComputationTime = 0;
    statistics.numSatisfaction = 0;
    statistics.numExecutions = 0;
}

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
    {"in_congestion", std::make_shared<InCongestionPredicate>()},
    {"exist_standing_leading_vehicle", std::make_shared<ExistStandingLeadingVehiclePredicate>()},
    {"left_of", std::make_shared<LeftOfPredicate>()},
    {"drives_faster", std::make_shared<DrivesFasterPredicate>()},
    {"in_slow_moving_traffic", std::make_shared<InSlowMovingTrafficPredicate>()},
    {"in_queue_of_vehicles", std::make_shared<InQueueOfVehiclesPredicate>()},
    {"drives_with_slightly_higher_speed", std::make_shared<DrivesWithSlightlyHigherSpeedPredicate>()},
    {"right_of_broad_lane_marking", std::make_shared<RightOfBroadLaneMarkingPredicate>()},
    {"left_of_broad_lane_marking", std::make_shared<LeftOfBroadLaneMarkingPredicate>()},
    {"on_access_ramp", std::make_shared<OnAccessRampPredicate>()},
    {"makes_u_turn", std::make_shared<MakesUTurnPredicate>()},
    {"reverses", std::make_shared<ReversesPredicate>()},
    {"interstate_broad_enough", std::make_shared<InterstateBroadEnoughPredicate>()},
    {"on_shoulder", std::make_shared<OnShoulderPredicate>()},
    {"in_leftmost_lane", std::make_shared<InLeftmostLanePredicate>()},
    {"in_rightmost_lane", std::make_shared<InRightmostLanePredicate>()},
    {"drives_leftmost", std::make_shared<DrivesLeftmostPredicate>()},
    {"drives_rightmost", std::make_shared<DrivesRightmostPredicate>()},
    {"on_main_carriage_way", std::make_shared<OnMainCarriageWayPredicate>()}};
