//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_predicate.h"
#include "braking/brakes_stronger_predicate.h"
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
#include "position/main_carriageway_right_lane_predicate.h"
#include "position/on_access_ramp_predicate.h"
#include "position/on_exit_ramp_predicate.h"
#include "position/on_main_carriage_way_predicate.h"
#include "position/on_shoulder_predicate.h"
#include "position/on_urban_road_predicate.h"
#include "position/passes_stop_line_predicate.h"
#include "position/right_of_broad_lane_marking_predicate.h"
#include "position/stop_line_in_front_predicate.h"
#include "regulatory/at_red_left_traffic_light_predicate.h"
#include "regulatory/at_red_right_traffic_light_predicate.h"
#include "regulatory/at_red_straight_traffic_light_predicate.h"
#include "regulatory/at_red_traffic_light_predicate.h"
#include "regulatory/at_stop_sign_predicate.h"
#include "velocity/drives_faster_predicate.h"
#include "velocity/drives_with_slightly_higher_speed_predicate.h"
#include "velocity/exist_standing_leading_vehicle_predicate.h"
#include "velocity/in_standstill_predicate.h"
#include "velocity/keeps_braking_speed_limit_predicate.h"
#include "velocity/keeps_fov_speed_limit_predicate.h"
#include "velocity/keeps_lane_speed_limit_predicate.h"
#include "velocity/keeps_type_speed_limit_predicate.h"
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

    // TODO Thread-local storage for stats?
    omp_set_lock(&writelock);
    {
        statistics.numExecutions++;
        statistics.totalComputationTime += static_cast<unsigned long>(compTime);
        if (compTime > statistics.maxComputationTime)
            statistics.maxComputationTime = compTime;
        if (static_cast<unsigned long>(compTime) < statistics.minComputationTime)
            statistics.minComputationTime = static_cast<size_t>(compTime);
        if (result)
            statistics.numSatisfaction++;
        omp_unset_lock(&writelock);
    }
    return result;
}

const PredicateParameters &CommonRoadPredicate::getParameters() const { return parameters; }

void CommonRoadPredicate::setParameters(const PredicateParameters &params) { parameters = params; }

const PredicateStatistics &CommonRoadPredicate::getStatistics() const { return statistics; }

void CommonRoadPredicate::resetStatistics() {
    statistics.minComputationTime = LONG_MAX;
    statistics.maxComputationTime = LONG_MIN;
    statistics.totalComputationTime = 0;
    statistics.numSatisfaction = 0;
    statistics.numExecutions = 0;
}

const Timer &CommonRoadPredicate::getEvaluationTimer() const { return evaluationTimer; }

CommonRoadPredicate::CommonRoadPredicate(bool vehicleDependent) : vehicleDependent(vehicleDependent) {
    omp_init_lock(&writelock);
}

CommonRoadPredicate::~CommonRoadPredicate() { omp_destroy_lock(&writelock); }

bool CommonRoadPredicate::isVehicleDependent() const { return vehicleDependent; }

std::map<std::string, std::shared_ptr<CommonRoadPredicate>> predicates{
    {"brakes_stronger", std::make_shared<BrakesStrongerPredicate>()},
    {"keeps_safe_distance_prec", std::make_shared<SafeDistancePredicate>()},
    {"unnecessary_braking", std::make_shared<UnnecessaryBrakingPredicate>()},
    {"in_congestion", std::make_shared<InCongestionPredicate>()},
    {"in_queue_of_vehicles", std::make_shared<InQueueOfVehiclesPredicate>()},
    {"in_slow_moving_traffic", std::make_shared<InSlowMovingTrafficPredicate>()},
    {"interstate_broad_enough", std::make_shared<InterstateBroadEnoughPredicate>()},
    {"lane_based_orientation_similar", std::make_shared<LaneBasedOrientationSimilarPredicate>()},
    {"makes_u_turn", std::make_shared<MakesUTurnPredicate>()},
    {"orientation_towards", std::make_shared<OrientationTowardsPredicate>()},
    {"drives_leftmost", std::make_shared<DrivesLeftmostPredicate>()},
    {"drives_rightmost", std::make_shared<DrivesRightmostPredicate>()},
    {"in_front_of", std::make_shared<InFrontOfPredicate>()},
    {"in_intersection_main_area", std::make_shared<InIntersectionMainAreaPredicate>()},
    {"in_leftmost_lane", std::make_shared<InLeftmostLanePredicate>()},
    {"in_rightmost_lane", std::make_shared<InRightmostLanePredicate>()},
    {"in_same_lane", std::make_shared<InSameLanePredicate>()},
    {"in_single_lane", std::make_shared<InSingleLanePredicate>()},
    {"left_of_broad_lane_marking", std::make_shared<LeftOfBroadLaneMarkingPredicate>()},
    {"left_of", std::make_shared<LeftOfPredicate>()},
    {"main_carriageway_right_lane", std::make_shared<MainCarriagewayRightLanePredicate>()},
    {"on_access_ramp", std::make_shared<OnAccessRampPredicate>()},
    {"on_exit_ramp", std::make_shared<OnExitRampPredicate>()},
    {"on_main_carriage_way", std::make_shared<OnMainCarriageWayPredicate>()},
    {"on_shoulder", std::make_shared<OnShoulderPredicate>()},
    {"on_urban_road", std::make_shared<OnUrbanRoadPredicate>()},
    {"passes_stop_line", std::make_shared<PassesStopLinePredicate>()},
    {"right_of_broad_lane_marking", std::make_shared<RightOfBroadLaneMarkingPredicate>()},
    {"stop_line_in_front", std::make_shared<StopLineInFrontPredicate>()},
    {"at_red_traffic_light", std::make_shared<AtRedTrafficLightPredicate>()},
    {"at_red_straight_traffic_light", std::make_shared<AtRedStraightTrafficLightPredicate>()},
    {"at_red_left_traffic_light", std::make_shared<AtRedLeftTrafficLightPredicate>()},
    {"at_red_right_traffic_light", std::make_shared<AtRedRightTrafficLightPredicate>()},
    {"at_stop_sign", std::make_shared<AtStopSignPredicate>()},
    {"drives_faster", std::make_shared<DrivesFasterPredicate>()},
    {"drives_with_slightly_higher_speed", std::make_shared<DrivesWithSlightlyHigherSpeedPredicate>()},
    {"exist_standing_leading_vehicle", std::make_shared<ExistStandingLeadingVehiclePredicate>()},
    {"in_standstill", std::make_shared<InStandstillPredicate>()},
    {"keeps_braking_speed_limit", std::make_shared<KeepsBrakingSpeedLimitPredicate>()},
    {"keeps_fov_speed_limit", std::make_shared<KeepsFOVSpeedLimitPredicate>()},
    {"keeps_lane_speed_limit", std::make_shared<KeepsLaneSpeedLimitPredicate>()},
    {"keeps_type_speed_limit", std::make_shared<KeepsTypeSpeedLimitPredicate>()},
    {"preserves_traffic_flow", std::make_shared<PreservesTrafficFlowPredicate>()},
    {"keeps_sign_min_speed_limit", std::make_shared<RequiredSpeedPredicate>()},
    {"reverses", std::make_shared<ReversesPredicate>()},
    {"slow_leading_vehicle", std::make_shared<SlowLeadingVehiclePredicate>()},
};
