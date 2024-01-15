//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/in_same_lane_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/in_same_dir_predicate.h>

bool InSameDirPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    // if in same lane vehicles have the same dir
    InSameLanePredicate inSameLanePredicate = InSameLanePredicate();
    if (inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP)) {
        return true;
    }
    // check if they have same orientation
    const auto orientation_p{obstacleP->getCurvilinearOrientation(world->getRoadNetwork(), timeStep)};
    const auto orientation_k{
        obstacleK->getCurvilinearOrientation(timeStep, obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep))};
    const auto global_ori_p{obstacleP->getStateByTimeStep(timeStep)->getGlobalOrientation()};
    const auto global_ori_k{obstacleK->getStateByTimeStep(timeStep)->getGlobalOrientation()};

    if (!(std::abs(global_ori_k - global_ori_p) < 0.3) or !(std::abs(orientation_k - orientation_p) < 0.3)) {
        return false;
    }
    // now check it the two lanes are next to each other
    const auto lanes_p{obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep)};
    const auto lanelets_k{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};

    for (const auto &lanelet : lanelets_k) {
        // first check right side
        auto lanelets_right{lanelet_operations::laneletsRightOfLanelet(lanelet, false)};
        for (const auto &lanelet_right : lanelets_right) {
            if (std::any_of(lanes_p.begin(), lanes_p.end(), [lanelet_right](const std::shared_ptr<Lane> &lane_p) {
                    return lane_p->getContainedLaneletIDs().find(lanelet_right->getId()) !=
                           lane_p->getContainedLaneletIDs().end();
                }))
                return true;
        }
        // now check left side
        auto lanelets_left{lanelet_operations::laneletsLeftOfLanelet(lanelet, false)};
        for (const auto &lanelet_left : lanelets_left) {
            if (std::any_of(lanes_p.begin(), lanes_p.end(), [lanelet_left](const std::shared_ptr<Lane> &lane_p) {
                    return lane_p->getContainedLaneletIDs().find(lanelet_left->getId()) !=
                           lane_p->getContainedLaneletIDs().end();
                }))
                return true;
        }
    }
    return true;
}
double
InSameDirPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                     const std::shared_ptr<Obstacle> &obstacleK,
                                     const std::shared_ptr<Obstacle> &obstacleP,
                                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InSameDirPredicate does not support robust evaluation!");
}
Constraint InSameDirPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InSameDirPredicate does not support constraint evaluation!");
}
InSameDirPredicate::InSameDirPredicate() : CommonRoadPredicate(true) {}
