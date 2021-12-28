//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include "commonroad_cpp/world.h"

#include "../../obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "drives_leftmost_predicate.h"

bool DrivesLeftmostPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {

    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacleK->getOccupiedLanelets(timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_left =
        obstacle_operations::obstacleDirectlyLeft(timeStep, world->getObstacles(), obstacleK);

    if (vehicle_directly_left != nullptr) {
        return (vehicle_directly_left->rightD(timeStep) - obstacleK->leftD(timeStep)) < parameters.closeToOtherVehicle;
    } else {
        double left_position = obstacleK->leftD(timeStep);
        double s_ego = obstacleK->getLonPosition(timeStep);
        std::vector<std::shared_ptr<Lane>> lanes;
        for (auto &occLa : occupiedLanelets) {
            std::vector<std::shared_ptr<Lane>> lanesOfLanelet =
                roadNetwork->findLanesByContainedLanelet(occLa->getId());
            for (auto &lane : lanesOfLanelet) {
                lanes.push_back(lane);
            }
        }
        return std::all_of(lanes.begin(), lanes.end(), [s_ego, left_position, this](const std::shared_ptr<Lane> &lane) {
            return 0.5 * lane->getWidth(s_ego) - left_position <= parameters.closeToLaneBorder;
        });
    }
}

double DrivesLeftmostPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives Leftmost Predicate does not support robust evaluation!");
}

Constraint DrivesLeftmostPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives Leftmost Predicate does not support constraint evaluation!");
}

DrivesLeftmostPredicate::DrivesLeftmostPredicate() : CommonRoadPredicate(false) {}