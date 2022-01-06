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
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacleK->getOccupiedLaneletsByShape(timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_left =
        obstacle_operations::obstacleDirectlyLeft(timeStep, world->getObstacles(), obstacleK);

    if (vehicle_directly_left != nullptr and
        (vehicle_directly_left->rightD(timeStep, obstacleK->getReferenceLane(timeStep)) - obstacleK->leftD(timeStep)) <
            parameters.closeToOtherVehicle) {
        return true;
    } else {
        std::vector<std::shared_ptr<Lane>> lanes;
        for (auto &occLa : occupiedLanelets) {
            std::vector<std::shared_ptr<Lane>> lanesOfLanelet =
                world->getRoadNetwork()->findLanesByContainedLanelet(occLa->getId());
            for (auto &lane : lanesOfLanelet) {
                lanes.push_back(lane);
            }
        }
        return std::all_of(lanes.begin(), lanes.end(), [obstacleK, this, timeStep](const std::shared_ptr<Lane> &lane) {
            return 0.5 * lane->getWidth(obstacleK->getLonPosition(timeStep, lane)) - obstacleK->leftD(timeStep, lane) <=
                   parameters.closeToLaneBorder;
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