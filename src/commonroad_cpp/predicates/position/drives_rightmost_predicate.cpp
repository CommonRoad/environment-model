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
#include "drives_rightmost_predicate.h"

bool DrivesRightmostPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacleK->getOccupiedLaneletsByShape(timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_right =
        obstacle_operations::obstacleDirectlyRight(timeStep, world->getObstacles(), obstacleK);

    if (vehicle_directly_right != nullptr) {
        return (obstacleK->rightD(timeStep) -
                vehicle_directly_right->leftD(timeStep, obstacleK->getReferenceLane(timeStep))) <
               parameters.closeToOtherVehicle;
    } else {
        double right_position = obstacleK->rightD(timeStep);
        double s_ego = obstacleK->getLonPosition(timeStep);
        std::vector<std::shared_ptr<Lane>> lanes;
        for (auto &occLa : occupiedLanelets) {
            std::vector<std::shared_ptr<Lane>> lanesOfLanelet =
                world->getRoadNetwork()->findLanesByContainedLanelet(occLa->getId());
            for (auto &lane : lanesOfLanelet) {
                lanes.push_back(lane);
            }
        }
        return std::all_of(lanes.begin(), lanes.end(),
                           [s_ego, right_position, this](const std::shared_ptr<Lane> &lane) {
                               return 0.5 * lane->getWidth(s_ego) + right_position <= parameters.closeToLaneBorder;
                           });
    }
}

double DrivesRightmostPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives Rightmost Predicate does not support robust evaluation!");
}

Constraint DrivesRightmostPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives Rightmost Predicate does not support constraint evaluation!");
}

DrivesRightmostPredicate::DrivesRightmostPredicate() : CommonRoadPredicate(false) {}