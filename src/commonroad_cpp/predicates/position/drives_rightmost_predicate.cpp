//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include "commonroad_cpp/world.h"

#include "../../obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "drives_rightmost_predicate.h"

bool DrivesRightmostPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 OptionalPredicateParameters additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_right =
        obstacle_operations::obstacleDirectlyRight(timeStep, world->getObstacles(), obstacleK, world->getRoadNetwork());

    if (vehicle_directly_right != nullptr and
        (obstacleK->rightD(world->getRoadNetwork(), timeStep) -
         vehicle_directly_right->leftD(timeStep, obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep))) <
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
            return 0.5 * lane->getWidth(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                        obstacleK->getStateByTimeStep(timeStep)->getYPosition()) +
                       obstacleK->rightD(timeStep, lane) <=
                   parameters.closeToLaneBorder;
        });
    }
}

double DrivesRightmostPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("Drives Rightmost Predicate does not support robust evaluation!");
}

Constraint DrivesRightmostPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("Drives Rightmost Predicate does not support constraint evaluation!");
}

DrivesRightmostPredicate::DrivesRightmostPredicate() : CommonRoadPredicate(false) {}