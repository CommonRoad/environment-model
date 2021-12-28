//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <Eigen/Dense>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>

#include "stop_line_in_front_predicate.h"

bool StopLineInFrontPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    auto lanelets{obstacleK->getOccupiedLanelets(world->getRoadNetwork(), timeStep)};
    for (const auto &lanelet : lanelets) {
        std::shared_ptr<StopLine> stopLine{lanelet->getStopLine()};
        if (stopLine == nullptr)
            continue;
        Eigen::Vector2d stopLineLonPosOne =
            obstacleK->getReferenceLane(timeStep)->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(
                stopLine->getPoints().at(0).x, stopLine->getPoints().at(0).y);
        Eigen::Vector2d stopLineLonPosTwo =
            obstacleK->getReferenceLane(timeStep)->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(
                stopLine->getPoints().at(1).x, stopLine->getPoints().at(1).y);
        auto stopLineMinPos{std::min(stopLineLonPosOne.x(), stopLineLonPosTwo.x())};

        // maybe check orientation as in BA
        if (stopLineMinPos - parameters.stopLineDistance < obstacleK->frontS(timeStep) and
            obstacleK->frontS(timeStep) < stopLineMinPos) {
            return true;
        }
    }
    return false;
}

double StopLineInFrontPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("StopLineInFrontPredicate does not support robust evaluation!");
}

Constraint StopLineInFrontPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("StopLineInFrontPredicate does not support constraint evaluation!");
}
StopLineInFrontPredicate::StopLineInFrontPredicate() : CommonRoadPredicate(false) {}
