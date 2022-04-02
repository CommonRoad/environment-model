//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "unobstructed_intersection_view_predicate.h"
#include "../../geometry/geometric_operations.h"
#include "../../obstacle/obstacle_operations.h"
#include "../../roadNetwork/lanelet/lanelet_operations.h"
#include "../../world.h"

namespace bg = boost::geometry;

bool UnobstructedIntersectionViewPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                              const std::shared_ptr<Obstacle> &obstacleK,
                                                              const std::shared_ptr<Obstacle> &obstacleP) {
    for (const auto &inter : obstacle_operations::getIntersections(timeStep, world->getRoadNetwork(), obstacleK)) {
        for (const auto &incom : inter->getIncomings()) {
            for (const auto &let : incom->getIncomingLanelets()) {
                auto newLanes{lanelet_operations::combineLaneletAndPredecessorsToLane(let, 50, 0, {})};
                for (const auto &laneLanelets : newLanes) {
                    auto lane{lanelet_operations::createLaneByContainedLanelets(laneLanelets, 1)};
                    bool laneContained{false};
                    std::deque<polygon_type> laneletIntersection;
                    bg::intersection(obstacleK->getFov(), lane->getOuterPolygon(), laneletIntersection);
                    // lane is completely within fov
                    if (laneletIntersection.empty() and bg::within(lane->getOuterPolygon(), obstacleK->getFov())) {
                        if (geometric_operations::euclideanDistance2Dim(lane->getLeftBorderVertices().back(),
                                                                        lane->getLeftBorderVertices().front()) > 50 or
                            geometric_operations::euclideanDistance2Dim(lane->getRightBorderVertices().back(),
                                                                        lane->getRightBorderVertices().front()) > 50)
                            continue;
                        else
                            return false;
                    } else if (laneletIntersection.empty())
                        return false;
                    for (const auto &vert : laneletIntersection.at(0).outer()) {
                        if (geometric_operations::euclideanDistance2Dim(lane->getLeftBorderVertices().back(),
                                                                        vertex{vert.x(), vert.y()}) > 50 or
                            geometric_operations::euclideanDistance2Dim(lane->getRightBorderVertices().back(),
                                                                        vertex{vert.x(), vert.y()}) > 50) {
                            laneContained = true;
                            break;
                        }
                    }
                    if (!laneContained)
                        return false;
                }
            }
        }
    }
    return true;
}

double UnobstructedIntersectionViewPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                               const std::shared_ptr<Obstacle> &obstacleK,
                                                               const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("UnobstructedIntersectionViewPredicate does not support robust evaluation!");
}

Constraint UnobstructedIntersectionViewPredicate::constraintEvaluation(size_t timeStep,
                                                                       const std::shared_ptr<World> &world,
                                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                                       const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("UnobstructedIntersectionViewPredicate does not support constraint evaluation!");
}

UnobstructedIntersectionViewPredicate::UnobstructedIntersectionViewPredicate() : CommonRoadPredicate(false) {}