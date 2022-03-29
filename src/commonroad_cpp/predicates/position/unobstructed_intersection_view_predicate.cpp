//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "unobstructed_intersection_view_predicate.h"
#include "../../obstacle/obstacle_operations.h"
#include "../../roadNetwork/lanelet/lanelet_operations.h"
#include "../../world.h"

namespace bg = boost::geometry;

bool UnobstructedIntersectionViewPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                              const std::shared_ptr<Obstacle> &obstacleK,
                                                              const std::shared_ptr<Obstacle> &obstacleP) {
    for (const auto &inter : obstacle_operations::getIntersections(timeStep, world->getRoadNetwork(), obstacleK)) {
        for (const auto &incom : inter->getIncomings()) {
            auto newLanes{lanelet_operations::createLanesBySingleLanelets({incom->getIncomingLanelets()},
                                                                          world->getRoadNetwork(), 50, 0, 1)};
            for (const auto &lane : newLanes) {
                bool laneContained{false};
                auto dist{lane->getPathLength()};
                for (size_t idx{lane->getCenterVertices().size()}; idx > 0; --idx) {
                    if ((dist.back() - dist.at(idx)) > 50 and
                        bg::within(point_type(lane->getCenterVertices().at(idx).x, lane->getCenterVertices().at(idx).y),
                                   obstacleK->getFov())) {
                        laneContained = true;
                        break;
                    }
                }
                if (!laneContained)
                    return false;
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