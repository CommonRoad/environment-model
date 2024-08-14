//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/approach_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/regulatory/in_front_of_intersection_predicate.h>

bool InFrontOfIntersectionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters) {

    ApproachIntersectionPredicate approachIntersectionPredicate = ApproachIntersectionPredicate();
    if (!approachIntersectionPredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP))
        return false;

    auto lanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    for (const auto &lanelet : lanelets) {
        if (!lanelet->hasLaneletType(LaneletType::incoming)) {
            continue;
        }
        const std::vector<vertex> &leftBorder = lanelet->getLeftBorderVertices();
        const std::vector<vertex> &rightBorder = lanelet->getRightBorderVertices();

        std::vector<vertex> last_line{rightBorder.back(), leftBorder.back()};
        auto min_dist{regulatory_elements_utils::minDistance(last_line, obstacleK->getOccupancyPolygonShape(timeStep))};

        if (min_dist < parameters.getParam("intersectionDistance"))
            return true;
    }
    return false;
}

double InFrontOfIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InFrontOfIntersectionPredicate does not support robust evaluation!");
}
Constraint InFrontOfIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InFrontOfIntersectionPredicate does not support constraint evaluation!");
}
InFrontOfIntersectionPredicate::InFrontOfIntersectionPredicate() : CommonRoadPredicate(false) {}
