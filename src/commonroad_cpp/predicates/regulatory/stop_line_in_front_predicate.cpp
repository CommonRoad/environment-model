//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <Eigen/Dense>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.h>

bool lineInFront(const std::pair<vertex, vertex> &line, const std::shared_ptr<Obstacle> &obs, size_t timeStep,
                 const std::shared_ptr<RoadNetwork> &roadNetwork) {
    auto ccs{obs->getReferenceLane(roadNetwork, timeStep)->getCurvilinearCoordinateSystem()};
    if (!ccs->cartesianPointInProjectionDomain(line.first.x, line.first.y))
        return false;
    auto pointA{ccs->convertToCurvilinearCoords(line.first.x, line.first.y)};
    if (!ccs->cartesianPointInProjectionDomain(line.second.x, line.second.y))
        return false;
    auto pointB{ccs->convertToCurvilinearCoords(line.second.x, line.second.y)};
    polygon_type shape{obs->getOccupancyPolygonShape(timeStep)};
    for (size_t idx{0}; idx < shape.outer().size(); ++idx) {
        auto point{shape.outer().at(idx)};
        if (!ccs->cartesianPointInProjectionDomain(point.x(), point.y()))
            return false;
        auto c{ccs->convertToCurvilinearCoords(point.x(), point.y())};
        // ((b.X - a.X)*(c.Y - a.Y) > (b.Y - a.Y)*(c.X - a.X)) -> left of line a.y < b.y
        if (pointA.y() < 0) {
            if ((pointB.x() - pointA.x()) * (c.y() - pointA.y()) < (pointB.y() - pointA.y()) * (c.x() - pointA.x()))
                return false;
        } else {
            if ((pointB.x() - pointA.x()) * (c.y() - pointA.y()) > (pointB.y() - pointA.y()) * (c.x() - pointA.x()))
                return false;
        }
    }
    return true;
}

double minDistance(const std::pair<vertex, vertex> &line, polygon_type shape) {
    auto pointA{(line.first.y - line.second.y)};
    auto pointB{line.second.x - line.first.x};
    auto pointC{(line.first.x - line.second.x) * line.first.y + (line.second.y - line.first.y) * line.first.x};
    std::vector<double> distances;
    for (const auto &point : shape.outer()) {
        distances.push_back(std::fabs((pointA * point.x() + pointB * point.y() + pointC)) /
                            (sqrt(pointA * pointA + pointB * pointB)));
    }
    return *std::min_element(distances.begin(), distances.end());
}

bool StopLineInFrontPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto lanelets{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep)};
    for (const auto &lanelet : lanelets) {
        std::shared_ptr<StopLine> stopLine{lanelet->getStopLine()};
        if (stopLine == nullptr)
            continue;

        if (lineInFront(stopLine->getPoints(), obstacleK, timeStep, world->getRoadNetwork()) and
            minDistance(stopLine->getPoints(), obstacleK->getOccupancyPolygonShape(timeStep)) <
                parameters.paramMap["stopLineDistance"])
            return true;
    }
    return false;
}

double StopLineInFrontPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("StopLineInFrontPredicate does not support robust evaluation!");
}

Constraint StopLineInFrontPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("StopLineInFrontPredicate does not support constraint evaluation!");
}
StopLineInFrontPredicate::StopLineInFrontPredicate() : CommonRoadPredicate(false) {}
