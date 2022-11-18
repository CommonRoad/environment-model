//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <Eigen/Dense>

#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.h>

bool lineInFront(const std::vector<vertex> &line, const std::shared_ptr<Obstacle> &obs, size_t timeStep,
                 const std::shared_ptr<RoadNetwork> &roadNetwork) {
    auto ccs{obs->getReferenceLane(roadNetwork, timeStep)->getCurvilinearCoordinateSystem()};
    if (!ccs->cartesianPointInProjectionDomain(line.at(0).x, line.at(0).y))
        return false;
    auto a{ccs->convertToCurvilinearCoords(line.at(0).x, line.at(0).y)};
    if (!ccs->cartesianPointInProjectionDomain(line.at(1).x, line.at(1).y))
        return false;
    auto b{ccs->convertToCurvilinearCoords(line.at(1).x, line.at(1).y)};
    polygon_type shape{obs->getOccupancyPolygonShape(timeStep)};
    for (size_t idx{0}; idx < shape.outer().size(); ++idx) {
        auto point{shape.outer().at(idx)};
        if (!ccs->cartesianPointInProjectionDomain(point.x(), point.y()))
            return false;
        auto c{ccs->convertToCurvilinearCoords(point.x(), point.y())};
        // ((b.X - a.X)*(c.Y - a.Y) > (b.Y - a.Y)*(c.X - a.X)) -> left of line a.y < b.y
        if (a.y() < 0) {
            if ((b.x() - a.x()) * (c.y() - a.y()) < (b.y() - a.y()) * (c.x() - a.x()))
                return false;
        } else {
            if ((b.x() - a.x()) * (c.y() - a.y()) > (b.y() - a.y()) * (c.x() - a.x()))
                return false;
        }
    }
    return true;
}

double minDistance(const std::vector<vertex> &line, polygon_type shape) {
    auto a{(line.at(0).y - line.at(1).y)};
    auto b{line.at(1).x - line.at(0).x};
    auto c{(line.at(0).x - line.at(1).x) * line.at(0).y + (line.at(1).y - line.at(0).y) * line.at(0).x};
    std::vector<double> distances;
    for (const auto &point : shape.outer()) {
        distances.push_back(std::fabs((a * point.x() + b * point.y() + c)) / (sqrt(a * a + b * b)));
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
                parameters.stopLineDistance)
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
