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

#include "stop_line_in_front_predicate.h"

bool lineInFront(const std::vector<vertex> line, std::shared_ptr<Obstacle> obs, size_t timeStep, std::shared_ptr<RoadNetwork> roadNetwork){
    auto ccs{obs->getReferenceLane(roadNetwork, timeStep)->getCurvilinearCoordinateSystem()};
    if(!ccs->cartesianPointInProjectionDomain(line.at(0).x, line.at(0).y))
        return false;
    auto a{ccs->convertToCurvilinearCoords(line.at(0).x, line.at(0).y)};
    if(!ccs->cartesianPointInProjectionDomain(line.at(1).x, line.at(1).y))
        return false;
    auto b{ccs->convertToCurvilinearCoords(line.at(1).x, line.at(1).y)};
    polygon_type shape{obs->getOccupancyPolygonShape(timeStep)};
    for(size_t idx{0}; idx < shape.outer().size(); ++idx){
        auto point{shape.outer().at(idx)};
        if(!ccs->cartesianPointInProjectionDomain(point.x(), point.y()))
            return false;
        auto c{ccs->convertToCurvilinearCoords(point.x(), point.y())};
        //  std::cout << (stopLine->getPoints().at(1).x - stopLine->getPoints().at(0).x) * (point.y() - stopLine->getPoints().at(0).y) - (stopLine->getPoints().at(0).y() - stopLine->getPoints().at(0).y) * (point.x() - stopLine->getPoints().at(0).x) << " - time: " << std::to_string(timeStep) <<'\n';
        if(((b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x())) > 0)
            return false;
    }
    return true;
}

double minDistance(const std::vector<vertex> line, polygon_type shape){
    auto a{(line.at(0).y - line.at(1).y)};
    auto b{line.at(1).x - line.at(0).x};
    auto c{(line.at(0).x - line.at(1).x) * line.at(0).y + (line.at(1).y - line.at(0).y) * line.at(0).x};
    std::vector<double> distances;
    for(const auto &point : shape.outer()){
        distances.push_back(std::fabs((a * point.x() + b * point.y() + c)) / (sqrt(a * a + b * b)));
        //distances.push_back(std::abs((line.at(1).x - line.at(0).x) * ( line.at(0).y - point.y()) - (line.at(0).x - point.x()) * (line.at(1).y - line.at(0).y)) / sqrt(()));
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

        //        Eigen::Vector2d stopLineLonPosOne =
        //            obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
        //                ->getCurvilinearCoordinateSystem()
        //                ->convertToCurvilinearCoords(stopLine->getPoints().at(0).x, stopLine->getPoints().at(0).y);
        //        Eigen::Vector2d stopLineLonPosTwo =
        //            obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
        //                ->getCurvilinearCoordinateSystem()
        //                ->convertToCurvilinearCoords(stopLine->getPoints().at(1).x, stopLine->getPoints().at(1).y);
        //        auto stopLineMaxPos{std::max(stopLineLonPosOne.x(), stopLineLonPosTwo.x())};

        //        for(const auto &point : obstacleK->getOccupancyPolygonShape(timeStep).outer()) {
        //            auto ccs{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
        //                         ->getCurvilinearCoordinateSystem()};
        //            if(ccs->cartesianPointInProjectionDomain(point.x(), point.x()))
        //                auto obsPoint{ccs
        //                                  ->convertToCurvilinearCoords(point.x(), point.x())};
        //        }
        //  std::cout << (stopLine->getPoints().at(1).x - stopLine->getPoints().at(0).x) * (point.y() - stopLine->getPoints().at(0).y) - (stopLine->getPoints().at(0).y() - stopLine->getPoints().at(0).y) * (point.x() - stopLine->getPoints().at(0).x) << " - time: " << std::to_string(timeStep) <<'\n';
        //     std::cout << timeStep << " - in front: " <<lineInFront(stopLine->getPoints(), obstacleK, timeStep, world->getRoadNetwork()) << '\n';
        //      std::cout << timeStep << " - distance: " <<minDistance(stopLine->getPoints(), obstacleK->getOccupancyPolygonShape(timeStep)) << '\n';
        if(lineInFront(stopLine->getPoints(), obstacleK, timeStep, world->getRoadNetwork()) and minDistance(stopLine->getPoints(), obstacleK->getOccupancyPolygonShape(timeStep)) < parameters.stopLineDistance)
            return true;
        // }
        //        if (stopLineMaxPos - parameters.stopLineDistance < obstacleK->frontS(world->getRoadNetwork(), timeStep) and
        //            obstacleK->frontS(world->getRoadNetwork(), timeStep) < stopLineMaxPos) {
        //            return true;
        //        }
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
