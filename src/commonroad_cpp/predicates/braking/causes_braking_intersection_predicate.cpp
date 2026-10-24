#include <limits>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.h>

bool CausesBrakingIntersectionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           const std::vector<std::string> &additionalFunctionParameters,
                                                           bool setBased) {
    if (obstacleP->getAcceleration(timeStep, setBased, true) >= parameters.getParam("aBrakingIntersection"))
        return false;
    std::vector<std::shared_ptr<Lane>> lanesP;
    std::vector<std::shared_ptr<Lane>> lanesK;

    // get all relevant lanes
    if (setBased and !obstacleP->getSetBasedPrediction().empty() and
        obstacleP->getCurrentState()->getTimeStep() < timeStep) {
        lanesP = obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep, setBased);
        lanesK = {obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
    } else if (setBased and !obstacleK->getSetBasedPrediction().empty() and
               obstacleK->getCurrentState()->getTimeStep() < timeStep) {
        lanesK = obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep, setBased);
        lanesP = {obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
    } else {
        lanesK = {obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
        lanesP = {obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
    }

    // iterate over all lane pairs and compute intersection points; if between any intersection points and any obstacle
    // front position is below threshold return true
    for (const auto &laneP : lanesP) {
        std::vector<vertex> points;
        for (const auto &laneK : lanesK) {
            auto intersectionPoints{laneP->computeIntersectionPointsWithShape({laneK->getOuterPolygon()})};
            if (intersectionPoints.empty())
                continue;
            for (const auto &point : intersectionPoints) {
                double distance{std::numeric_limits<double>::max()};
                auto pointCCSLon{
                    laneP->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(point.x, point.y).x()};
                if (setBased and !obstacleP->getSetBasedPrediction().empty() and
                    obstacleP->getCurrentState()->getTimeStep() < timeStep) {
                    for (const auto &obsShape : obstacleP->getOccupancyPolygonShape(timeStep))
                        for (const auto &obsPoint : obsShape.outer()) {
                            polygon_type polygonPos;
                            boost::geometry::append(polygonPos, point_type{obsPoint.x(), obsPoint.y()});
                            if (!laneP->applyIntersectionTesting(polygonPos))
                                continue;
                            auto pointCCSLonObs{laneP->getCurvilinearCoordinateSystem()
                                                    ->convertToCurvilinearCoords(obsPoint.x(), obsPoint.y())
                                                    .x()};
                            distance = std::min(distance, pointCCSLon - pointCCSLonObs);
                        }
                } else
                    distance = pointCCSLon - obstacleP->frontS(timeStep, laneP->getCurvilinearCoordinateSystem());
                if (parameters.getParam("dCauseBrakingIntersection") <= distance and
                    distance <= parameters.getParam("dBrakingIntersection"))
                    return true;
            }
        }
    }
    return false;
}

bool CausesBrakingIntersectionPredicate::evaluateSetBased(const size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const bool setBased, const bool must) {
    const bool occupancyP{evaluatesOccupancy(obstacleP, timeStep, setBased)};
    const bool occupancyK{!occupancyP and evaluatesOccupancy(obstacleK, timeStep, setBased)};
    if (!occupancyP and !occupancyK)
        return CausesBrakingIntersectionPredicate::booleanEvaluation(timeStep, world, obstacleK, obstacleP, {"0.0"},
                                                                     setBased);
    const double dCause{parameters.getParam("dCauseBrakingIntersection")};
    const double dBraking{parameters.getParam("dBrakingIntersection")};

    // the pth vehicle has to brake: must: all covered behaviors (highest acceleration), may: one covered behavior
    // (lowest acceleration)
    if (obstacleP->getAcceleration(timeStep, true, !(occupancyP and must)) >=
        parameters.getParam("aBrakingIntersection"))
        return false;

    // Set-based prediction of the pth vehicle: the fronts of the covered behaviors lie within the longitudinal extent
    // [minLonObs, maxLonObs] of the occupancy within a lane, so the covered distances to an intersection point form the
    // interval [pointLon - maxLonObs, pointLon - minLonObs]. The occupancy is clipped to the lane so that the extent
    // also covers occupancies which cross the lane with all vertices outside of it and edges which cross the lane
    // border. must: the whole occupancy lies within the lane and all covered distances are within [dCause, dBraking];
    // may: some covered distance is within [dCause, dBraking].
    if (occupancyP) {
        auto occupancy{obstacleP->getOccupancyPolygonShape(timeStep)};
        boost::geometry::correct(occupancy); // area and intersection require closed polygons with clockwise orientation
        const double occupancyArea{boost::geometry::area(occupancy)};
        const auto laneK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
        for (const auto &laneP : obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep, true)) {
            multi_polygon_type occupancyInLane;
            boost::geometry::intersection(occupancy, laneP->getOuterPolygon(), occupancyInLane);
            if (occupancyInLane.empty())
                continue;
            if (must and boost::geometry::area(occupancyInLane) < occupancyArea * (1 - 1e-9))
                continue; // some covered behaviors are outside of the lane
            double minLonObs{std::numeric_limits<double>::max()};
            double maxLonObs{std::numeric_limits<double>::lowest()};
            for (const auto &obsShape : occupancyInLane)
                for (const auto &obsPoint : obsShape.outer()) {
                    const double lonObs{laneP->getCurvilinearCoordinateSystem()
                                            ->convertToCurvilinearCoords(obsPoint.x(), obsPoint.y())
                                            .x()};
                    minLonObs = std::min(minLonObs, lonObs);
                    maxLonObs = std::max(maxLonObs, lonObs);
                }
            for (const auto &point : laneP->computeIntersectionPointsWithShape({laneK->getOuterPolygon()})) {
                const double pointLon{
                    laneP->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(point.x, point.y).x()};
                const double minDistance{pointLon - maxLonObs};
                const double maxDistance{pointLon - minLonObs};
                if (must ? (dCause <= minDistance and maxDistance <= dBraking)
                         : (minDistance <= dBraking and dCause <= maxDistance))
                    return true;
            }
        }
        return false;
    }

    // Set-based prediction of the kth vehicle: each covered behavior drives on one of the occupied lanes.
    // must: the condition holds for every occupied lane; may: it holds for some occupied lane.
    const auto laneP{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
    const double frontP{obstacleP->frontS(timeStep, laneP->getCurvilinearCoordinateSystem())};
    const auto lanesK{obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep, true)};
    if (lanesK.empty())
        return false;
    for (const auto &laneK : lanesK) {
        bool satisfied{false};
        for (const auto &point : laneP->computeIntersectionPointsWithShape({laneK->getOuterPolygon()})) {
            const double distance{
                laneP->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(point.x, point.y).x() - frontP};
            if (dCause <= distance and distance <= dBraking) {
                satisfied = true;
                break;
            }
        }
        if (must and !satisfied)
            return false;
        if (!must and satisfied)
            return true;
    }
    return must;
}

Constraint CausesBrakingIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

double CausesBrakingIntersectionPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

CausesBrakingIntersectionPredicate::CausesBrakingIntersectionPredicate() : CommonRoadPredicate(true) {}
