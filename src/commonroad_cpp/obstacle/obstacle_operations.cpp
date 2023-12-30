//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/road_network_config.h>

std::shared_ptr<Obstacle>
obstacle_operations::getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList, size_t obstacleId) {
    std::shared_ptr<Obstacle> temp{nullptr};
    for (const auto &obs : obstacleList) {
        if (obs->getId() == obstacleId) {
            temp = obs;
            break;
        }
    }
    return temp;
}

ObstacleType obstacle_operations::matchStringToObstacleType(const std::string &type) {
    if (type == "car")
        return ObstacleType::car;
    else if (type == "truck")
        return ObstacleType::truck;
    else if (type == "pedestrian")
        return ObstacleType::pedestrian;
    else if (type == "bus")
        return ObstacleType::bus;
    else if (type == "vehicle")
        return ObstacleType::vehicle;
    else if (type == "bicycle")
        return ObstacleType::bicycle;
    else if (type == "priority_vehicle")
        return ObstacleType::priority_vehicle;
    else if (type == "train")
        return ObstacleType::train;
    else if (type == "motorcycle")
        return ObstacleType::motorcycle;
    else if (type == "taxi")
        return ObstacleType::taxi;
    else
        return ObstacleType::unknown;
}

std::shared_ptr<Obstacle>
obstacle_operations::obstacleDirectlyLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                          const std::shared_ptr<Obstacle> &obstacleK,
                                          const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_left = obstaclesLeft(timeStep, obstacles, obstacleK, roadNetwork);
    if (vehicles_left.empty())
        return nullptr;
    else if (vehicles_left.size() == 1)
        return vehicles_left[0];
    else {
        std::shared_ptr<Obstacle> vehicle_directly_left = vehicles_left[0];
        for (const auto &obs : vehicles_left)
            if (obs->getLatPosition(timeStep, obstacleK->getReferenceLane(roadNetwork, timeStep)) <
                vehicle_directly_left->getLatPosition(timeStep, obstacleK->getReferenceLane(roadNetwork, timeStep)))
                vehicle_directly_left = obs;

        return vehicle_directly_left;
    }
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::obstaclesLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                   const std::shared_ptr<Obstacle> &obstacleK,
                                   const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_left;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj =
        obstaclesAdjacent(timeStep, obstacles, obstacleK, roadNetwork);
    // use cross product between a line and a point to evaluate whether obstacle is left
    const auto &obstacleKShape = obstacleK->getOccupancyPolygonShape(timeStep);
    assert(obstacleKShape.outer().size() >= 2);
    vertex vertA{obstacleKShape.outer()[1].x(), obstacleKShape.outer()[1].y()};
    vertex vertC{obstacleKShape.outer()[0].x(), obstacleKShape.outer()[0].y()};
    vertC -= vertA;

    for (const auto &obs : vehicles_adj) {
        const auto &obsShape = obs->getOccupancyPolygonShape(timeStep);
        assert(obsShape.outer().size() >= 4);
        vertex vertP0{obsShape.outer()[0].x(), obsShape.outer()[0].y()};
        vertex vertP1{obsShape.outer()[1].x(), obsShape.outer()[1].y()};
        vertex vertP2{obsShape.outer()[2].x(), obsShape.outer()[2].y()};
        vertex vertP3{obsShape.outer()[3].x(), obsShape.outer()[3].y()};
        auto crossProductF0{vertC.x * (vertP0.y - vertA.y) - vertC.y * (vertP0.x - vertA.x)};
        auto crossProductF1{vertC.x * (vertP1.y - vertA.y) - vertC.y * (vertP1.x - vertA.x)};
        auto crossProductF2{vertC.x * (vertP2.y - vertA.y) - vertC.y * (vertP2.x - vertA.x)};
        auto crossProductF3{vertC.x * (vertP3.y - vertA.y) - vertC.y * (vertP3.x - vertA.x)};
        if (crossProductF0 < 0 and crossProductF1 < 0 and crossProductF2 < 0 and crossProductF3 < 0)
            vehicles_left.push_back(obs);
    }

    return vehicles_left;
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::obstaclesAdjacent(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                       const std::shared_ptr<Obstacle> &obstacleK,
                                       const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::vector<std::shared_ptr<Obstacle>> vehiclesAdj;
    // use cross product between a line and a point to evaluate whether obstacle is adjacent

    const auto &obstacleKShape{obstacleK->getOccupancyPolygonShape(timeStep)};
    assert(obstacleKShape.outer().size() >= 4);
    vertex vertA{obstacleKShape.outer()[1].x(), obstacleKShape.outer()[1].y()};
    vertex vertB{obstacleKShape.outer()[2].x(), obstacleKShape.outer()[2].y()};
    vertex vertC{obstacleKShape.outer()[0].x(), obstacleKShape.outer()[0].y()};
    vertex vertD{obstacleKShape.outer()[3].x(), obstacleKShape.outer()[3].y()};
    vertB -= vertA;
    vertD -= vertC;
    for (const auto &obs : obstacles) {
        if (!obs->timeStepExists(timeStep) or obs->getId() == obstacleK->getId())
            continue;
        const auto &shape{obs->getOccupancyPolygonShape(timeStep)};
        assert(shape.outer().size() >= 4);
        vertex vertP0{shape.outer()[0].x(), shape.outer()[0].y()};
        vertex vertP1{shape.outer()[1].x(), shape.outer()[1].y()};
        vertex vertP2{shape.outer()[2].x(), shape.outer()[2].y()};
        vertex vertP3{shape.outer()[3].x(), shape.outer()[3].y()};

        auto crossProductF0{vertB.x * (vertP0.y - vertA.y) - vertB.y * (vertP0.x - vertA.x)};
        auto crossProductF1{vertB.x * (vertP1.y - vertA.y) - vertB.y * (vertP1.x - vertA.x)};
        auto crossProductF2{vertB.x * (vertP2.y - vertA.y) - vertB.y * (vertP2.x - vertA.x)};
        auto crossProductF3{vertB.x * (vertP3.y - vertA.y) - vertB.y * (vertP3.x - vertA.x)};
        auto crossProductH0{vertD.x * (vertP0.y - vertC.y) - vertD.y * (vertP0.x - vertC.x)};
        auto crossProductH1{vertD.x * (vertP1.y - vertC.y) - vertD.y * (vertP1.x - vertC.x)};
        auto crossProductH2{vertD.x * (vertP2.y - vertC.y) - vertD.y * (vertP2.x - vertC.x)};
        auto crossProductH3{vertD.x * (vertP3.y - vertC.y) - vertD.y * (vertP3.x - vertC.x)};

        if (crossProductF0 >= 0 and crossProductF1 >= 0 and crossProductF2 >= 0 and crossProductF3 >= 0)
            continue;
        else if (crossProductH0 <= 0 and crossProductH1 <= 0 and crossProductH2 <= 0 and crossProductH3 <= 0)
            continue;
        else
            vehiclesAdj.push_back(obs);
    }

    return vehiclesAdj;
}

std::shared_ptr<Obstacle>
obstacle_operations::obstacleDirectlyRight(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_right = obstaclesRight(timeStep, obstacles, obstacleK, roadNetwork);
    if (vehicles_right.empty())
        return nullptr;
    else if (vehicles_right.size() == 1)
        return vehicles_right[0];
    else {
        std::shared_ptr<Obstacle> vehicle_directly_right = vehicles_right[0];
        for (const auto &obs : vehicles_right)
            if (obs->getLatPosition(timeStep, obstacleK->getReferenceLane(roadNetwork, timeStep)) >
                vehicle_directly_right->getLatPosition(timeStep, obstacleK->getReferenceLane(roadNetwork, timeStep)))
                vehicle_directly_right = obs;
        return vehicle_directly_right;
    }
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::obstaclesRight(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_right;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj =
        obstaclesAdjacent(timeStep, obstacles, obstacleK, roadNetwork);
    // use cross product between a line and a point to evaluate whether obstacle is right
    const auto &obstacleKShape{obstacleK->getOccupancyPolygonShape(timeStep)};
    assert(obstacleKShape.outer().size() >= 4);
    vertex vertA{obstacleKShape.outer()[2].x(), obstacleKShape.outer()[2].y()};
    vertex vertC{obstacleKShape.outer()[3].x(), obstacleKShape.outer()[3].y()};
    vertC -= vertA;

    for (const auto &obs : vehicles_adj) {
        const auto &obsShape{obs->getOccupancyPolygonShape(timeStep)};
        assert(obsShape.outer().size() >= 4);
        vertex vertP0{obsShape.outer()[0].x(), obsShape.outer()[0].y()};
        vertex vertP1{obsShape.outer()[1].x(), obsShape.outer()[1].y()};
        vertex vertP2{obsShape.outer()[2].x(), obsShape.outer()[2].y()};
        vertex vertP3{obsShape.outer()[3].x(), obsShape.outer()[3].y()};
        auto crossProductF0{vertC.x * (vertP0.y - vertA.y) - vertC.y * (vertP0.x - vertA.x)};
        auto crossProductF1{vertC.x * (vertP1.y - vertA.y) - vertC.y * (vertP1.x - vertA.x)};
        auto crossProductF2{vertC.x * (vertP2.y - vertA.y) - vertC.y * (vertP2.x - vertA.x)};
        auto crossProductF3{vertC.x * (vertP3.y - vertA.y) - vertC.y * (vertP3.x - vertA.x)};
        if (crossProductF0 > 0 and crossProductF1 > 0 and crossProductF2 > 0 and crossProductF3 > 0)
            vehicles_right.push_back(obs);
    }
    return vehicles_right;
}

std::set<std::shared_ptr<Lanelet>>
obstacle_operations::laneletsRightOfObstacle(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                             const std::shared_ptr<Obstacle> &obs) {
    std::set<std::shared_ptr<Lanelet>> rightLanelets;
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obs->getOccupiedLaneletsByShape(roadNetwork, timeStep);

    for (auto &occ_l : occupiedLanelets) {
        std::vector<std::shared_ptr<Lanelet>> newLanelets = lanelet_operations::laneletsRightOfLanelet(occ_l);
        for (const auto &lanelet : newLanelets)
            rightLanelets.emplace(lanelet);
    }
    return rightLanelets;
}

std::set<std::shared_ptr<Lanelet>>
obstacle_operations::laneletsLeftOfObstacle(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                            const std::shared_ptr<Obstacle> &obs) {
    std::set<std::shared_ptr<Lanelet>> leftLanelets;
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obs->getOccupiedLaneletsByShape(roadNetwork, timeStep);

    for (auto &occ_l : occupiedLanelets) {
        std::vector<std::shared_ptr<Lanelet>> newLanelets = lanelet_operations::laneletsLeftOfLanelet(occ_l);
        for (const auto &lanelet : newLanelets)
            leftLanelets.emplace(lanelet);
    }
    return leftLanelets;
}

std::vector<std::shared_ptr<Intersection>>
obstacle_operations::getIntersections(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                      const std::shared_ptr<Obstacle> &obs) {
    std::vector<std::shared_ptr<Intersection>> relevantIntersections;
    auto relevantLanelets{lane_operations::extractLaneletsFromLanes(obs->getOccupiedLanes(roadNetwork, timeStep))};
    for (const auto &inter : roadNetwork->getIntersections())
        for (const auto &interLet : inter->getMemberLanelets(roadNetwork))
            if (std::any_of(
                    relevantLanelets.begin(), relevantLanelets.end(),
                    [interLet](const std::shared_ptr<Lanelet> &let) { return let->getId() == interLet->getId(); })) {
                relevantIntersections.push_back(inter);
                break;
            }
    return relevantIntersections;
}

double obstacle_operations::drivingDistanceToCoordinatePoint(double xPosition, double yPosition,
                                                             const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                             const std::shared_ptr<Obstacle> &obs, size_t timeStep) {
    double car_front = obs->frontS(roadNetwork, timeStep);
    double lane_s = obs->getReferenceLane(roadNetwork, timeStep)
                        ->getCurvilinearCoordinateSystem()
                        ->convertToCurvilinearCoords(xPosition, yPosition)[0] -
                    RoadNetworkParameters::numAdditionalSegmentsCCS * RoadNetworkParameters::eps2;
    return lane_s - car_front;
}
