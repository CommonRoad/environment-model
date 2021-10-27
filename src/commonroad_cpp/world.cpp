//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "roadNetwork/lanelet/lanelet_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <memory>
#include <utility>
#include <vector>

#include "world.h"

World::World(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
             std::vector<std::shared_ptr<Obstacle>> egoVehicles, std::vector<std::shared_ptr<Obstacle>> obstacles,
             double dt)
    : timeStep(timeStep), roadNetwork(roadNetwork), egoVehicles(std::move(egoVehicles)),
      obstacles(std::move(obstacles)), dt(dt) {
    for (const auto &la : roadNetwork->getLanes())
        idCounter = std::max(idCounter, la->getId());
    for (const auto &la : roadNetwork->getLaneletNetwork())
        idCounter = std::max(idCounter, la->getId());
    for (const auto &obs : egoVehicles)
        idCounter = std::max(idCounter, obs->getId());
    for (const auto &obs : obstacles)
        idCounter = std::max(idCounter, obs->getId());
    setInitialLanes();
    //    setCurvilinearStates();
}

size_t World::getTimeStep() const { return timeStep; }

std::shared_ptr<RoadNetwork> World::getRoadNetwork() const { return roadNetwork; }

const std::vector<std::shared_ptr<Obstacle>> &World::getEgoVehicles() const { return egoVehicles; }

const std::vector<std::shared_ptr<Obstacle>> &World::getObstacles() const { return obstacles; }

std::vector<std::shared_ptr<Obstacle>> World::findObstacles(const std::vector<size_t> &obstacleIdList) const {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    obstacleList.reserve(obstacleIdList.size());
    for (const auto &id : obstacleIdList) {
        for (const auto &obs : obstacles)
            if (id == obs->getId())
                obstacleList.emplace_back(obs);
    }
    for (const auto &id : obstacleIdList) {
        for (const auto &obs : egoVehicles)
            if (id == obs->getId())
                obstacleList.emplace_back(obs);
    }

    return obstacleList;
}

std::shared_ptr<Obstacle> World::findObstacle(size_t obstacleId) const {
    for (const auto &obs : obstacles)
        if (obstacleId == obs->getId())
            return obs;
    for (const auto &obs : egoVehicles)
        if (obstacleId == obs->getId())
            return obs;
    throw std::logic_error("Provided obstacle ID does not exist! ID: " + std::to_string(obstacleId));
}

void World::setInitialLanes() {
    for (auto &obs : egoVehicles)
        obs->computeLanes(roadNetwork, std::make_shared<size_t>(idCounter));
}

std::shared_ptr<size_t> World::getIdCounterRef() const { return std::make_shared<size_t>(idCounter); }

double World::getDt() const { return dt; }

void World::setCurvilinearStates() {
    for (auto &obs : egoVehicles)
        if (!obs->getIsStatic())
            obs->setCurvilinearStates();
}