//
// Created by Sebastian Maierhofer on 06.04.21.
//

#include "world.h"

#include <utility>

World::World(int timeStep, std::shared_ptr<RoadNetwork> roadNetwork, std::vector<std::shared_ptr<Obstacle>> egoVehicles,
             std::vector<std::shared_ptr<Obstacle>> obstacles)
    : timeStep(timeStep), roadNetwork(roadNetwork), egoVehicles(std::move(egoVehicles)),
      obstacles(std::move(obstacles)) {}

int World::getTimeStep() const { return timeStep; }

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