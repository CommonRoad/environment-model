//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <stdexcept>

#include <commonroad_cpp/world.h>

#include "commonroad_container.h"

// Global static pointer used to ensure a single instance of the class.
std::shared_ptr<CommonRoadContainer> CommonRoadContainer::instance =
    nullptr; // with static declaration, this can live outside of class instance

void CommonRoadContainer::registerScenario(const size_t id, size_t timeStep,
                                           const std::shared_ptr<RoadNetwork> &roadNetwork,
                                           std::vector<std::shared_ptr<Obstacle>> &egoVehicles,
                                           std::vector<std::shared_ptr<Obstacle>> &obstacleList) {
    if (worldList.find(id) != worldList.end())
        return; // ID does already exist
    worldList.insert({id, std::make_shared<World>(timeStep, roadNetwork, egoVehicles, obstacleList)});
}

std::shared_ptr<CommonRoadContainer> CommonRoadContainer::getInstance() {
    if (!instance) // Only allow one instance of class to be generated.
        instance = std::make_shared<CommonRoadContainer>();
    return instance;
}

std::shared_ptr<World> CommonRoadContainer::findWorld(size_t id) {
    auto world{worldList.find(id)};
    if (world == worldList.end())
        throw std::logic_error("ID does not exist in container!");
    else
        return world->second;
}

void CommonRoadContainer::removeScenario(const size_t id) {
    if (worldList.find(id) == worldList.end())
        throw std::logic_error("ID does not exist in container!");
    worldList.erase(id);
}