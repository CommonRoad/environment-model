//
// Created by Sebastian Maierhofer on 06.04.21.
//

#include "world.h"

World::World(int timeStep, std::shared_ptr <RoadNetwork> &roadNetwork,
             const std::vector <std::shared_ptr<Obstacle>> &egoVehicles,
             const std::vector <std::shared_ptr<Obstacle>> &obstacles) : timeStep(timeStep), roadNetwork(roadNetwork),
                                                                         egoVehicles(egoVehicles),
                                                                         obstacles(obstacles) {}

int World::getTimeStep() const { return timeStep; }

std::shared_ptr <RoadNetwork> &World::getRoadNetwork() const { return roadNetwork; }

const std::vector <std::shared_ptr<Obstacle>> &World::getEgoVehicles() const { return egoVehicles; }

const std::vector <std::shared_ptr<Obstacle>> &World::getObstacles() const { return obstacles; }
