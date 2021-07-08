//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <memory>

class World {
  public:
    /**
     * Constructor for world.
     *
     * @param timeStep Current time step of world object.
     * @param roadNetwork Currently relevant road network.
     * @param egoVehicles List of ego vehicles.
     * @param obstacles List of obstacles.
     */
    World(size_t timeStep, std::shared_ptr<RoadNetwork> roadNetwork, std::vector<std::shared_ptr<Obstacle>> egoVehicles,
          std::vector<std::shared_ptr<Obstacle>> obstacles);

    /**
     * Getter for world time step.
     *
     * @return Time step of world.
     */
    [[nodiscard]] size_t getTimeStep() const;

    /**
     * Getter for pointer to road network object.
     *
     * @return Pointer to road network object.
     */
    [[nodiscard]] std::shared_ptr<RoadNetwork> getRoadNetwork() const;

    /**
     * Getter for pointer to vector of ego vehicle objects.
     *
     * @return Vector with pointers to obstacle objects.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Obstacle>> &getEgoVehicles() const;

    /**
     * Getter for pointer to vector of ego vehicle objects.
     *
     * @return Vector with pointers to obstacle objects.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Obstacle>> &getObstacles() const;

    /**
     * Find all obstacles objects from given set of IDs.
     *
     * @return Vector with pointers to obstacle objects.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Obstacle>> findObstacles(const std::vector<size_t> &obstacleIdList) const;

    /**
     * Find all obstacles object corresponding to given ID.
     *
     * @return Obstacle object.
     */
    [[nodiscard]] std::shared_ptr<Obstacle> findObstacle(size_t obstacleId) const;

    void setInitialLanes();

    size_t getIdCounter() const;

  private:
    size_t timeStep;
    size_t idCounter;
    std::shared_ptr<RoadNetwork> roadNetwork;           //**< road network containing lanelets, traffic signs, etc. */
    std::vector<std::shared_ptr<Obstacle>> egoVehicles; //**< pointers to ego vehicle objects */
    std::vector<std::shared_ptr<Obstacle>> obstacles;   //**< pointers to obstacles*
};
