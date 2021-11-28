//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <map>
#include <memory>
#include <vector>

class World;
class RoadNetwork;
class Obstacle;

/**
 * Class for storing world objects based on CommonRoad scenarios.
 * The class is mainly used for the Python interface and implements the singleton pattern.
 */
class CommonRoadContainer {
  public:
    /**
     * Registers scenario in container.
     *
     * @param id ID under which CommonRoad scenario should be stored (not CommonRoad benchmark ID).
     * @param timeStep Time step the scenario (world representation) corresponds to.
     * @param dt Time step size[s].
     * @param roadNetwork CommonRoad road network representation.
     * @param obstacleList List of obstacles.
     * @param egoVehicles List of ego vehicles.
     */
    void registerScenario(size_t id, size_t timeStep, double dt, const std::shared_ptr<RoadNetwork> &roadNetwork,
                          std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                          std::vector<std::shared_ptr<Obstacle>> &egoVehicles);

    /**
     * Removes scenario from container.
     *
     * @param id ID of world object which should be removed from container.
     */
    void removeScenario(size_t id);

    /**
     * Instantiates singleton/gives back self-reference.
     *
     * @return Self-reference.
     */
    static std::shared_ptr<CommonRoadContainer> getInstance();

    /**
     * Returns world object with given ID.
     *
     * @param id ID of required world object.
     * @return World object.
     */
    std::shared_ptr<World> findWorld(size_t id);

  private:
    static std::shared_ptr<CommonRoadContainer> instance; //**< self-reference */
    std::map<size_t, std::shared_ptr<World>> worldList;   //**< map containing pairs of id and world objects */
};

