//
// Created by Sebastian Maierhofer on 06.04.21.
//

#ifndef ENV_MODEL_WORLD_H
#define ENV_MODEL_WORLD_H

#include <memory>
#include "roadNetwork/road_network.h"
#include "obstacle/obstacle.h"

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
    World(int timeStep, std::shared_ptr<RoadNetwork> &roadNetwork,
          std::vector<std::shared_ptr<Obstacle>> egoVehicles,
          std::vector<std::shared_ptr<Obstacle>> obstacles);

    /**
     * Getter for world time step.
     *
     * @return Time step of world.
     */
    [[nodiscard]] int getTimeStep() const;

    /**
     * Getter for pointer to road network object.
     *
     * @return Pointer to road network object.
     */
    [[nodiscard]] std::shared_ptr<RoadNetwork> &getRoadNetwork() const;

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

private:
    int timeStep;
    std::shared_ptr<RoadNetwork>& roadNetwork;              //**< road network containing lanelets, traffic signs, etc. */
    std::vector<std::shared_ptr<Obstacle>> egoVehicles;     //**< pointers to ego vehicle objects */
    std::vector<std::shared_ptr<Obstacle>> obstacles;       //**< pointers to obstacles*/
};


#endif //ENV_MODEL_WORLD_H
