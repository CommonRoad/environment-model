#pragma once

#include "commonroad_cpp/obstacle/actuator_parameters.h"
#include "commonroad_cpp/obstacle/sensor_parameters.h"
#include "commonroad_cpp/roadNetwork/road_network_config.h"
#include <memory>
#include <string>
#include <vector>

class RoadNetwork;
class Obstacle;

/**
 * Storage for general parameters.
 */
class WorldParameters {
  public:
    WorldParameters() = default;

    /**
     * Constructor of world parameters.
     *
     * @param roadNetworkParameters Parameters for road network.
     * @param sensorParameters Parameters for obstacle sensors.
     * @param actuatorParameters Parameters for obstacle actuators.
     */
    WorldParameters(const RoadNetworkParameters &roadNetworkParameters, const SensorParameters &sensorParameters,
                    const ActuatorParameters &actuatorParameters)
        : roadNetworkParams(roadNetworkParameters), sensorParams(sensorParameters), actuatorParams(actuatorParameters),
          defaultParams(false) {}

    /**
     * Getter for road network parameters.
     *
     * @return Road network parameters.
     */
    [[nodiscard]] RoadNetworkParameters getRoadNetworkParams() const { return roadNetworkParams; }

    /**
     * Getter for sensor parameters.
     *
     * @return Sensor parameters.
     */
    [[nodiscard]] SensorParameters getSensorParams() const { return sensorParams; }

    /**
     * Getter for actuator parameters.
     *
     * @return Actuator parameters.
     */
    [[nodiscard]] ActuatorParameters getActuatorParams() const { return actuatorParams; }

    /**
     * Checks whether default parameters are set.
     *
     * @return Boolean indicating whether default parameters are set.
     */
    [[nodiscard]] bool hasDefaultParams() const { return defaultParams; }

    /**
     * Setter for road network parameters.
     *
     * @param params Road network parameters.
     */
    void setRoadNetworkParams(const RoadNetworkParameters &params) {
        defaultParams = false;
        roadNetworkParams = params;
    }

    /**
     * Setter for sensor parameters.
     *
     * @param params Sensor parameters.
     */
    void setSensorParams(const SensorParameters &params) {
        defaultParams = false;
        sensorParams = params;
    }

    /**
     * Setter for actuator parameters.
     *
     * @param params Actuator parameters.
     */
    void setActuatorParams(const ActuatorParameters &params) {
        defaultParams = false;
        actuatorParams = params;
    }

  private:
    RoadNetworkParameters roadNetworkParams{RoadNetworkParameters()};         //**< Parameters for road network. */
    SensorParameters sensorParams{SensorParameters::dynamicDefaults()};       /** Parameters for obstacle sensors. */
    ActuatorParameters actuatorParams{ActuatorParameters::vehicleDefaults()}; /** Parameters for obstacle actuators. */
    bool defaultParams{true}; /** Boolean indicating whether default parameters are set. */
};

class World {
  public:
    /**
     * Constructor for world.
     *
     * @param name ID/Name of world.
     * @param timeStep Current time step of world object.
     * @param roadNetwork Currently relevant road network.
     * @param egos List of ego vehicles.
     * @param otherObstacles List of obstacles.
     * @param timeStepSize Time step size [s].
     * @param worldParams Parameters for world.
     */
    World(std::string name, size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
          std::vector<std::shared_ptr<Obstacle>> egos, std::vector<std::shared_ptr<Obstacle>> otherObstacles,
          double timeStepSize, const WorldParameters &worldParams = WorldParameters());

    /**
     * Default constructor without parameters for a scenario.
     */
    World() = default;

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
     * Setter for ego vehicles of world
     *
     * @param egos vector of pointer to obstacles
     */
    void setEgoVehicles(std::vector<std::shared_ptr<Obstacle>> &egos);

    /**
     * Sets ego vehicles for scenario by ids and moves the respective obstacles from obstacles to egoVehicles
     *
     * @param egos
     */
    void setEgoVehicles(std::vector<size_t> &egos);

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

    /**
     * Computes all curvilinear states for ego vehicles and obstacles.
     */
    void setCurvilinearStates();

    /**
     * Creates pointer to ID counter so that it can be increased by other classes.
     * @return Pointer to ID counter.
     */
    [[nodiscard]] std::shared_ptr<size_t> getIdCounterRef() const;

    /**
     * Getter for time step size.
     *
     * @return Time step size [s].
     */
    [[nodiscard]] double getDt() const;

    /**
     * Getter for name.
     *
     * @return Name of world.
     */
    [[nodiscard]] const std::string &getName() const;

  private:
    std::string name;                                   //**< ID/name of world. */
    size_t timeStep;                                    //**< reference time step where world was created. */
    size_t idCounter{0};                                //**< counter to ensure unique IDs among all objects. */
    std::shared_ptr<RoadNetwork> roadNetwork;           //**< road network containing lanelets, traffic signs, etc. */
    std::vector<std::shared_ptr<Obstacle>> egoVehicles; //**< pointers to ego vehicle objects */
    std::vector<std::shared_ptr<Obstacle>> obstacles;   //**< pointers to obstacles *
    double dt;                                          //**<Time step size [s] *
    WorldParameters worldParameters; //**< General parameters for world, e.g. obstacles, road network. */

    /**
     * Initializes missing state information, e.g, acceleration or reaction time.
     */
    void initMissingInformation();

    /**
     * Computes for all ego vehicles occupied lanes per time step and sets reference lane.
     */
    void setInitialLanes();
};
