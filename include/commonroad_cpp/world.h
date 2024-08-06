#pragma once

#include <memory>
#include <string>
#include <vector>

class RoadNetwork;
class Obstacle;

class World {
  public:
    /**
     * Constructor for world.
     *
     * @param ID/Name of world.
     * @param timeStep Current time step of world object.
     * @param roadNetwork Currently relevant road network.
     * @param egos List of ego vehicles.
     * @param otherObstacles List of obstacles.
     * @param timeStepSize Time step size [s].
     */
    World(std::string name, size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
          std::vector<std::shared_ptr<Obstacle>> egos, std::vector<std::shared_ptr<Obstacle>> otherObstacles,
          double timeStepSize);

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

    /**
     * Initializes missing state information, e.g, acceleration or reaction time.
     */
    void initMissingInformation();

    /**
     * Computes for all ego vehicles occupied lanes per time step and sets reference lane.
     */
    void setInitialLanes();
};
