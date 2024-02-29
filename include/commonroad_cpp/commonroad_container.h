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
     * @param scenarioId ID under which CommonRoad scenario should be stored (not CommonRoad benchmark ID).
     * @param name Name/Benchmark ID of scenario.
     * @param timeStep Time step the scenario (world representation) corresponds to.
     * @param timeStepSize Time step size[s].
     * @param roadNetwork CommonRoad road network representation.
     * @param obstacleList List of obstacles.
     * @param egoVehicles List of ego vehicles.
     */
    void registerScenario(size_t scenarioId, const std::string &name, size_t timeStep, double timeStepSize,
                          const std::shared_ptr<RoadNetwork> &roadNetwork,
                          const std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                          const std::vector<std::shared_ptr<Obstacle>> &egoVehicles);

    /**
     * Removes scenario from container.
     *
     * @param scenarioId ID of world object which should be removed from container.
     */
    void removeScenario(size_t scenarioId);

    /**
     * Instantiates singleton/gives back self-reference.
     *
     * @return Self-reference.
     */
    static std::shared_ptr<CommonRoadContainer> getInstance();

    /**
     * Returns world object with given ID.
     *
     * @param scenarioId ID of required world object.
     * @return World object.
     */
    std::shared_ptr<World> findWorld(size_t scenarioId);

  private:
    static std::shared_ptr<CommonRoadContainer> instance; //**< self-reference */
    std::map<size_t, std::shared_ptr<World>> worldList;   //**< map containing pairs of id and world objects */
};
