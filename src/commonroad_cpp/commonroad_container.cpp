#include <stdexcept>

#include <commonroad_cpp/world.h>

#include <commonroad_cpp/commonroad_container.h>

// Global static pointer used to ensure a single instance of the class.
std::shared_ptr<CommonRoadContainer> CommonRoadContainer::instance =
    nullptr; // with static declaration, this can live outside of class instance

void CommonRoadContainer::registerScenario(const size_t scenarioId, const std::string &name, size_t timeStep,
                                           double timeStepSize, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                           const std::vector<std::shared_ptr<Obstacle>> &egoVehicles,
                                           const std::vector<std::shared_ptr<Obstacle>> &obstacleList) {
    if (worldList.find(scenarioId) != worldList.end())
        return; // ID does already exist
    worldList.insert(
        {scenarioId, std::make_shared<World>(name, timeStep, roadNetwork, egoVehicles, obstacleList, timeStepSize)});
}

std::shared_ptr<CommonRoadContainer> CommonRoadContainer::getInstance() {
    if (!instance) // Only allow one instance of class to be generated.
        instance = std::make_shared<CommonRoadContainer>();
    return instance;
}

std::shared_ptr<World> CommonRoadContainer::findWorld(size_t scenarioId) {
    auto world{worldList.find(scenarioId)};
    if (world == worldList.end())
        throw std::logic_error("ID does not exist in container!");
    else
        return world->second;
}

void CommonRoadContainer::removeScenario(const size_t scenarioId) {
    if (worldList.find(scenarioId) == worldList.end())
        throw std::logic_error("ID does not exist in container!");
    worldList.erase(scenarioId);
}