#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <memory>
#include <utility>
#include <vector>

#include <commonroad_cpp/world.h>

World::World(std::string name, size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
             std::vector<std::shared_ptr<Obstacle>> egos, std::vector<std::shared_ptr<Obstacle>> otherObstacles,
             double timeStepSize)
    : name(name), timeStep(timeStep), roadNetwork(roadNetwork), egoVehicles(std::move(egos)),
      obstacles(std::move(otherObstacles)), dt(timeStepSize) {
    for (const auto &lane : roadNetwork->getLanes())
        idCounter = std::max(idCounter, lane->getId());
    for (const auto &lanelet : roadNetwork->getLaneletNetwork())
        idCounter = std::max(idCounter, lanelet->getId());
    for (const auto &obs : egoVehicles)
        idCounter = std::max(idCounter, obs->getId());
    for (const auto &obs : obstacles)
        idCounter = std::max(idCounter, obs->getId());
    roadNetwork->setIdCounterRef(std::make_shared<size_t>(idCounter));
    setInitialLanes();
}

size_t World::getTimeStep() const { return timeStep; }

std::shared_ptr<RoadNetwork> World::getRoadNetwork() const { return roadNetwork; }

const std::vector<std::shared_ptr<Obstacle>> &World::getEgoVehicles() const { return egoVehicles; }

const std::vector<std::shared_ptr<Obstacle>> &World::getObstacles() const { return obstacles; }

std::vector<std::shared_ptr<Obstacle>> World::findObstacles(const std::vector<size_t> &obstacleIdList) const {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    obstacleList.reserve(obstacleIdList.size());
    for (const auto &obstacleID : obstacleIdList) {
        for (const auto &obs : obstacles)
            if (obstacleID == obs->getId())
                obstacleList.emplace_back(obs);
    }
    for (const auto &obstacleID : obstacleIdList) {
        for (const auto &obs : egoVehicles)
            if (obstacleID == obs->getId())
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

void World::setInitialLanes() {
    for (auto &obs : egoVehicles)
        obs->computeLanes(roadNetwork);
}

std::shared_ptr<size_t> World::getIdCounterRef() const { return std::make_shared<size_t>(idCounter); }

double World::getDt() const { return dt; }

void World::setCurvilinearStates() {
    for (auto &obs : egoVehicles)
        if (!obs->isStatic())
            obs->setCurvilinearStates(roadNetwork);
}

void World::setEgoVehicles(std::vector<std::shared_ptr<Obstacle>> &egos) { egoVehicles = egos; }

const std::string &World::getName() const { return name; }
