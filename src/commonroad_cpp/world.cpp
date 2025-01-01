#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <memory>
#include <utility>
#include <vector>

#include "commonroad_cpp/roadNetwork/lanelet/lane_operations.h"
#include <commonroad_cpp/world.h>
#include <spdlog/spdlog.h>

World::World(std::string name, size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
             std::vector<std::shared_ptr<Obstacle>> egos, std::vector<std::shared_ptr<Obstacle>> otherObstacles,
             double timeStepSize, const WorldParameters &worldParams)
    : name(std::move(name)), timeStep(timeStep), roadNetwork(roadNetwork), egoVehicles(std::move(egos)),
      obstacles(std::move(otherObstacles)), dt(timeStepSize), worldParameters(worldParams) {
    for (const auto &lane : roadNetwork->getLanes())
        idCounter = std::max(idCounter, lane->getId());
    for (const auto &lanelet : roadNetwork->getLaneletNetwork())
        idCounter = std::max(idCounter, lanelet->getId());
    for (const auto &obs : egoVehicles)
        idCounter = std::max(idCounter, obs->getId());
    for (const auto &obs : obstacles)
        idCounter = std::max(idCounter, obs->getId());
    if (!worldParams.hasDefaultParams()) {
        for (const auto &obs : obstacles) {
            obs->setActuatorParameters(worldParams.getActuatorParamsObstacles());
            obs->setSensorParameters(worldParams.getSensorParams());
            obs->setRoadNetworkParameters(worldParams.getRoadNetworkParams());
            obs->setTimeParameters(worldParams.getTimeParams());
        }
        for (const auto &obs : egoVehicles) {
            obs->setActuatorParameters(worldParams.getActuatorParamsEgo());
            obs->setSensorParameters(worldParams.getSensorParams());
            obs->setRoadNetworkParameters(worldParams.getRoadNetworkParams());
            obs->setTimeParameters(worldParams.getTimeParams());
        }
    }
    roadNetwork->setIdCounterRef(std::make_shared<size_t>(idCounter));
    setInitialLanes();
    initMissingInformation();
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
    // create lanes occupied by ego vehicle
    for (auto &obs : egoVehicles)
        obs->computeLanes(roadNetwork);
    // create lanes for each lanelet as initial lanelet
    for (const auto &la : roadNetwork->getLaneletNetwork())
        auto lanes{lane_operations::createLanesBySingleLanelets(
            {la}, roadNetwork, worldParameters.getSensorParams().getFieldOfViewFront(),
            worldParameters.getSensorParams().getFieldOfViewRear(),
            worldParameters.getRoadNetworkParams().numIntersectionsPerDirectionLaneGeneration, {})};
    // initialize curvilinear coordinate system for each lane
    for (const auto &lane : roadNetwork->getLanes())
        try {
            auto l{
                lane->getCurvilinearCoordinateSystem()}; // This is a dummy call to ensure that the CCS is initialized
        } catch (const std::runtime_error &e) {
            spdlog::error("World::setInitialLanes: Error while setting initial lanes: {}", e.what());
        } catch (...) {
            spdlog::error("World::setInitialLanes: Error while setting initial lanes.");
        }
}

std::shared_ptr<size_t> World::getIdCounterRef() const { return std::make_shared<size_t>(idCounter); }

double World::getDt() const { return dt; }

void World::setCurvilinearStates() {
    for (auto &obs : egoVehicles)
        if (!obs->isStatic())
            obs->setCurvilinearStates(roadNetwork);
}

void World::setEgoVehicles(std::vector<std::shared_ptr<Obstacle>> &egos) { egoVehicles = egos; }

void World::setEgoVehicles(std::vector<size_t> &egos) {
    for (const auto &eID : egos) {
        auto itr{std::find_if(obstacles.begin(), obstacles.end(),
                              [eID](const std::shared_ptr<Obstacle> &obs) { return obs->getId() == eID; })};
        if (itr != obstacles.end()) {
            egoVehicles.push_back(*itr);
            obstacles.erase(itr);
        } else
            throw std::runtime_error("PythonInterface::createScenario: Unknown ego ID.");
    }
}

const std::string &World::getName() const { return name; }

void World::initMissingInformation() {
    for (const auto &obs : egoVehicles) {
        if (obs->isStatic())
            continue;
        for (const auto &obsTimeStep : obs->getTimeSteps())
            if (!obs->getStateByTimeStep(obsTimeStep)->getValidStates().acceleration)
                obs->interpolateAcceleration(obsTimeStep, dt);
    }
    for (const auto &obs : obstacles) {
        if (obs->isStatic())
            continue;
        for (const auto &obsTimeStep : obs->getTimeSteps())
            if (!obs->getStateByTimeStep(obsTimeStep)->getValidStates().acceleration)
                obs->interpolateAcceleration(obsTimeStep, dt);
    }
}

void World::updateObstacles(std::vector<std::shared_ptr<Obstacle>> &obstacleList) {
    std::vector<std::shared_ptr<Obstacle>> newObstacles;
    std::set<size_t> newObstacleIds;
    for (const auto &obs : obstacleList) {
        newObstacleIds.insert(obs->getId());
        auto existingObs{std::find_if(obstacles.begin(), obstacles.end(), [obs](const std::shared_ptr<Obstacle> &o) {
            return o->getId() == obs->getId();
        })};

        // obstacle is new -> add to list
        if (existingObs == obstacles.end()) {
            newObstacles.push_back(obs);
            continue;
        }

        // obstacle already existing -> update if history of new obstacle is empty; otherwise add new one to list;
        // catches case where obstacle is already updated in calling function
        if (!obs->getTrajectoryHistory().empty()) {
            newObstacles.push_back(obs);
            continue;
        }
        (*existingObs)->updateCurrentState(obs->getCurrentState());
        (*existingObs)->setTrajectoryPrediction(obs->getTrajectoryPrediction());
        newObstacles.push_back(*existingObs);
    }

    // obstacle is not present anymore -> consider until history not relevant anymore
    for (const auto &obs : obstacles) {
        if (newObstacleIds.find(obs->getId()) == newObstacleIds.end()) {
            // if obs history passed -> continue
            if (obs->historyPassed(newObstacles.front()->getCurrentState()->getTimeStep()))
                continue;
            newObstacles.push_back(obs);
        }
    }
    obstacles = newObstacles;
}

void World::updateObstaclesTraj(
    std::vector<std::shared_ptr<Obstacle>> &obstacleList, std::map<size_t, std::shared_ptr<State>> &currentStates,
    std::map<size_t, tsl::robin_map<time_step_t, std::shared_ptr<State>>> &trajectoryPredictions) {
    std::vector<std::shared_ptr<Obstacle>> newObstacles{obstacleList};

    for (const auto &[obsID, state] : currentStates) {
        auto existingObs{findObstacle(obsID)};
        existingObs->updateCurrentState(state);
        existingObs->setTrajectoryPrediction(trajectoryPredictions[obsID]);
        newObstacles.push_back(existingObs);
    }

    // obstacle is not present anymore -> consider until history not relevant anymore
    for (const auto &obs : obstacles) {
        if (std::find_if(newObstacles.begin(), newObstacles.end(), [obs](const std::shared_ptr<Obstacle> &o) {
                return o->getId() == obs->getId();
            }) == newObstacles.end()) {
            // if obs history passed -> continue
            if (obs->historyPassed(newObstacles.front()->getCurrentState()->getTimeStep()))
                continue;
            newObstacles.push_back(obs);
        }
    }
    obstacles = newObstacles;
}

WorldParameters World::getWorldParameters() const { return worldParameters; }
