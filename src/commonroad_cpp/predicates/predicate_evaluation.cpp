//
// Created by Sebastian Maierhofer on 26.04.21.
//

#include "predicate_evaluation.h"

// Global static pointer used to ensure a single instance of the class.
std::shared_ptr<PredicateEvaluation> PredicateEvaluation::instance =
    nullptr; // with static declaration, this can live outside of class instance

uint8_t PredicateEvaluation::registerScenario(const size_t id, size_t timeStep,
                                              std::shared_ptr<RoadNetwork> roadNetwork,
                                              std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                              std::vector<std::shared_ptr<Obstacle>> &egoVehicles) {
    for (auto const &wo : worldList) {
        if (id == std::get<0>(wo))
            return 1; // Id already given
    }
    worldList.emplace_back(id, std::make_shared<World>(timeStep, roadNetwork, egoVehicles, obstacleList));

    return 0;
}

std::shared_ptr<PredicateEvaluation> PredicateEvaluation::getInstance() {
    if (!instance) // Only allow one instance of class to be generated.
        instance = std::make_shared<PredicateEvaluation>();
    return instance;
}

std::shared_ptr<World> PredicateEvaluation::findWorld(size_t scenarioId) {
    for (auto &it : worldList) {
        if (std::get<0>(it) == scenarioId)
            return std::get<1>(it);
    }
}

uint8_t PredicateEvaluation::removeScenario(const size_t scenarioId) {
  for (size_t i{0}; i < worldList.size(); ++i)
    if (std::get<0>(worldList.at(i)) == scenarioId) {
      worldList.erase(worldList.begin() + static_cast<long>(i));
      return 0;
    }
  return 1;
}