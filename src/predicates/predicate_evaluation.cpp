//
// Created by Sebastian Maierhofer on 26.04.21.
//

#include "predicate_evaluation.h"

// Global static pointer used to ensure a single instance of the class.
PredicateEvaluation *PredicateEvaluation::instance = nullptr; // with static declaration, this can live outside of class instance

uint8_t PredicateEvaluation::registerScenario(const int id, int timeStep,
                                              std::shared_ptr<RoadNetwork> roadNetwork,
                                              const std::vector<std::shared_ptr<Obstacle>>& obstacleList,
                                              const std::vector<std::shared_ptr<Obstacle>>& egoVehicles) {
    for (auto const &wo : worldList) {
        if (std::get<0>(wo) == id) {
            return 1;                                               // Id already given
        }
    }
    worldList.emplace_back(id, std::make_shared<World>(timeStep, roadNetwork, egoVehicles, obstacleList));

    return 0;
}

PredicateEvaluation *PredicateEvaluation::getInstance() {
    if (!instance) // Only allow one instance of class to be generated.
    {
        instance = new PredicateEvaluation();
    }
    return instance;
}

std::shared_ptr<World> PredicateEvaluation::findWorld(int scenarioId) {
    for (auto &it : worldList) {
        if (std::get<0>(it) == scenarioId)
            return std::get<1>(it);
    }
}
