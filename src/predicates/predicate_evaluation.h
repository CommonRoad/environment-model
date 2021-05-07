//
// Created by Sebastian Maierhofer on 26.04.21.
//

#ifndef ENV_MODEL_PREDICATE_EVALUATION_H
#define ENV_MODEL_PREDICATE_EVALUATION_H

#include "world.h"
#include <memory>

// TODO extract interface which can also be used by SPOT
class PredicateEvaluation {
  public:
    uint8_t registerScenario(size_t id, size_t timeStep, std::shared_ptr<RoadNetwork> roadNetwork,
                             std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                             std::vector<std::shared_ptr<Obstacle>> &egoVehicles);
    //    uint8_t removeScenario(const int &Id);
    static std::shared_ptr<PredicateEvaluation> getInstance(); // Instantiates Singleton/gives back reference
    std::shared_ptr<World> findWorld(size_t scenarioId);

  private:
    //   PredicateEvaluation() = default;                        // Private constructor so that it cannot be called
    // ~PredicateEvaluation() = default;                       // private destructor
    static std::shared_ptr<PredicateEvaluation> instance; // self-reference
    std::vector<std::tuple<int, std::shared_ptr<World>>> worldList;
};

#endif // ENV_MODEL_PREDICATE_EVALUATION_H
