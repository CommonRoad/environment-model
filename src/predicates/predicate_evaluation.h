//
// Created by Sebastian Maierhofer on 26.04.21.
//

#ifndef ENV_MODEL_PREDICATE_EVALUATION_H
#define ENV_MODEL_PREDICATE_EVALUATION_H

#include <memory>
#include "world.h"

// TODO extract interface which can also be used by SPOT
class PredicateEvaluation {
public:
    uint8_t registerScenario(int id, std::shared_ptr<RoadNetwork> roadNetwork,
                             std::vector<std::shared_ptr<Obstacle>> obstacleList,
                             std::vector<std::shared_ptr<Obstacle>> egoVehicles);
//    uint8_t removeScenario(const int &Id);
    static PredicateEvaluation *getInstance(); // Instantiates Singleton/gives back reference

private:
    static PredicateEvaluation *instance; // self-reference
    std::vector<std::tuple<int, std::shared_ptr<World>>> worldList;

};


#endif //ENV_MODEL_PREDICATE_EVALUATION_H
