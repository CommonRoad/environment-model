//
// Created by Sebastian Maierhofer on 26.04.21.
//

#include "predicate_evaluation.h"

uint8_t PredicateEvaluation::registerScenario(const int id, std::shared_ptr<RoadNetwork> roadNetwork,
                               std::vector<std::shared_ptr<Obstacle>> obstacleList,
                               std::vector<std::shared_ptr<Obstacle>> egoVehicles) {
    for (auto const &wo : worldList) {
        std::cout << "Registered IDs: " << std::get<0>(it) << std::endl;

        if (std::get<0>(wo) == Id) {
            std::cout << "Scenario ID already in use" << std::endl; // only for debugging
            return 1;                                               // Id already given
        }
    }

    std::cout << "Unique ID: " << Id << std::endl;
    worldList.emplace_back(std::make_tuple(Id, World(roadNetwork, egoVehicles, obstacleList));

    return 0;
}

PredicateEvaluation *PredicateEvaluation::getInstance() {
    if (!PredicateEvaluation::Instance) // Only allow one instance of class to be generated.
    {
        std::cout << "Creating unique instance of PredicateEvaluation" << std::endl; // only for debugging
        PredicateEvaluation::Instance = new PredicateEvaluation();
    }
    // std::cout << "Returning instance of Spot" << std::endl; // only for debugging
    return PredicateEvaluation::Instance;
}
