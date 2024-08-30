#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/adjacent_lanelet_of_type_predicate.h>
#include <commonroad_cpp/world.h>

bool AdjacentLaneletOfTypePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    std::set<size_t> laneletIDs;
    for (const auto &la : lanelets)
        laneletIDs.insert(la->getId());
    std::vector<LaneletType> laTypes{lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(1))};
    if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::left)
        return std::any_of(
            lanelets.begin(), lanelets.end(),
            [additionalFunctionParameters, laneletIDs, laTypes](const std::shared_ptr<Lanelet> &lanelet) {
                std::vector<LaneletType> laTypesTmp;
                if (laTypes.size() == 1 and laTypes.at(0) == LaneletType::all)
                    laTypesTmp.insert(laTypesTmp.end(), lanelet->getLaneletTypes().begin(),
                                      lanelet->getLaneletTypes().end());
                else
                    laTypesTmp = laTypes;
                return lanelet->getAdjacentLeft().adj != nullptr and
                       lanelet->getAdjacentLeft().adj->hasLaneletTypes(laTypesTmp) and
                       laneletIDs.find(lanelet->getAdjacentLeft().adj->getId()) == laneletIDs.end();
            });
    else if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::right) {
        return std::any_of(
            lanelets.begin(), lanelets.end(),
            [additionalFunctionParameters, laneletIDs, laTypes](const std::shared_ptr<Lanelet> &lanelet) {
                std::vector<LaneletType> laTypesTmp;
                if (laTypes.size() == 1 and laTypes.at(0) == LaneletType::all)
                    laTypesTmp.insert(laTypesTmp.end(), lanelet->getLaneletTypes().begin(),
                                      lanelet->getLaneletTypes().end());
                else
                    laTypesTmp = laTypes;
                return lanelet->getAdjacentRight().adj != nullptr and
                       lanelet->getAdjacentRight().adj->hasLaneletTypes(laTypesTmp) and
                       laneletIDs.find(lanelet->getAdjacentRight().adj->getId()) == laneletIDs.end();
            });
    }
    throw std::runtime_error("AdjacentLaneletOfTypePredicate::booleanEvaluation:");
}

double AdjacentLaneletOfTypePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AdjacentLaneletOfTypePredicate does not support robust evaluation!");
}

Constraint AdjacentLaneletOfTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AdjacentLaneletOfTypePredicate does not support constraint evaluation!");
}

AdjacentLaneletOfTypePredicate::AdjacentLaneletOfTypePredicate() : CommonRoadPredicate(false) {}
