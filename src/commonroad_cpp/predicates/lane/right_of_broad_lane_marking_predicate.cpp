#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/lane/right_of_broad_lane_marking_predicate.h>

bool RightOfBroadLaneMarkingPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters) {
    auto lanelets{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep)};
    if (std::any_of(lanelets.begin(), lanelets.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
            return lanelet->getLineMarkingRight() == LineMarking::broad_dashed or
                   lanelet->getLineMarkingRight() == LineMarking::broad_solid;
        }))
        return false;
    auto laneletsLeft = obstacle_operations::laneletsLeftOfObstacle(timeStep, world->getRoadNetwork(), obstacleK);
    return std::any_of(laneletsLeft.begin(), laneletsLeft.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
        return lanelet->getLineMarkingRight() == LineMarking::broad_dashed or
               lanelet->getLineMarkingRight() == LineMarking::broad_solid;
    });
}

double RightOfBroadLaneMarkingPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("Right of broad lane marking does not support robust evaluation!");
}

Constraint RightOfBroadLaneMarkingPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("Right of broad lane marking does not support constraint evaluation!");
}

RightOfBroadLaneMarkingPredicate::RightOfBroadLaneMarkingPredicate() : CommonRoadPredicate(false) {}
