//
// Created by Sebastian Maierhofer on 07.04.21.
//

#include "in_same_lane_predicate.h"

bool InSameLanePredicate::booleanEvaluation(int timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP) {
    for (const auto &laneP : obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep)) {
        for (const auto &laneK : obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep)) {
            if (laneP->getLanelet().getId() == laneK->getLanelet().getId()) {
                return true;
            }
        }
    }
    return false;

}

double InSameLanePredicate::robustEvaluation(int timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP) {
    return 0;
}

Constraint InSameLanePredicate::constraintEvaluation(int timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    return Constraint();
}
