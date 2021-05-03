//
// Created by Sebastian Maierhofer on 07.04.21.
//

#ifndef ENV_MODEL_IN_SAME_LANE_H
#define ENV_MODEL_IN_SAME_LANE_H

#include "predicates/predicate.h"

class InSameLanePredicate : public Predicate{
    /**
     * Constraint evaluation of predicate.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    bool booleanEvaluation(size_t timeStep,
                                    const std::shared_ptr<World>& world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP);

    double
    robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                     const std::shared_ptr<Obstacle> &obstacleP) override;

    Constraint
    constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
                         const std::shared_ptr<Obstacle> &obstacleP) override;

};


#endif //ENV_MODEL_IN_SAME_LANE_H
