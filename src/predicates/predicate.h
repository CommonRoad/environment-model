//
// Created by Sebastian Maierhofer on 19.02.21.
//

#ifndef ENV_MODEL_PREDICATE_H
#define ENV_MODEL_PREDICATE_H

#include "../roadNetwork/road_network.h"
#include "../obstacle/obstacle.h"
#include "../world.h"
#include "../auxiliaryDefs/structs.h"

/**
 * Interface for a predicate.
 */
class Predicate {
public:
    /**
     * Constructor for predicate class.
     *
     * @param parameters Struct containing parameters of all predicates.
     */
    explicit Predicate(const PredicateParameters &parameters);

    /**
     * Default constructor for predicate class without parameter.
     *
     * @param parameters Struct containing parameters of all predicates.
     */
    Predicate();

    /**
     * Virtual function for the boolean evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool booleanEvaluation(size_t timeStep,
                                   const std::shared_ptr<World>& world,
                                   const std::shared_ptr<Obstacle>& obstacleK,
                                   const std::shared_ptr<Obstacle>& obstacleP) = 0;

    /**
     * Virtual function for the robustness evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double robustEvaluation(size_t timeStep,
                                    const std::shared_ptr<World>& world,
                                    const std::shared_ptr<Obstacle>& obstacleK,
                                    const std::shared_ptr<Obstacle>& obstacleP) = 0;


    /**
     * Virtual function for the constraint evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint constraintEvaluation(size_t timeStep,
                                            const std::shared_ptr<World>& world,
                                            const std::shared_ptr<Obstacle>& obstacleK,
                                            const std::shared_ptr<Obstacle>& obstacleP) = 0;

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     */
    bool statisticBooleanEvaluation(size_t timeStep,
                                    const std::shared_ptr<World>& world,
                                    const std::shared_ptr<Obstacle>& obstacleK,
                                    const std::shared_ptr<Obstacle>& obstacleP);

    /**
     * Getter for parameters.
     *
     * @return Struct containing all parameters.
     */
    [[nodiscard]] const PredicateParameters &getParameters() const;

    /**
     * Setter for parameters.
     *
     * @param parameters Struct containing all parameters.
     */
    void setParameters(const PredicateParameters &parameters);

protected:
    PredicateParameters parameters;             //**< Struct containing parameters of all predicates. */
public:
    static PredicateStatistics statistics;      //**< Struct storing statistics of a predicate, e.g., average computation time,  number of calls, etc. */
};


#endif //ENV_MODEL_PREDICATE_H
