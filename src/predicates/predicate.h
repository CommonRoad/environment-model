//
// Created by Sebastian Maierhofer on 19.02.21.
//

#ifndef ENV_MODEL_PREDICATE_H
#define ENV_MODEL_PREDICATE_H

#include "../roadNetwork/road_network.h"
#include "../obstacle/obstacle.h"
#include "../auxiliaryDefs/structs.h"

/**
 * Interface for a predicate.
 */
class Predicate {
public:
    /**
     * Virtual function for the boolean evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool booleanEvaluation(int timeStep,
                                   const std::shared_ptr<RoadNetwork>& roadNetwork,
                                   const std::shared_ptr<Obstacle>& obstacleK,
                                   const std::shared_ptr<Obstacle>& obstacleP,
                                   const std::vector<std::shared_ptr<Obstacle>>& obstacles) = 0;


    /**
     * Virtual function for the robustness evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double robustEvaluation(int timeStep,
                                    const std::shared_ptr<RoadNetwork>& roadNetwork,
                                    const std::shared_ptr<Obstacle>& obstacleK,
                                    const std::shared_ptr<Obstacle>& obstacleP,
                                    const std::vector<std::shared_ptr<Obstacle>>& obstacles) = 0;


    /**
     * Virtual function for the constraint evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint constraintEvaluation(int timeStep,
                                            const std::shared_ptr<RoadNetwork>& roadNetwork,
                                            const std::shared_ptr<Obstacle>& obstacleK,
                                            const std::shared_ptr<Obstacle>& obstacleP,
                                            const std::vector<std::shared_ptr<Obstacle>>& obstacles) = 0;

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param evalType Evaluation type which should be used for statistical evaluation (robust, boolean, constraint).
     * @param timeStep Time step of interest.
     * @param roadNetwork RoadNetwork object.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param obstacles Pointers to all obstacles. This is an optional parameter.
     */
    bool statisticBooleanEvaluation(int timeStep,
                                    const std::shared_ptr<RoadNetwork>& roadNetwork,
                                    const std::shared_ptr<Obstacle>& obstacleK,
                                    const std::shared_ptr<Obstacle>& obstacleP,
                                    const std::vector<std::shared_ptr<Obstacle>>& obstacles);

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

    /**
     * Getter for predicate statistics (for all object instances).
     *
     * @return Struct containing statistics of this predicate.
     */
    [[nodiscard]] static const PredicateStatistics &getStatistics() ;

protected:
    PredicateParameters parameters;             //**< Struct containing parameters of all predicates. */
public:
    static PredicateStatistics statistics;      //**< Struct storing statistics of a predicate, e.g., average computation time,  number of calls, etc. */
};


#endif //ENV_MODEL_PREDICATE_H
