#pragma once

#include <memory>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/auxiliaryDefs/timer.h"

#include "predicate_parameter_collection.h"

class Obstacle;
class World;

/**
 * Evaluation mode for set-based predictions of obstacles.
 *
 * For an obstacle with a set-based prediction, a predicate is evaluated on the occupancy, which covers many behaviors
 * of the obstacle. Which version is sound depends on how the predicate is used, e.g., on its polarity in a rule, so the
 * caller selects the version.
 */
enum class SetBasedMode {
    none, //**< no set-based evaluation */
    must, //**< holds only if it holds for all behaviors covered by the occupancy (robustness: minimum) */
    may   //**< holds if it holds for at least one behavior covered by the occupancy (robustness: maximum) */
};

/**
 * Interface for a predicate.
 */
class CommonRoadPredicate {
  public:
    /**
     * Default constructor for predicate class without parameter.
     *
     * @param vehicleDependent Boolean indicating whether predicate depends only on one vehicle
     */
    CommonRoadPredicate(bool vehicleDependent);

    virtual ~CommonRoadPredicate();

    CommonRoadPredicate(const CommonRoadPredicate &) = delete;

    /**
     * Virtual function for the boolean evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<Obstacle> &obstacleK,
                                   const std::shared_ptr<Obstacle> &obstacleP,
                                   const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                   bool setBased = false) = 0;

    /**
     * Boolean evaluation of a predicate with a must- or may-version for set-based predictions.
     * Predicates which provide only a single set-based version use it for both modes.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param mode Set-based evaluation mode.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<Obstacle> &obstacleK,
                                   const std::shared_ptr<Obstacle> &obstacleP,
                                   const std::vector<std::string> &additionalFunctionParameters, SetBasedMode mode);

    // FIXME: Make this internal if possible (friend function?)
    /**
     * Variant of booleanEvaluation without optional parameter.
     * FOR INTERNAL USE ONLY
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool simpleBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                 const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                                 const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                 bool setBased = false);

    /**
     * Virtual function for the robustness evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                    bool setBased = false) = 0;

    /**
     * Robustness evaluation of a predicate with a must- or may-version for set-based predictions.
     * Predicates which provide only a single set-based version use it for both modes.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param mode Set-based evaluation mode.
     * @return Real value indicating robustness of the predicate.
     */
    virtual double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::vector<std::string> &additionalFunctionParameters, SetBasedMode mode);

    /**
     * Virtual function for the constraint evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                            bool setBased = false) = 0;

    /**
     * Constraint evaluation of a predicate with a must- or may-version for set-based predictions.
     * Predicates which provide only a single set-based version use it for both modes.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param mode Set-based evaluation mode.
     * @return Constraints defined by the predicate.
     */
    virtual Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            SetBasedMode mode);

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param evaluationTimer Timer which measures computation time.
     * @param statistics Statistic of predicates.
     * @param obstacleP Pointer to the pth obstacle. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @param setBased Boolean indicating whether set-based evaluation should be used.
     * @return Boolean indicating satisfaction
     */
    bool statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Timer> &evaluationTimer,
                                    const std::shared_ptr<PredicateStatistics> &statistics,
                                    const std::shared_ptr<Obstacle> &obstacleP = {},
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                    bool setBased = false);

    /**
     * Function for the statistical evaluation of a predicate with a must- or may-version for set-based predictions.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param obstacleK Pointer to the kth obstacle.
     * @param evaluationTimer Timer which measures computation time.
     * @param statistics Statistic of predicates.
     * @param obstacleP Pointer to the pth obstacle.
     * @param additionalFunctionParameters Additional parameters.
     * @param mode Set-based evaluation mode.
     * @return Boolean indicating satisfaction
     */
    bool statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::shared_ptr<Timer> &evaluationTimer,
                                    const std::shared_ptr<PredicateStatistics> &statistics,
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::vector<std::string> &additionalFunctionParameters, SetBasedMode mode);

    /**
     * Getter for parameters.
     *
     * @return Struct containing all parameters.
     */
    [[nodiscard]] PredicateParameters &getParameters();

    /**
     * Setter for parameters.
     *
     * @param parameters Struct containing all parameters.
     */
    void setParameters(const PredicateParameters &parameters);

    /**
     * Returns whether predicate is vehicle dependent.
     *
     * @return Boolean indicating whether predicate is vehicle dependent.
     */
    [[nodiscard]] bool isVehicleDependent() const;

  protected:
    /**
     * Returns whether an obstacle is evaluated on its set-based prediction, i.e., on the occupancy at a future time
     * step.
     *
     * @param obstacle Obstacle of interest.
     * @param timeStep Time step of interest.
     * @param mode Set-based evaluation mode.
     * @return Boolean indicating whether the occupancy of the obstacle is evaluated.
     */
    static bool evaluatesOccupancy(const std::shared_ptr<Obstacle> &obstacle, size_t timeStep, SetBasedMode mode);

    PredicateParameters parameters; //**< Struct containing parameters of all predicates. */
    const bool vehicleDependent; //**< Boolean indicating whether predicate depends on one specific obstacle or two. */
};

extern std::map<std::string, std::shared_ptr<CommonRoadPredicate>> predicates; //**< List of all predicates **/
