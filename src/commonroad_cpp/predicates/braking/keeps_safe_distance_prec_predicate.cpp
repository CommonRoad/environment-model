#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

double KeepsSafeDistancePrecPredicate::computeSafeDistance(double velocityK, double velocityP, double minAccelerationK,
                                                           double minAccelerationP, double tReact) {
    if (minAccelerationK >= 0 or minAccelerationP >= 0)
        throw std::logic_error("Acceleration is not negative!");
    return pow(velocityP, 2) / (-2 * std::abs(minAccelerationP)) -
           pow(velocityK, 2) / (-2 * std::abs(minAccelerationK)) + velocityK * tReact;
}

bool KeepsSafeDistancePrecPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters, setBased) > 0;
}

bool KeepsSafeDistancePrecPredicate::booleanEvaluation(double lonPosK, double lonPosP, double velocityK,
                                                       double velocityP, double minAccelerationK,
                                                       double minAccelerationP, double tReact, double lengthK,
                                                       double lengthP) {
    return robustEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK, minAccelerationP, tReact, lengthK,
                            lengthP) > 0;
}

Constraint KeepsSafeDistancePrecPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    double aMinK{obstacleK->getAminLong()};
    double aMinP{obstacleP->getAminLong()};

    return {obstacleP->rearS(
                timeStep,
                obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem(),
                setBased) -
            0.5 * dynamic_cast<Rectangle &>(obstacleK->getGeoShape()).getLength() -
            computeSafeDistance(obstacleK->getVelocity(timeStep, setBased, true),
                                obstacleP->getVelocity(timeStep, setBased, true), aMinK, aMinP,
                                obstacleK->getReactionTime())};
}

Constraint KeepsSafeDistancePrecPredicate::constraintEvaluation(double lonPosP, double velocityK, double velocityP,
                                                                double minAccelerationK, double minAccelerationP,
                                                                double tReact, double lengthK, double lengthP) {
    return {lonPosP - 0.5 * lengthP - 0.5 * lengthK -
            computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact)};
}

double KeepsSafeDistancePrecPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters,
                                                        bool setBased) {
    double aMinK{obstacleK->getAminLong()};
    double aMinP{obstacleP->getAminLong()};

    double dSafe{computeSafeDistance(obstacleK->getVelocity(timeStep, false),
                                     obstacleP->getVelocity(timeStep, setBased, true), aMinK, aMinP,
                                     obstacleK->getReactionTime())};
    double deltaS{obstacleP->rearS(
                      timeStep,
                      obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem(),
                      setBased) -
                  obstacleK->frontS(world->getRoadNetwork(), timeStep)};

    // if pth vehicle is not in front of the kth vehicle, safe distance is not applicable -> return positive
    // robustness; collision must be checked separately
    if (deltaS < 0)
        return std::abs(deltaS);
    const double param0 = std::stod(additionalFunctionParameters.at(0));
    if (deltaS - param0 < 0)
        return std::min(deltaS - param0, deltaS - dSafe);
    return (deltaS - dSafe);
}

bool KeepsSafeDistancePrecPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       SetBasedMode mode) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters, mode) > 0;
}

double KeepsSafeDistancePrecPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters,
                                                        SetBasedMode mode) {
    if (!evaluatesOccupancy(obstacleP, timeStep, mode))
        return robustEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters,
                                mode != SetBasedMode::none);
    const bool must{mode == SetBasedMode::must};
    const auto ccsK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem()};
    const double frontK{obstacleK->frontS(world->getRoadNetwork(), timeStep)};
    const double rearP{obstacleP->rearS(timeStep, ccsK, true)};
    const double frontP{obstacleP->frontS(timeStep, ccsK, true)};

    // Covered behaviors behind the front of the kth vehicle satisfy the predicate (safe distance not applicable;
    // collision must be checked separately). must: the whole occupancy is behind it; may: any part of it is.
    if (must and frontP < frontK)
        return frontK - frontP;
    if (!must and rearP < frontK)
        return frontK - rearP;

    // The robustness increases with the distance and the velocity of the pth vehicle, so the must-version uses the
    // rear of the occupancy and the lowest velocity, and the may-version uses the front and the highest velocity.
    const double deltaS{(must ? rearP : frontP) - frontK};
    const double dSafe{computeSafeDistance(obstacleK->getVelocity(timeStep, false),
                                           obstacleP->getVelocity(timeStep, true, must), obstacleK->getAminLong(),
                                           obstacleP->getAminLong(), obstacleK->getReactionTime())};
    const double param0 = std::stod(additionalFunctionParameters.at(0));
    if (deltaS - param0 < 0)
        return std::min(deltaS - param0, deltaS - dSafe);
    return deltaS - dSafe;
}

double KeepsSafeDistancePrecPredicate::robustEvaluation(double lonPosK, double lonPosP, double velocityK,
                                                        double velocityP, double minAccelerationK,
                                                        double minAccelerationP, double tReact, double lengthK,
                                                        double lengthP) {
    double dSafe{computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact)};
    double deltaS = (lonPosP - 0.5 * lengthP) - (lonPosK + 0.5 * lengthK);
    // if pth vehicle is not in front of the kth vehicle, safe distance is not applicable -> return positive robustness
    if (deltaS < 0)
        return std::abs(deltaS);
    return (deltaS - dSafe);
}

KeepsSafeDistancePrecPredicate::KeepsSafeDistancePrecPredicate() : CommonRoadPredicate(true) {}
