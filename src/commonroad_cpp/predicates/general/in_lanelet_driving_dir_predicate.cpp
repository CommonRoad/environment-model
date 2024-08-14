#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/predicates/general/is_of_type_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/predicates/general/in_lanelet_driving_dir_predicate.h>

bool InLaneletDrivingDirPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters) {
    double theta = obstacleK->getStateByTimeStep(timeStep)->getGlobalOrientation();

    if (!obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
             ->getCurvilinearCoordinateSystem()
             ->cartesianPointInProjectionDomain(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                                obstacleK->getStateByTimeStep(timeStep)->getYPosition()))
        return false;
    obstacleK->convertPointToCurvilinear(world->getRoadNetwork(), timeStep);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);

    return std::any_of(lanelets.begin(), lanelets.end(), [&, theta](const std::shared_ptr<Lanelet> &lanelet) {
        double lanelet_orientation =
            lanelet->getOrientationAtPosition(obstacleK->getStateByTimeStep(timeStep)->getLonPosition(),
                                              obstacleK->getStateByTimeStep(timeStep)->getLatPosition());

        // Calculate the difference between the kth vehicle orientation and the lanelet orientation.
        double angle_diff =
            static_cast<double>(std::abs(theta) >= std::abs(lanelet_orientation)) * (theta - lanelet_orientation) +
            static_cast<double>(std::abs(theta) < std::abs(lanelet_orientation)) * (lanelet_orientation - theta);

        IsOfTypePredicate isVehicle;

        // Check whether the vehicle type of the kth vehicle is allowed to use the lanelet in one direction.
        std::set<ObstacleType> usersOneWay = lanelet->getUsersOneWay();
        bool oneWay = std::any_of(usersOneWay.begin(), usersOneWay.end(), [&](const ObstacleType &user) {
            return user == obstacleK->getObstacleType() or
                   isVehicle.booleanEvaluation(timeStep, world, obstacleK, obstacleP, {"vehicle"});
        });

        // Check whether the vehicle type of the kth vehicle is allowed to use the lanelet in both directions.
        std::set<ObstacleType> usersBidirectional = lanelet->getUsersBidirectional();
        bool bidirectional =
            std::any_of(usersBidirectional.begin(), usersBidirectional.end(), [&](const ObstacleType &user) {
                return user == obstacleK->getObstacleType() or
                       isVehicle.booleanEvaluation(timeStep, world, obstacleK, obstacleP, {"vehicle"});
            });

        // The kth vehicle is driving in the direction of travel of the lanelet if it is allowed to only drive in
        // one direction and the orientation difference is smaller than pi, or it is allowed to drive in both
        // directions.
        return ((std::abs(angle_diff) < M_PI / 2 && oneWay) || bidirectional);
    });
}

double InLaneletDrivingDirPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InLaneletDrivingDirPredicate does not support robust evaluation!");
}

Constraint InLaneletDrivingDirPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InLaneletDrivingDirPredicate does not support constraint evaluation!");
}
InLaneletDrivingDirPredicate::InLaneletDrivingDirPredicate() : CommonRoadPredicate(false) {}
