//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_OBSTACLE_H
#define ENV_MODEL_OBSTACLE_H

#include "../roadNetwork/lanelet/lane.h"
#include "../roadNetwork/road_network.h"
#include "../auxiliaryDefs/structs.h"
#include "../auxiliaryDefs/types_and_definitions.h"
#include "../geometry/rectangle.h"
#include "../geometry/shape.h"

#include "state.h"
#include <map>

class Obstacle {
  public:
    Obstacle() = default;
    Obstacle(int id,
             bool isStatic,
             const State &currentState,
             ObstacleType obstacleType,
             double vMax,
             double aMax,
             double aMaxLong,
             double aMinLong,
             double reactionTime,
             std::map<int, State> trajectoryPrediction,
             double length,
             double width);

    void setId(int num);
    void setIsStatic(bool isStatic);
    void setCurrentState(const State &currentState);
    void setObstacleType(ObstacleType type);
    void setVmax(double vmax);
    void setAmax(double amax);
    void setAmaxLong(double amax);
    void setAminLong(double amin);
    void setReactionTime(double tReact);
    void setReferenceLane(const std::vector<std::shared_ptr<Lane>>& possibleLanes, int timeStep);
    void setTrajectoryPrediction(const std::map<int, State> &trajPrediction);
    void setRectangleShape(double length, double width);

    void appendStateToTrajectoryPrediction(State state);
    void appendStateToHistory(State state);

    [[nodiscard]] int getId() const;
    [[nodiscard]] bool getIsStatic() const;
    [[nodiscard]] const State &getCurrentState() const;
    [[nodiscard]] State getStateByTimeStep(int timeStep) const;
    [[nodiscard]] ObstacleType getObstacleType() const;
    [[nodiscard]] double getVmax() const;
    [[nodiscard]] double getAmax() const;
    [[nodiscard]] double getAmaxLong() const;
    [[nodiscard]] double getAminLong() const;
    [[nodiscard]] double getReactionTime() const;
    [[nodiscard]] std::shared_ptr<Lane> getReferenceLane() const;
    [[nodiscard]] std::map<int, State> getTrajectoryPrediction() const;
    [[nodiscard]] int getTrajectoryLength();
    [[nodiscard]] polygon_type getOccupancyPolygonShape(int timeStamp);
    [[nodiscard]] Shape &getGeoShape();
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getOccupiedLanelets(
            const std::shared_ptr<RoadNetwork>& roadNetwork,
            int timeStep);

    /**
    * Computes the maximum longitudinal front position of obstacle (for rectangle shapes)
    *
    * @param timeStep time step of interest
    * @return longitudinal position of obstacle front
    */
    double frontS(int timeStep);

    /**
    * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes)
    *
    * @param timeStep time step of interest
    * @return longitudinal position of obstacle front
    */
    double rearS(int timeStep);

    /**
    * Computes the longitudinal position of obstacle based on Cartesian state and assigned lane
    *
    * @param timeStep time step of interest
    * @return longitudinal position of obstacle state
    */
    [[nodiscard]] double getLonPosition(int timeStep) const;

    /**
    * Computes the lateral position of obstacle based on Cartesian state and assigned lane
    *
    * @param timeStep time step of interest
    * @return lateral position of obstacle state
    */
    [[nodiscard]] double getLatPosition(int timeStep) const;

private:
    int id{};                                                                   //**< unique ID of lanelet */
    bool isStatic{false};                                                       //**< true if Obstacle is static */
    State currentState;                                                         //**< current state of obstacle */
    ObstacleType obstacleType{ObstacleType::unknown};                           //**< CommonRoad obstacle type */
    double vMax{};                                                              //**< maximum velocity of obstacle in m/s */
    double aMax{};                                                              //**< maximum absolute acceleration of obstacle in [m/s^2] */
    double aMaxLong{};                                                          //**< maximal longitudinal acceleration of obstacle in [m/s^2] */
    double aMinLong{};                                                          //**< minimal longitudinal acceleration of obstacle in [m/s^2] */
    double reactionTime{};                                                      //**< reaction time of obstacle in [s] */
    std::map<int, State> trajectoryPrediction{};                                //**< trajectory prediction of the obstacle */
    std::map<int, State> history{};                                             //**< previous states of the obstacle */
    Rectangle geoShape;                                                         //**< shape of the obstacle */
    std::map<int, std::vector<std::shared_ptr<Lanelet>>> occupiedLanelets{};    //**< map of time steps to lanelets occupied by the obstacle */
    std::shared_ptr<Lane> referenceLane{nullptr};                               //**< lane which is used as reference for curvilinear projection */
};

#endif //ENV_MODEL_OBSTACLE_H
