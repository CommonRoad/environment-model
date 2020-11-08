//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_OBSTACLE_H
#define ENV_MODEL_OBSTACLE_H

#include "state.h"
#include "../auxiliaryDefs/structs.h"
#include "../auxiliaryDefs/types.h"
#include "../geometry/rectangle.h"
#include "../geometry/shape.h"
#include <map>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::box<point_type> box;

class Obstacle {
  public:
    /*
     * setter functions
     */
    void setId(size_t num);

    void setVmax(double vmax);
    void setAmax(double amax);
    void setAmaxLong(double amax_long);
    void setAminLong(double amin_long);
    void setIsStatic(bool isStatic);
    void appendState(State state);
    void setCurrentState(const State &currentState);
    void setType(ObstacleType type);

    /*
     * getter functions
     */
    [[nodiscard]] double getVmax() const;
    [[nodiscard]] double getAmax() const;
    [[nodiscard]] double getAmaxLong() const;
    [[nodiscard]] double getAminLong() const;
    [[nodiscard]] size_t getId() const;
    [[nodiscard]] ObstacleType getType() const;
    [[nodiscard]] const State &getCurrentState() const;
    [[nodiscard]] bool getIsStatic() const;
    polygon_type getOccupancyPolygonShape(int timeStamp);
    Shape &getGeoShape();

  private:
    size_t id{}; // unique id
    bool isStatic{false}; // true if Obstacle is static
    State currentState;
    std::map<int, State> trajectoryPrediction{};
    std::map<int, State> history{};
    ObstacleType type;
    Rectangle geoShape;
    double v_max{};      // maximum velocity of the Obstacle in m/s
    double a_max{};      // maximum absolute acceleration of the Obstacle in m/s^2
    double a_max_long{}; // maximal longitudinal acceleration
    double a_min_long{}; // minimal longitudinal acceleration
};

#endif //ENV_MODEL_OBSTACLE_H
