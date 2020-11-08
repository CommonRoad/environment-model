#ifndef HEADER_OBSTACLE
#define HEADER_OBSTACLE

#include "state.h"
#include "../auxiliaryDefs/structs.h"
#include "../auxiliaryDefs/types.h"
#include "../geometry/rectangle.h"
#include "../geometry/shape.h"
#include <map>

class Obstacle {
  public:
    // constructor
    explicit Obstacle(bool isStatic = false);
    Obstacle(const Obstacle &) = default;            // copy constructor
    Obstacle &operator=(const Obstacle &) = default; // copy assignment
    Obstacle(Obstacle &&) = default;                 // move constructor
    Obstacle &operator=(Obstacle &&) = default;      // move assignment
    virtual ~Obstacle() = default;                   // virtual destructor

    /*
     * setter functions
     */
    void setId(size_t num);
//    void addInLane(lane *l);
//    void setNewObst(bool val);
//    void useShapeAsRef(bool val);

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
//    const std::vector<lane *> &getInLane() const;
//    [[nodiscard]] std::vector<Lanelet *> getInLanelets(const std::vector<Lanelet> &lanelets, int timeStep);
//    [[nodiscard]] bool getUseShape() const;
    polygon_type getOccupancyPolygonShape(int timeStamp);
    [[nodiscard]] bool getIsStatic() const;


//    virtual void updateInLane(std::vector<lane *> &lanes);

    Shape &getGeoShape();

//  protected:

//    bool findLaneletsCorrespondingToObstacle(const std::vector<vehicularLanelet *> &lanelets,
//                                        std::vector<vehicularLanelet *> &inLanelet); // TODO: make return type clearer
//
//    std::vector<std::vector<occTypes>> occupancyMatrix; // occupancy
//    std::vector<lane *> inLanes;                        // lane, in which the Obstacle is located in
//    std::vector<vehicularLanelet *> inLanelets;         // lanelets, in which the Obstacle is located in

    /* defined in subclass */


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



//    virtual void updateInLanelets();

};

//#include "Obstacle.ipp"

ObstacleType matchObstacleTypeToString(const char *type);

#endif
