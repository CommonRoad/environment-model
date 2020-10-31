#ifndef HEADER_OBSTACLE
#define HEADER_OBSTACLE

#include "../auxiliaryDefs/structs.h"
#include "../geometry/rectangle.h"
#include "../geometry/shape.h"
//#include "../lane/lane.h"
//#include "../lanelets/pedestrianLanelet.h"
//#include "../lanelets/vehicularLanelet.h"
#include "../road_network/lanelet/lanelet.h"
#include <variant>

//struct occTypes {
//    std::vector<lane *> forLane;
//    std::vector<polygon_type> vertices;
//    timeStruct timeInterval;
//    double minVelo;
//    double maxVelo;
//};

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
    void setId(const size_t &num);
    void setPosition(const double &x, const double &y);
    void setOrientation(const double &value);
//    void addInLane(lane *l);
    void setOccType(size_t type);
//    void setNewObst(bool val);
//    void useShapeAsRef(bool val);
    void setVelocity(double velo);
    void setAcceleration(double acc);
    void setVmax(double vmax);
    void setAmax(double amax);
    void setAmaxLong(double amax_long);
    void setAminLong(double amin_long);
    void setIsStatic(bool isStatic);

    /*
     * getter functions
     */
    [[nodiscard]] double getVelocity() const;
    [[nodiscard]] double getAcceleration() const;
    [[nodiscard]] double getVmax() const;
    [[nodiscard]] double getAmax() const;
    [[nodiscard]] double getAmaxLong() const;
    [[nodiscard]] double getAminLong() const;
    [[nodiscard]] size_t getId() const;
    [[nodiscard]] double getXpos() const;
    [[nodiscard]] double getYpos() const;
    [[nodiscard]] double getOrientation() const;
//    const std::vector<lane *> &getInLane() const;
//    const std::vector<vehicularLanelet *> &getInLanelets() const;
//    [[nodiscard]] bool getUseShape() const;
    const polygon_type getOccupancyPolygonShape();
    [[nodiscard]] bool getIsStatic() const;

//    virtual void updateInLane(std::vector<lane *> &lanes);

    virtual shape &getGeoShape();

  protected:

//    bool findLaneletsCorrespondingToObstacle(const std::vector<vehicularLanelet *> &lanelets,
//                                        std::vector<vehicularLanelet *> &inLanelet); // TODO: make return type clearer
//
//    std::vector<std::vector<occTypes>> occupancyMatrix; // occupancy
//    std::vector<lane *> inLanes;                        // lane, in which the Obstacle is located in
//    std::vector<vehicularLanelet *> inLanelets;         // lanelets, in which the Obstacle is located in

    /* defined in subclass */
    double v_max{};      // maximum velocity of the Obstacle in m/s
    double a_max{};      // maximum absolute acceleration of the Obstacle in m/s^2
    double a_max_long{}; // maximal longitudinal acceleration
    double a_min_long{}; // minimal longitudinal acceleration

  private:
    double xPosition{0.0};         // x-coordinate of the Obstacle
    double yPosition{0.0};         // y-coordinate of the Obstacle
    double orientation{};            // orientation of the Obstacle
    double velocity{0};           // scalar velocity of the Obstacle in m/s
    double acceleration{0};       // absoulte acceleration of the Obstacle in m/s^2
    bool isStatic{false}; // true if Obstacle is static
    rectangle geoShape;
    size_t occType{0};
    size_t id{}; // unique id

//    virtual void updateInLanelets();

};

//#include "Obstacle.ipp"

#endif
