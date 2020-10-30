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

class obstacle {
  public:
    // constructor
    obstacle(const bool isStatic = false) {
//        occupancyMatrix = std::vector<std::vector<occTypes>>{};
//        inLanes = std::vector<lane *>{};
//        reachableLanes = std::vector<size_t>{};
        setIsStatic(isStatic);
    }
    obstacle(const obstacle &) = default;            // copy constructor
    obstacle &operator=(const obstacle &) = default; // copy assignment
    obstacle(obstacle &&) = default;                 // move constructor
    obstacle &operator=(obstacle &&) = default;      // move assignment
    virtual ~obstacle() = default;                   // virtual destructor

    /*
     * setter functions
     */
    void setId(const size_t &num);
    void setPosition(const double &x, const double &y);
    void setOrientation(const double &value);
    void setOrientationError(const double &value);
//    void addInLane(lane *l);
    void setOccType(size_t type);
    void setTimeStamp(double stamp);
//    void setOccupancyMatrix(const std::vector<std::vector<occTypes>> &occMatrix);
//    void setOccupancyMatrix(const std::vector<std::vector<occTypes>> &&occMatrix);
    void setNewObst(bool val);
    void useShapeAsRef(bool val);
    void setVelocity(double velo);
    void setAcceleration(double acc);
    void setAccelerationError(double acc_error);
    void setVmax(double vmax);
    void setAmax(double amax);
    void setAmaxLong(double amax_long);
    void setAminLong(double amin_long);
    void setVelocityError(double v_error);
    void setOffset(double val);
//    void setReachableLanes(std::vector<lane *> lanes);
    void setViolatedCvmax(bool val);
    void setViolatedCamax(bool val);
    void setIsStatic(bool isStatic);

    /*
     * getter functions
     */
    [[nodiscard]] double getVelocity() const;
    [[nodiscard]] double getAcceleration() const;
    [[nodiscard]] double getAccelerationError() const;
    [[nodiscard]] double getVmax() const;
    [[nodiscard]] double getAmax() const;
    [[nodiscard]] double getAmaxLong() const;
    [[nodiscard]] double getAminLong() const;
    [[nodiscard]] double getVelocityError() const;
    [[nodiscard]] double getOffset() const;
    std::vector<size_t> getReachableLanes();
    [[nodiscard]] bool getViolatedCvmax() const;
    [[nodiscard]] bool getViolatedCamax() const;
    [[nodiscard]] size_t getId() const;
    [[nodiscard]] double getXpos() const;
    [[nodiscard]] double getYpos() const;
    [[nodiscard]] double getOrientation() const;
    [[nodiscard]] double getOrientationError() const;
//    const std::vector<lane *> &getInLane() const;
//    const std::vector<vehicularLanelet *> &getInLanelets() const;
//    const std::vector<std::vector<occTypes>> &getOccupancyMatrix() const;
//    const std::vector<std::vector<occTypes>> *getOccupancyMatrixPtr();
    [[nodiscard]] size_t getOccType() const;
    [[nodiscard]] double getTimeStamp() const;
    [[nodiscard]] bool getNewObst() const;
    [[nodiscard]] bool getUseShape() const;
    const polygon_type getOccupancyPolygonShape();
    [[nodiscard]] bool getIsStatic() const;

//    virtual void updateInLane(std::vector<lane *> &lanes);

    uint8_t updateProperties(const std::map<std::string, std::variant<bool, float>> &UpdateMap);

    /*
     * Virtual methods
     */
    // predicts the occupancy of the obstacle on the given map for the given time interval
//    virtual void computeOccupancyCore(const std::vector<vehicularLanelet *> &vehLanelets,
//                                      const std::vector<pedestrianLanelet *> &pedLanelets, std::vector<lane *> &lanes,
//                                      timeStruct &timeInterval, class EgoVehicle *egoVehicle,
//                                      mpolygon_t &unionLanelets);
//    // TODO: is this second computeOccupancyCore still needed
//    virtual void computeOccupancyCore(const std::vector<vehicularLanelet *> &lanelets, std::vector<lane *> &lanes,
//                                      timeStruct &timeInterval, class EgoVehicle *egoVehicle,
//                                      mpolygon_t &unionLanelets);

    // determine if obstacle is relevant for the occupancy calculation
//    virtual bool possibleIntersection(polygon_type *egoM1Polygon, timeStruct &timeInterval);

    // checks if assumptions for constraints are still valid for the obstacle
//    virtual void manageConstraints();

    // return specific shape type for sub obstacle types
    virtual shape &getGeoShape();

    // Update the obstacle's properties according to an update map.
    virtual uint8_t updateProperty(const std::string &name, const std::variant<bool, float> &value);

  protected:
    /**
     * @brief      Find lanelets where the obstacle has approximately the same or 180 degrees moved orientation as the
     lanelet.
     *
     * @details    This function assumes the obstacle intersects the lanelets in the lanelets vector.
                       The orientation of the obstacle is calculated, as is the orientation of the lanelets in the
     lanelets vector. Comparing these orientation with some margin it is determined if the obstacle is in some lanelets
     with the same orientation or is in some lanelets where the orientation differs by around 180°, i.e. the obstacle
     moving in the wrong direction (might happen e.g. at overtaking). The
     *
     * @param      lanelets The lanelets where the obstacle might be in.
     *
     * @param      inLanelet The vector which contains the lanelets where the obstacle is in.
     *
     * @param      obj The obstacle for which to determine corresponding lanelets it is in.
     *
     * @return     return type
     */
//    bool
//    findLaneletsCorrespondingToObstacle(const std::vector<vehicularLanelet *> &lanelets,
//                                        std::vector<vehicularLanelet *> &inLanelet); // TODO: make return type clearer
//
//    std::vector<std::vector<occTypes>> occupancyMatrix; // occupancy
//    std::vector<lane *> inLanes;                        // lane, in which the obstacle is located in
//    std::vector<vehicularLanelet *> inLanelets;         // lanelets, in which the obstacle is located in
    bool useShape; // if set the whole shape is used for inferring the current lanes instead of a single center point

    /* defined in subclass */
    double v_max;      // maximum velocity of the obstacle in m/s
    double a_max;      // maximum absolute acceleration of the obstacle in m/s^2
    double a_max_long; // maximal longitudinal acceleration
    double a_min_long; // minimal longitudinal acceleration

    // counting number of violations
    bool violated_cvmax{false};
    bool violated_camax{false};

  private:
    double xPosition{0.0};         // x-coordinate of the obstacle
    double yPosition{0.0};         // y-coordinate of the obstacle
    double orientation;            // orientation of the obstacle
    double orientation_error{0.0}; // measurment uncertainty regarding orientation

    double timeStamp{0};
    double velocity{0};           // scalar velocity of the obstacle in m/s
    double velocity_error{0.0};   // measurement uncertainty regarding velocity
    double acceleration{0};       // absoulte acceleration of the obstacle in m/s^2
    double acceleration_error{0}; // measurement uncertainty regarding absoulute acceleration
    double offset{0};     // delta_t; used if state of last measurement is preferred instead of the current state
    bool newObst{false};  // true, if the obstacle appears for the first time
    bool isStatic{false}; // true if obstacle is static
    rectangle geoShape;
    std::vector<size_t> reachableLanes; // ids of the reachable lanes
    size_t occType{0};
    size_t id; // unique id

//    virtual void updateInLanelets();

    /**
     * @brief      Finds all tracks in which the obstacle is positioned in.
     *
     * @param      tracks All the tracks to check through.
     *
     * @return     All the tracks the obstacle is positioned in (determined by shape or only 2D point).
     */
//    template <typename _T_ptr> std::vector<_T_ptr> tracksAtObjPositionOrShape(const std::vector<_T_ptr> &tracks);

    /**
     * @brief      Find all tracks the obstacle is part of.
     *
     * @details    First find all tracks the obstacle is positioned in and then check for the tracks which orientiation
     aligns approximately with the orientation of the obstacle.

     *
     * @param      tracks All the tracks to check through.
     *
     * @return     All the tracks the obstacle is part of.
     */
//    template <typename _T_ptr> std::vector<_T_ptr> getInTracks(const std::vector<_T_ptr> &tracks);
};

//#include "obstacle.ipp"

#endif
