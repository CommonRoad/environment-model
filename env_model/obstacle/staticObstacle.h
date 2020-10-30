/*
 * representation for static obstacles.
 */

#ifndef HEADER_STATICOBSTACLE
#define HEADER_STATICOBSTACLE

#include "obstacle.h"

class staticObstacle : public obstacle {
  public:
    /*
     * constructor
     */
    staticObstacle() : obstacle() {}
    /*
     * destructor
     */
    virtual ~staticObstacle() {}

    /*
     * setter functions
     */

    /*
     * getter functions
     */

    // determines if there is a possible intersection with the ego vehicle for the time horizon
    bool possibleIntersection(polygon_type *egoM1Polygon, timeStruct &timeInterval);

    // Implements pure virtual function from interface "obstacle"
    void computeOccupancyCore(std::vector<vehicularLanelet *> &vehLanelets,
                              std::vector<pedestrianLanelet*>& pedLanelets, std::vector<lane *> &lanes,
                              timeStruct &timeInterval, obstacle *egoVehicle, mpolygon_t &unionLanelets);

    // Empty implementation (but forced by obstacle parent class to build this function
    // Needed for dynamic obstacles
    // Todo Verify, that this structure is optimal (level 2 architecture change)
    void manageConstraints();

  private:
};

#endif
