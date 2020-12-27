//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_LANE_H
#define ENV_MODEL_LANE_H

#include "lanelet.h"
#include "geometry/curvilinear_coordinate_system.h"

typedef geometry::CurvilinearCoordinateSystem CurvilinearCoordinateSystem;

class Lane {
public:
    Lane(std::vector<std::shared_ptr<Lanelet>>  containedLanelets, Lanelet lanelet, CurvilinearCoordinateSystem ccs);
    ~Lane() = default;
    Lane(const Lane &other) = default;
    Lane& operator=(Lane const& other) { };
    Lane& operator=(Lane && other) = default;

    [[nodiscard]] Lanelet getLanelet() const;
    [[nodiscard]] const CurvilinearCoordinateSystem &getCurvilinearCoordinateSystem() const;
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getContainedLanelets() const;

    /**
    * Given a polygon, checks whether the polygon intersects with the lanelet given an intersection category.
    * Forwards the evaluation to the checkIntersection function of the lanelet class.
    *
    * @param polygon_shape boost polygon
    * @param intersection_type specifies whether shape can be partially occupied by lanelet
    *  or must be completely occupied
    * @return boolean indicating whether lanelet is occupied
    */
    [[nodiscard]] bool checkIntersection(const polygon_type &polygon_shape, int intersection_type) const;

private:
        std::vector<std::shared_ptr<Lanelet>> containedLanelets;    //**< list of pointers to lanelets constructing lane */
        Lanelet lanelet;                                            //**< lanelet representing lane */
        CurvilinearCoordinateSystem curvilinearCoordinateSystem;    //**< curvilinear coordinate system defined by lane */
};

#endif //ENV_MODEL_LANE_H
