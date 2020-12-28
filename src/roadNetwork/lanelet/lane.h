//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_LANE_H
#define ENV_MODEL_LANE_H

#include "lanelet.h"
#include "geometry/curvilinear_coordinate_system.h"

using CurvilinearCoordinateSystem = geometry::CurvilinearCoordinateSystem;

/**
 * Class representing a lane.
 */
class Lane {
    public:
    /**
    * Constructor initializing contained lanelet, lanelet describing lane, and Curvilinear coordinate system.
     *
     * @param containedLanelets Lanelets contained in lane.
     * @param lanelet Lanelet object spanning lane.
     * @param ccs Curvilinear coordinate system object.
    */
        Lane(std::vector<std::shared_ptr<Lanelet>>  containedLanelets,
             Lanelet lanelet,
             CurvilinearCoordinateSystem ccs);

        /**
         * Getter for lanelet spanning lane.
         *
         * @return Lanelet object.
        */
        [[nodiscard]] Lanelet getLanelet() const;

        /**
         * Getter for curvilinear coordinate system using center line of lane as reference.
         *
         * @return Curvilinear coordinate system object.
        */
        [[nodiscard]] const CurvilinearCoordinateSystem &getCurvilinearCoordinateSystem() const;

        /**
         * Getter for lanelets contained in lane.
         *
         * @return List of pointers to lanelets.
        */
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
        [[nodiscard]] bool checkIntersection(const polygon_type &polygon_shape, ContainmentType intersection_type) const;

    private:
            std::vector<std::shared_ptr<Lanelet>> containedLanelets;    //**< list of pointers to lanelets constructing lane */
            Lanelet lanelet;                                            //**< lanelet representing lane */
            CurvilinearCoordinateSystem curvilinearCoordinateSystem;    //**< curvilinear coordinate system defined by lane */
};

#endif //ENV_MODEL_LANE_H
