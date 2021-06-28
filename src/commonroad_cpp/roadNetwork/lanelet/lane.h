//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_LANE_H
#define ENV_MODEL_LANE_H

#include <memory>                                    // for shared_ptr
#include <vector>                                    // for vector

#include "lanelet.h"

namespace geometry {
class CurvilinearCoordinateSystem;
};

using CurvilinearCoordinateSystem = geometry::CurvilinearCoordinateSystem;

/**
 * Class representing a lane.
 */
class Lane : public Lanelet {
  public:
    /**
     * Constructor initializing contained lanelet, lanelet describing lane, and Curvilinear coordinate system.
     *
     * @param containedLanelets Lanelets contained in lane.
     * @param lanelet Lanelet object spanning lane.
     * @param ccs Curvilinear coordinate system object.
     */
    Lane(std::vector<std::shared_ptr<Lanelet>> containedLanelets, Lanelet lanelet, std::shared_ptr<CurvilinearCoordinateSystem> ccs);

    /**
     * Getter for curvilinear coordinate system using center line of lane as reference.
     *
     * @return Curvilinear coordinate system object.
     */
    [[nodiscard]] const std::shared_ptr<CurvilinearCoordinateSystem> &getCurvilinearCoordinateSystem() const;

    /**
     * Getter for lanelets contained in lane.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getContainedLanelets() const;

  private:
    std::vector<std::shared_ptr<Lanelet>> containedLanelets; //**< list of pointers to lanelets constructing lane */
    std::shared_ptr<CurvilinearCoordinateSystem> curvilinearCoordinateSystem; //**< curvilinear coordinate system defined by lane */
};

#endif // ENV_MODEL_LANE_H
