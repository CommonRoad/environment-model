//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <geometry/curvilinear_coordinate_system.h>
#include <memory> // for shared_ptr
#include <optional>
#include <vector> // for vector

#include "lanelet.h"

namespace geometry {
class CurvilinearCoordinateSystem;
}

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
    Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet,
         CurvilinearCoordinateSystem ccs);

    /**
     * Constructor initializing contained lanelet, lanelet describing lane, and Curvilinear coordinate system.
     *
     * @param containedLanelets Lanelets contained in lane.
     * @param lanelet Lanelet object spanning lane.
     */
    Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet);

    /**
     * Getter for curvilinear coordinate system using center line of lane as reference.
     *
     * @return Curvilinear coordinate system object.
     */
    [[nodiscard]] const CurvilinearCoordinateSystem &getCurvilinearCoordinateSystem();

    /**
     * Getter for lanelets contained in lane.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getContainedLanelets() const;

    /**
     * Getter for lanelet IDs contained in lane.
     *
     * @return Set of IDs of the lanelets contained in lane.
     */
    [[nodiscard]] const std::set<size_t> &getContainedLaneletIDs() const;

  private:
    mutable std::vector<std::shared_ptr<Lanelet>>
        containedLanelets; //**< list of pointers to lanelets constructing lane */
    mutable std::optional<CurvilinearCoordinateSystem>
        curvilinearCoordinateSystem;              //**< curvilinear coordinate system defined by lane */
    mutable std::set<size_t> containedLaneletIds; //**< set of IDs of the lanelets constructing lane */
};
