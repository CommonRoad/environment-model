//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <memory> // for shared_ptr
#include <optional>
#include <unordered_set>
#include <vector> // for vector

#include <omp.h>

#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>
#include <commonroad_cpp/roadNetwork/road_network_config.h>
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
         std::shared_ptr<CurvilinearCoordinateSystem> ccs);

    /**
     * Constructor initializing contained lanelet, lanelet describing lane, and Curvilinear coordinate system.
     *
     * @param containedLanelets Lanelets contained in lane.
     * @param lanelet Lanelet object spanning lane.
     */
    Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet);

    ~Lane();

    /**
     * Getter for curvilinear coordinate system using center line of lane as reference.
     *
     * @return Curvilinear coordinate system object.
     */
    [[nodiscard]] const std::shared_ptr<CurvilinearCoordinateSystem> &getCurvilinearCoordinateSystem();

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
    [[nodiscard]] const std::unordered_set<size_t> &getContainedLaneletIDs() const;

    /**
     * Collects all successor lanelets within lane given a start lanelet.
     *
     * @param lanelet Starting lanelet.
     * @return List of pointers to succeeding lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>> getSuccessorLanelets(const std::shared_ptr<Lanelet> &lanelet) const;

    /**
     * Checks whether provided lanelet is part of lane.
     *
     * @param lanelet Pointer to lanelet object.
     * @return Boolean indicating whether lanelet is part of lane.
     */
    bool containsLanelet(const std::shared_ptr<Lanelet> &lanelet) const;

    /**
     * Checks whether lanelet with provided lanelet ID is part of lane.
     *
     * @param laneletId Lanelet ID.
     * @return Boolean indicating whether lanelet is part of lane.
     */
    bool containsLanelet(size_t laneletId) const;

    /**
     * Boolean indicating whether any provided lanelet is part of lane.
     *
     * @param lanelets List of pointers to lanelets.
     * @return Boolean indicating whether any lanelet is part of lane.
     */
    bool contains(std::vector<std::shared_ptr<Lanelet>> lanelets);

  private:
    mutable std::vector<std::shared_ptr<Lanelet>>
        containedLanelets; //**< list of pointers to lanelets constructing lane */
    mutable std::shared_ptr<CurvilinearCoordinateSystem>
        curvilinearCoordinateSystem;                //**< curvilinear coordinate system defined by lane */
    std::unordered_set<size_t> containedLaneletIds; //**< set of IDs of the lanelets constructing lane */

    omp_lock_t ccs_lock;
};
