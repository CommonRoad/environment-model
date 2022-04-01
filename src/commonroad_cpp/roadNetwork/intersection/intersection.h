//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "incoming.h"

/**
 * Class representing an intersection.
 */
class Intersection {
  public:
    /**
     * Default constructor of intersection class.
     */
    Intersection() = default;

    /**
     * Constructor of intersection class.
     * @param intersectionId Id of intersection.
     * @param incomings List of references to incomings of intersection.
     * @param crossings List of references to crossing lanelets of intersection.
     */
    Intersection(size_t intersectionId, std::vector<std::shared_ptr<Incoming>> incomings,
                 std::vector<std::shared_ptr<Lanelet>> crossings);

    /**
     * Getter for intersection ID.
     *
     * @return ID of intersection.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for incomings belonging to intersection.
     *
     * @return List of pointers to incomings.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Incoming>> &getIncomings() const;

    /**
     * Getter for crossing lanelets belonging to intersection.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getCrossings() const;

    /**
     * Getter for intersection ID.
     *
     * @param num ID of intersection.
     */
    void setId(size_t num);

    /**
     * Setter for incomings.
     *
     * @param incs List of pointers to incomings.
     */
    void setIncomings(const std::vector<std::shared_ptr<Incoming>> &incs);

    /**
     * Adds and incoming to the intersection.
     *
     * @param incoming Pointer to incoming.
     */
    void addIncoming(const std::shared_ptr<Incoming> &incoming);

    /**
     * Setter for crossing lanelets belonging to intersection.
     *
     * @param cross List of pointers to lanelets.
     */
    void setCrossings(const std::vector<std::shared_ptr<Lanelet>> &cross);

    /**
     *
     * @return
     */
    const std::vector<std::shared_ptr<Lanelet>> &getMemberLanelets() const;

    /**
     *
     */
    void computeMemberLanelets();

  private:
    size_t id;                                        //**< ID of intersection. */
    std::vector<std::shared_ptr<Incoming>> incomings; //**< List of pointers to incomings belonging to intersection. */
    std::vector<std::shared_ptr<Lanelet>>
        crossings; //**< List of pointers to crossing lanelets belonging to intersection. */
    std::vector<std::shared_ptr<Lanelet>> memberLanelets; //**< List of lanelets belonging to intersection starting from
                                                          // incoming until outgoing. Crossings are not considered. */
};
