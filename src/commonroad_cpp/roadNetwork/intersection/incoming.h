//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

/**
 * Class representing an incoming of an intersection.
 */
class Incoming {
  public:
    /**
     * Default constructor.
     */
    Incoming() = default;

    /**
     * Constructor of incoming.
     *
     * @param incomingId Incoming ID.
     * @param incomingLanelets Reference to lanelets belonging to incoming.
     * @param isLeftOf Reference incoming orientated left.
     * @param straightOutgoings Reference to straight outgoing lanelets.
     * @param leftOutgoings Reference to left outgoing lanelets.
     * @param rightOutgoings Reference to right outgoing lanelets.
     * @param oncomings Reference to oncoming lanelets.
     */
    Incoming(size_t incomingId, std::vector<std::shared_ptr<Lanelet>> incomingLanelets,
             std::shared_ptr<Incoming> isLeftOf, std::vector<std::shared_ptr<Lanelet>> straightOutgoings,
             std::vector<std::shared_ptr<Lanelet>> leftOutgoings, std::vector<std::shared_ptr<Lanelet>> rightOutgoings,
             std::vector<std::shared_ptr<Lanelet>> oncomings);

    /**
     * Getter for incoming ID.
     *
     * @return Id of incoming.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for lanelets belonging to incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getIncomingLanelets() const;

    /**
     * Getter for successor right lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsRight() const;

    /**
     * Getter for successor straight lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsStraight() const;

    /**
     * Getter for successor left lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsLeft() const;

    /**
     * Getter for incoming which is left of this incoming.
     *
     * @return Pointer to incoming.
     */
    [[nodiscard]] const std::shared_ptr<Incoming> &getIsLeftOf() const;

    /**
     * Setter of incoming ID.
     *
     * @param incomingId Id of incoming.
     */
    void setId(size_t incomingId);

    /**
     * Setter for lanelets belonging to incoming.
     *
     * @param incomingLanelet List of pointers to lanelets.
     */
    void setIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>> &incomingLanelets);

    /**
     * Setter for incoming which is left of this incoming.
     *
     * @param isLeftOf Pointer to incoming.
     */
    void setIsLeftOf(const std::shared_ptr<Incoming> &isLeftOf);

    /**
     * Getter for straight outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getStraightOutgoings() const;

    /**
     * Setter for straight outgoing lanelets of incoming.
     *
     * @param straightOutgoings List of pointers to lanelets.
     */
    void setStraightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &straightOutgoings);

    /**
     * Getter for left outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLeftOutgoings() const;

    /**
     * Setter for left outgoing lanelets of incoming.
     *
     * @param leftOutgoings List of pointers to lanelets.
     */
    void setLeftOutgoings(const std::vector<std::shared_ptr<Lanelet>> &leftOutgoings);

    /**
     * Getter for right outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getRightOutgoings() const;

    /**
     * Setter for right outgoing lanelets of incoming.
     *
     * @param rightOutgoings List of pointers to lanelets.
     */
    void setRightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &rightOutgoings);

    /**
     * Getter for oncoming lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getOncomings() const;

    /**
     * Setter for oncoming lanelets of incoming.
     *
     * @param oncomings List of pointers to lanelets.
     */
    void setOncomings(const std::vector<std::shared_ptr<Lanelet>> &oncomings);

  private:
    size_t id;
    std::vector<std::shared_ptr<Lanelet>> incomingLanelets; //**< set of pointers to lanelets belonging to incoming */
    std::shared_ptr<Incoming> isLeftOf; //**< pointer to incoming which is left */
    std::vector<std::shared_ptr<Lanelet>>
        straightOutgoings; //**< set of pointers to straight outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>>
        leftOutgoings; //**< set of pointers to left outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>>
        rightOutgoings; //**< set of pointers to right outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>> oncomings; //**< set of pointers to oncoming lanelets of this incoming */
};
