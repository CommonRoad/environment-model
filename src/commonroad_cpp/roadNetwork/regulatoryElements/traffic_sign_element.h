//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <string>
#include <vector>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include <memory>
#include <vector>

/**
 * Class representing a CommonRoad traffic sign element.
 */
class TrafficSignElement {
  public:
    /**
     * Constructor of traffic sign element.
     *
     * @param id ID of traffic sign element. Note this ID corresponds to the official national traffic sign ID of a
     * country.
     */
    explicit TrafficSignElement(std::string id, std::vector<std::string> values = {});

    /**
     * Copy constructor of traffic sign element.
     *
     */
    TrafficSignElement(const TrafficSignElement &) = default;

    /**
     * Getter for ID of traffic sign element.
     *
     * @return ID of traffic sign element.
     */
    [[nodiscard]] std::string getId() const;

    /**
     * Getter for additional values of traffic sign element.
     *
     * @return Additional value.
     */
    [[nodiscard]] std::vector<std::string> getAdditionalValues() const;

    /**
     * Adds an additional value to a traffic sign element.
     *
     * @return Additional value.
     */
    void addAdditionalValue(const std::string &value);

    /**
     * Adds a list of additional values to a traffic sign element.
     *
     * @return Additional value.
     */
    void setAdditionalValues(const std::vector<std::string> &values);

    /**
     * Converts German traffic sign ID to string.
     *
     * @param signId Traffic sign ID from corresponding enum.
     * @return String which corresponds to traffic sign ID.
     */
    static std::string convertGermanTrafficSignIdToString(TrafficSignTypes signId);

  private:
    std::string id; //**< official national traffic sign ID of a country */
    std::vector<std::string>
        additionalValues; //**< list of additional values classifying traffic sign, e.g., velocity, weight, time */
};
