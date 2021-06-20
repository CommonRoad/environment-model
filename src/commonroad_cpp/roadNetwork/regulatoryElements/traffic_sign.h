//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_SIGN_H
#define ENV_MODEL_TRAFFIC_SIGN_H

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "traffic_sign_element.h"

/**
 * Class representing a CommonRoad traffic sign.
 */
class TrafficSign {
  public:
    /**
     * Constructor of traffic sign.
     * @param id Traffic sign id.
     * @param traffic_sign_element List of traffic sign elements assigned to traffic sign.
     * @param position Position of traffic sign.
     * @param virtual_element Boolean indicating whether traffic sign is virtual.
     */
    TrafficSign(size_t id, std::vector<std::shared_ptr<TrafficSignElement>> traffic_sign_element,
                const vertex &position, bool virtual_element = false);

    /**
     * Default constructor of traffic sign.
     */
    TrafficSign() = default;

    /**
     * Setter for the ID of a traffic sign.
     *
     * @param num Traffic sign ID.
     */
    void setId(size_t num);

    /**
     * Adds traffic sign element to traffic sign.
     *
     * @param sign_element Traffic sign element.
     */
    void addTrafficSignElement(const std::shared_ptr<TrafficSignElement> &sign_element);

    /**
     * Setter for virtual indicator.
     *
     * @param virtualElement Boolean value indicating whether traffic sign is virtual.
     */
    void setVirtualElement(bool virtualElement);

    /**
     * Setter for traffic sign elements.
     *
     * @param trafficSignElement List of traffic sign elements.
     */
    void setTrafficSignElement(const std::vector<std::shared_ptr<TrafficSignElement>> &trafficSignElement);

    /**
     * Setter for traffic sign position.
     *
     * @param pos Position of traffic sign.
     */
    void setPosition(vertex pos);

    /**
     * Getter for ID of traffic sign.
     *
     * @return
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for virtual indicator.
     *
     * @return Boolean indicating whether traffic sign is virtual.
     */
    [[nodiscard]] bool isVirtualElement() const;

    /**
     * Getter for traffic sign elements.
     *
     * @return List of traffic sign elements.
     */
    [[nodiscard]] std::vector<std::shared_ptr<TrafficSignElement>> getTrafficSignElements() const;

    /**
     * Getter for traffic sign position.
     *
     * @return Position of traffic sign.
     */
    [[nodiscard]] vertex getPosition() const;

  private:
    size_t id; //**< unique ID of traffic sign */
    std::vector<std::shared_ptr<TrafficSignElement>>
        trafficSignElement; //**< list of traffic sign elements represented by traffic sign */
    vertex position{};      //**< position of traffic sign */
    bool virtualElement;    //**< indicator whether traffic sign is artificially added */
};

#endif // ENV_MODEL_TRAFFIC_SIGN_H
