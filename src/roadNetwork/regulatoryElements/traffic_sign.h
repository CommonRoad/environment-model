//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_SIGN_H
#define ENV_MODEL_TRAFFIC_SIGN_H

#include "../../auxiliaryDefs/structs.h"
#include "traffic_sign_element.h"

/**
 * Class representing a CommonRoad traffic sign.
 */
class TrafficSign {
public:
    /**
     * Setter for the ID of a traffic sign.
     *
     * @param num Traffic sign ID.
     */
    void setId(int num);

    /**
     * Adds traffic sign element to traffic sign.
     *
     * @param sign_element Traffic sign element.
     */
    void addTrafficSignElement(const TrafficSignElement &sign_element);

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
    void setTrafficSignElement(const std::vector<TrafficSignElement> &trafficSignElement);

    /**
     * Getter for ID of traffic sign.
     *
     * @return
     */
    [[nodiscard]] int getId() const;

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
    [[nodiscard]] std::vector<TrafficSignElement> getTrafficSignElement() const;

private:
    int id;                                                 //**< unique ID of traffic sign */
    bool virtualElement;                                    //**< indicator whether traffic sign is artificially added */ //
    std::vector<TrafficSignElement> trafficSignElement;     //**< list of traffic sign elements represented by traffic sign */ //
};


#endif //ENV_MODEL_TRAFFIC_SIGN_H
