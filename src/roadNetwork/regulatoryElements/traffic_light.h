//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_LIGHT_H
#define ENV_MODEL_TRAFFIC_LIGHT_H

#include "../../auxiliaryDefs/structs.h"

/**
 * Class representing a CommonRoad traffic light.
 */
class TrafficLight {
public:
    /**
    * Default Constructor without parameters for a traffic light.
    */
    TrafficLight();

    /**
     * Setter for traffic light ID.
     *
     * @param num Traffic light ID.
     */
    void setId(int num);

    /**
     * Setter for traffic light cycle.
     *
     * @param light_cycle List of traffic light cycle elements.
     */
    void setCycle(const std::vector<CycleElement> &light_cycle);

    /**
     * Setter for traffic light offset.
     *
     * @param ofst Time offset.
     */
    void setOffset(int ofst);

    /**
     * Setter for traffic light direction.
     *
     * @param dir Direction of the traffic light.
     */
    void setDirection(TrafficLightDirection dir);

    /**
     * Setter for traffic light activity.
     *
     * @param ac Boolean indicating whether traffic light is active.
     */
    void setActive(bool ac);

    /**
     * Adds a cycle element to the traffic light.
     *
     * @param ce Traffic light cycle element.
     */
    void addCycleElement(CycleElement ce);

     /**
      * Getter for traffic light ID.
      *
      * @return ID of traffic light.
      */
    [[nodiscard]] int getId() const;

    /**
     * Getter for traffic light cycle.
     *
     * @return List of traffic light cycle elements.
     */
    [[nodiscard]] std::vector<CycleElement> getCycle() const;

    /**
     * Getter for traffic light offset.
     *
     * @return Time offset of traffic light.
     */
    [[nodiscard]] int getOffset() const;

    /**
     * Getter for traffic light direction.
     *
     * @return Direction of traffic light.
     */
    [[nodiscard]] TrafficLightDirection getDirection() const;

    /**
     * Getter for traffic light activity indicator.
     *
     * @return Boolean indicating whether traffic light is active.
     */
    [[nodiscard]] bool isActive() const;

    /**
     * Computes traffic light cycle element at a given time step.
     *
     * @param time Time step of interest.
     * @return Traffic light cycle element which is active at the provided time step.
     */
    CycleElement getElementAtTime(int time);

private:
    int id;                             //**< unique ID of traffic light */
    std::vector<CycleElement> cycle;    //**< cycle of traffic light */
    int offset;                         //**< time offset of traffic light */
    TrafficLightDirection direction;    //**< direction for which traffic light is valid */
    bool active{};                      //**< boolean indicating whether traffic sign is valid */
};

#endif //ENV_MODEL_TRAFFIC_LIGHT_H
