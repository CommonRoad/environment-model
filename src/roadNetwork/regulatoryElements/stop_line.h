//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_STOP_LINE_H
#define ENV_MODEL_STOP_LINE_H

#include "traffic_sign.h"
#include "traffic_light.h"
#include "../../auxiliaryDefs/types_and_definitions.h"

/**
 * Class representing a CommonRoad stop line.
 */
class StopLine {
public:
    /**
     * Getter for start and end point of stop line.
     *
     * @return Start and end vertex.
     */
    [[nodiscard]] const std::vector<vertex> &getPoints() const;

    /**
     * Getter for traffic sign referenced by stop line.
     *
     * @return Pointer to traffic sign.
     */
    [[nodiscard]] std::shared_ptr<TrafficSign> getTrafficSign() const;

    /**
     * Getter for traffic light referenced by stop line.
     *
     * @return Pointer to traffic light.
     */
    [[nodiscard]] std::shared_ptr<TrafficLight> getTrafficLight() const;

    /**
     * Getter for line marking type of stop line.
     *
     * @return Line marking type.
     */
    [[nodiscard]] LineMarking getLineMarking() const;

    /**
     * Setter for start and end vertex of stop line.
     *
     * @param points Start and end vertex.
     */
    void setPoints(const std::vector<vertex> &points);

    /**
     * Setter for traffic sign referenced by stop line.
     *
     * @param trafficSign Pointer to traffic sign.
     */
    void setTrafficSign(std::shared_ptr<TrafficSign> trafficSign);

    /**
     * Setter for traffic light referenced by stop line.
     *
     * @param trafficLight Pointer to traffic light.
     */
    void setTrafficLight(std::shared_ptr<TrafficLight> trafficLight);

    /**
     * Setter for line marking of stop line.
     *
     * @param lineMarking Type of line marking.
     */
    void setLineMarking(LineMarking lineMarking);

private:
    std::vector<vertex> points;                     //**< start and end vertex of stop line */
    std::shared_ptr<TrafficSign> trafficSign;       //**< pointer to traffic sign referenced by stop line */
    std::shared_ptr<TrafficLight> trafficLight;     //**< pointer to traffic light referenced by stop line */
    LineMarking lineMarking;                        //**< type of line marking */
};

#endif //ENV_MODEL_STOP_LINE_H
