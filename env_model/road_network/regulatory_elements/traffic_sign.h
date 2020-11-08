//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_SIGN_H
#define ENV_MODEL_TRAFFIC_SIGN_H

#include "../../auxiliaryDefs/structs.h"
#include "traffic_sign_element.h"

class TrafficSign {
public:
    TrafficSign();                                     // default constructor
    TrafficSign(const TrafficSign &) = default;            // copy constructor
    TrafficSign &operator=(const TrafficSign &) = default; // copy assignment
    TrafficSign(TrafficSign &&) = default;                 // move constructor
    TrafficSign &operator=(TrafficSign &&) = default;      // move assignment

    /*
     * setter functions*
     */
    void setId(size_t num);
    void addTrafficSignElement(const TrafficSignElement& sign_element);
    void setVirtualElement(bool virtualElement);
    void setTrafficSignElement(const std::vector<TrafficSignElement>& trafficSignElement);
    /*
     * getter functions
     */
    [[nodiscard]] size_t getId() const;
    [[nodiscard]] bool isVirtualElement() const;
    [[nodiscard]] std::vector<TrafficSignElement> getTrafficSignElement() const;



    /*
     * functions
     */

private:
    size_t id;                                  //unique id
    bool virtualElement;                       //artificially added element
    std::vector<TrafficSignElement> trafficSignElement;
};


#endif //ENV_MODEL_TRAFFIC_SIGN_H
