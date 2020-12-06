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
        /*
         * setter functions*
         */
        void setId(int num);
        void addTrafficSignElement(const TrafficSignElement& sign_element);
        void setVirtualElement(bool virtualElement);
        void setTrafficSignElement(const std::vector<TrafficSignElement>& trafficSignElement);
        /*
         * getter functions
         */
        [[nodiscard]] int getId() const;
        [[nodiscard]] bool isVirtualElement() const;
        [[nodiscard]] std::vector<TrafficSignElement> getTrafficSignElement() const;

    private:
        int id;                                  //unique id
        bool virtualElement;                       //artificially added element
        std::vector<TrafficSignElement> trafficSignElement;
};


#endif //ENV_MODEL_TRAFFIC_SIGN_H
