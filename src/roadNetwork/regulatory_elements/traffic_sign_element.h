//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_SIGN_ELEMENT_H
#define ENV_MODEL_TRAFFIC_SIGN_ELEMENT_H

#include "../../auxiliaryDefs/structs.h"

class TrafficSignElement {
    public:
        explicit TrafficSignElement(std::string id);
        TrafficSignElement(const TrafficSignElement &) = default;            // copy constructor

        [[nodiscard]] std::string getId() const;
        [[nodiscard]] std::vector<std::string> getAdditionalValue() const;

        void addAdditionalValue(const std::string& additionalValue); //additional values, e.g. allowed time, weight

    private:
            std::string id;                                  //official national traffic sign id of a country
            std::vector<std::string> additional_value;       //list of additional values classifying traffic sign, e.g., velocity, weight, time

};

#endif //ENV_MODEL_TRAFFIC_SIGN_ELEMENT_H
