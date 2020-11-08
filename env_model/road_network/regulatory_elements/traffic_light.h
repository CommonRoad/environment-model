//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_LIGHT_H
#define ENV_MODEL_TRAFFIC_LIGHT_H

#include "../../auxiliaryDefs/structs.h"

class TrafficLight {
    public:
        TrafficLight();                                     // default constructor
        /*
         * setter functions
         */
        void setId(size_t num);
        void setCycle(const std::vector<CycleElement>& light_cycle);
        void setOffset(float of);
        void setDirection(TrafficLightDirection dir);
        void setActive(bool ac);
        void addCycleElement(CycleElement ce);

        /*
         * getter functions
         */
        [[nodiscard]] size_t getId() const;
        [[nodiscard]] std::vector<CycleElement> getCycle() const;
        [[nodiscard]] float getOffset() const;
        [[nodiscard]] TrafficLightDirection getDirection() const;
        [[nodiscard]] bool isActive() const;

        CycleElement getElementAtTime(float time);

    private:
        size_t id;                          //unique id
        std::vector<CycleElement> cycle;    //cycle of the traffic light
        float offset;                       //the offset for the cycle
        TrafficLightDirection direction;
        bool active{};
};

#endif //ENV_MODEL_TRAFFIC_LIGHT_H
