//
// Created by sebastian on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFICLIGHT_H
#define ENV_MODEL_TRAFFICLIGHT_H

#include "../../auxiliaryDefs/structs.h"

enum CycleElementType{red, green, yellow, red_yellow };

struct CycleElement{
    CycleElementType color;
    float duration;
};

class TrafficLight {
    public:
        TrafficLight();                                     // default constructor
        TrafficLight(const TrafficLight &) = default;            // copy constructor
        TrafficLight &operator=(const TrafficLight &) = default; // copy assignment
        TrafficLight(TrafficLight &&) = default;                 // move constructor
        TrafficLight &operator=(TrafficLight &&) = default;      // move assignment

        /*
         * setter functions
         */
        void setId(const size_t num);
        void setCycle(const std::vector<CycleElement>& light_cycle);
        void setOffset(const float of);

        /*
         * getter functions
         */
        size_t getId() const;
        std::vector<CycleElement> getCycle() const;
        float getOffset() const;

        /*
         * functions
         */
        int getPedestrianPriorityAtTime(float time);
        CycleElement getElementAtTime(float time);

    private:
        size_t id;                          //unique id
        std::vector<CycleElement> cycle;    //cycle of the traffic light
        float offset;                       //the offset for the cycle
};


#endif //ENV_MODEL_TRAFFICLIGHT_H
