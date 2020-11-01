//
// Created by sebastian on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFICLIGHT_H
#define ENV_MODEL_TRAFFICLIGHT_H

#include "../../auxiliaryDefs/structs.h"

enum CycleElementType{red, green, yellow, red_yellow };
enum TrafficLightDirection{right, straight, left, leftStraight, straightRight, leftRight, all };

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
        void setId(size_t num);
        void setCycle(const std::vector<CycleElement>& light_cycle);
        void setOffset(int of);
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

        /*
         * functions
         */
        CycleElement getElementAtTime(float time);

    private:
        size_t id;                          //unique id
        std::vector<CycleElement> cycle;    //cycle of the traffic light
        float offset;                       //the offset for the cycle
        TrafficLightDirection direction;
        bool active{};
};

#endif //ENV_MODEL_TRAFFICLIGHT_H
