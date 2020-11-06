//
// Created by sebastian on 23.10.20.
//

#ifndef ENVIRONMENT_MODEL_LANELET_H
#define ENVIRONMENT_MODEL_LANELET_H

#include "../../auxiliaryDefs/structs.h"
#include "../regulatory_elements/traffic_light.h"
#include "../regulatory_elements/traffic_sign.h"

class Lanelet {
    public:
        Lanelet();                                     // default constructor
        Lanelet(const Lanelet &) = default;            // copy constructor
        Lanelet &operator=(const Lanelet &) = default; // copy assignment
        Lanelet(Lanelet &&) = default;                 // move constructor
        Lanelet &operator=(Lanelet &&) = default;      // move assignment
        virtual ~Lanelet() = default;                  // virtual destructor

        /*
         * setter functions
         */
        void setId(size_t num);
        void addLeftVertice(vertice left);
        void addRightVertice(vertice right);
        void addCenterVertice(vertice center);
        void addPredecessor(Lanelet *pre);
        void addSuccessor(Lanelet *suc);
        void setLeftAdjacent(Lanelet *left, std::string dir);
        void setRightAdjacent(Lanelet *right, std::string dir);
        void setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices);
        void setRightBorderVertices(const std::vector<vertice> &rightBorderVertices);
        void setCenterVertices(const std::vector<vertice> &center);
        void setTrafficLight(TrafficLight *light);
        void addTrafficSign(TrafficSign *sign);
        void setOuterPolygon(const polygon_type &outerPolygon);
        void setBoundingBox(const box &boundingBox);

        // Takes rvalue and moves the data
        void moveLeftBorder(std::vector<vertice> &&leftBorderVertices);
        void moveRightBorder(std::vector<vertice> &&rightBorderVertices);
        void moveCenterVertices(std::vector<vertice> &&center);
        void createCenterVertices();

        /*
         * getter functions
         */
        [[nodiscard]] size_t getId() const;
        [[nodiscard]] std::vector<vertice> getLeftBorderVerticesDirect() const;
        [[nodiscard]] std::vector<vertice> getRightBorderVerticesDirect() const;
        [[nodiscard]] std::vector<vertice> getCenterVerticesDirect() const;
        [[nodiscard]] std::vector<Lanelet *> getPredecessors() const;
        [[nodiscard]] std::vector<Lanelet *> getSuccessors() const;
        [[nodiscard]] const std::vector<vertice> &getCenterVertices() const;
        [[nodiscard]] const std::vector<vertice> &getLeftBorderVertices() const;
        [[nodiscard]] const std::vector<vertice> &getRightBorderVertices() const;
        [[nodiscard]] TrafficLight *getTrafficLight() const;
        [[nodiscard]] std::vector<TrafficSign *> getTrafficSigns() const;
        [[nodiscard]] const polygon_type &getOuterPolygon() const;
        [[nodiscard]] const box &getBoundingBox() const;


        struct adjacent {
            std::vector<Lanelet *> adj;
            std::string dir;
        };

        void constructOuterPolygon(); // construct outer shape from borders
        [[nodiscard]] bool applyIntersectionTesting(const polygon_type &intersection) const;
        [[nodiscard]] bool checkIntersection(const polygon_type &intersecting, size_t intersection_flag) const;

    private:
        size_t id;                                          // unique ID of lanelet

        std::vector<vertice> centerVertices;                // vertices of center line of lanelet
        std::vector<vertice> leftBorder;                    // vertices of left border
        std::vector<vertice> rightBorder;                   // vertices of right border

        std::vector<Lanelet *> predecessorLanelets;         // previous lanelets
        std::vector<Lanelet *> successorLanelets;           // longitudinally adjacent lanelets
        adjacent adjacentLeft;                              // left adjacent lanelet with driving tag
        adjacent adjacentRight;                             // right adjacent lanelet with driving tag
        polygon_type outerPolygon;
        box boundingBox{};                                  // Boost bounding box of the lanelet
        TrafficLight *trafficLightPtr{};                      // traffic light assigned to lanelet
        std::vector<TrafficSign *> trafficSigns;            // traffic signs assigned to lanelet
        TrafficLight * trafficLight{};            // traffic signs assigned to lanelet
};

#endif //ENVIRONMENT_MODEL_LANELET_H
