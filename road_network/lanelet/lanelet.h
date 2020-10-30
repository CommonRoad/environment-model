//
// Created by sebastian on 23.10.20.
//

#ifndef ENVIRONMENT_MODEL_LANELET_H
#define ENVIRONMENT_MODEL_LANELET_H

#include "../../auxiliaryDefs/structs.h"

class Lanelet {
    public:
        Lanelet() { id = 0; }
        Lanelet(const Lanelet &) = default;            // copy constructor
        Lanelet &operator=(const Lanelet &) = default; // copy assignment
        Lanelet(Lanelet &&) = default;                 // move constructor
        Lanelet &operator=(Lanelet &&) = default;      // move assignment
        virtual ~Lanelet() = default;                  // virtual destructor

        /*
         * setter functions
         */
        void addLeftVertice(vertice left);
        void addRightVertice(vertice right);
        void addCenterVertice(vertice center);

        void setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices);
        void setRightBorderVertices(const std::vector<vertice> &rightBorderVertices);
        void setCenterVertices(const std::vector<vertice> &center);

        // Takes rvalue and moves the data
        void moveLeftBorder(std::vector<vertice> &&leftBorderVertices);
        void moveRightBorder(std::vector<vertice> &&rightBorderVertices);
        void moveCenterVertices(std::vector<vertice> &&center);
        void createCenterVertices();

        /*
         * getter functions
         */
        [[nodiscard]] std::vector<vertice> getLeftBorderVerticesDirect() const;
        [[nodiscard]] std::vector<vertice> getRightBorderVerticesDirect() const;
        [[nodiscard]] std::vector<vertice> getCenterVerticesDirect() const;

        [[nodiscard]] const std::vector<vertice> &getCenterVertices() const;
        [[nodiscard]] const std::vector<vertice> &getLeftBorderVertices() const;
        [[nodiscard]] const std::vector<vertice> &getRightBorderVertices() const;

    private:
        size_t id;
        polygon_type outerPolygon; // outer shape of the lanelets as Boost polyon
        box boundingBox;           // Boost bounding box of the lanelet
        std::vector<vertice> centerVertices;
        std::vector<vertice> leftBorder;  // vertices of left border
        std::vector<vertice> rightBorder; // vertices of right border

};


#endif //ENVIRONMENT_MODEL_LANELET_H
