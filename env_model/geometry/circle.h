//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_CIRCLE_H
#define ENV_MODEL_CIRCLE_H


#include "../auxiliaryDefs/structs.h"
#include "shape.h"

class Circle : public Shape {
public:
    Circle() {
        radius = 0;
        center = vertice{.x = 0, .y = 0};
    }
    explicit Circle(double &rad) {
        radius = rad;
        center = {0.0, 0.0};
    }
    Circle(double &rad, vertice &vert) {
        radius = rad;
        center = vert;
    }
    Circle(const Circle &) = default;            // copy constructor
    Circle &operator=(const Circle &) = default; // copy assignment
    Circle(Circle &&) = default;                 // move constructor
    Circle &operator=(Circle &&) = default;      // move assignment
    ~Circle() = default;                         // virtual destructor

    /*
     * setter functions
     */
    void setRadius( double rad) override;
    void setCenter( double x, double y) override;

    /*
     * getter functions
     */
    [[nodiscard]] double getRadius() const override;
    [[nodiscard]] vertice getCenter() const override;

    void scaleShape(double factor) override;
    std::string getType() override;

private:
    double radius;
    vertice center{};
};


#endif //ENV_MODEL_CIRCLE_H
