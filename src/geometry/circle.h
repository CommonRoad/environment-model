//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_CIRCLE_H
#define ENV_MODEL_CIRCLE_H

#include "../auxiliaryDefs/structs.h"
#include "shape.h"

/**
 * Class representing a circle.
 */
class Circle : public Shape {
public:
    /**
     * Constructor without parameters. Sets center to origin and radius to zero.
    */
    Circle() {
        radius = 0;
        center = vertice{.x = 0, .y = 0};
    }

    /**
     * Constructor assigning only radius. Center is assigned the origin.
     *
     * @param rad Radius of circle [m].
    */
    explicit Circle(double rad) {
        radius = rad;
        center = {0.0, 0.0};
    }

    /**
     * Constructor assigning radius and center.
     *
     * @param rad Radius of circle [m].
     * @param vert Center vertex.
    */
    Circle(double rad, vertice &vert) {
        radius = rad;
        center = vert;
    }

    /*
     * setter functions
     */
    void setRadius( double rad) override;
    void setCenter( double x, double y) override;

    /**
     * Getter for radius of circle.
     *
     * @return Circle radius [m].
    */
    [[nodiscard]] double getRadius() const override;

    /**
     * Getter for type.
     *
     * @return Circle type.
    */
    ShapeType getType() override;

    /**
     * Getter for center circle. Function can only be used for circles.
     *
     * @return Center vertex.
    */
    [[nodiscard]] vertice getCenter() const override;

    /**
     * Function for scaling a circle.
     *
     * @param factor Scaling factor.
    */
    void scaleShape(double factor) override;


private:
    double radius;
    vertice center{};
};

#endif //ENV_MODEL_CIRCLE_H
