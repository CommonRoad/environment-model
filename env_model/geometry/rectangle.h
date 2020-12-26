//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_RECTANGLE_H
#define ENV_MODEL_RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape {
public:
    explicit Rectangle(double len = 4.5, double wid = 1.8) : length(len), width(wid) {}

    /*
     * setter functions
     */
    void setLength(double len) override;
    void setWidth(double wid) override;

    /*
     * getter functions
     */
    [[nodiscard]] double getLength() const  override;
    [[nodiscard]] double getWidth() const  override;

    void scaleShape(double factor)  override;
    void printParameters()  override;
    ShapeType getType() override;

private:
    double length{};
    double width{};
};


#endif //ENV_MODEL_RECTANGLE_H
