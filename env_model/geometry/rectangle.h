//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_RECTANGLE_H
#define ENV_MODEL_RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape {
public:
    explicit Rectangle(double l = 4.5, double w = 1.8) : length(l), width(w) {}

    /*
     * setter functions
     */
    void setLength(const double l) override;
    void setWidth(const double w) override;

    /*
     * getter functions
     */
    [[nodiscard]] double getLength() const  override;
    [[nodiscard]] double getWidth() const  override;

    void scaleShape(double factor)  override;
    void printParameters()  override;
    std::string getType() override;

private:
    double length{};
    double width{};
};


#endif //ENV_MODEL_RECTANGLE_H
