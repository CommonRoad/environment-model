//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_RECTANGLE_H
#define ENV_MODEL_RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape {
public:
    Rectangle() {
        length = 4.5;
        width = 1.8;
        raw_length = 0;
        raw_width = 0;
    }

    /*
     * setter functions
     */
    void setLength(const double &l) override;
    void setWidth(const double &w) override;
    void setLength_raw(const double &raw_l) override;
    void setWidth_raw(const double &raw_w) override;

    /*
     * getter functions
     */
    [[nodiscard]] double getLength() const  override;
    [[nodiscard]] double getWidth() const  override;
    [[nodiscard]] double getRawLength() const  override;
    [[nodiscard]] double getRawWidth() const  override;

    void scaleShape(double factor)  override;
    void printParameters()  override;
    std::string getType() override;

private:
    double length;
    double width;
    double raw_length;
    double raw_width;
};


#endif //ENV_MODEL_RECTANGLE_H
