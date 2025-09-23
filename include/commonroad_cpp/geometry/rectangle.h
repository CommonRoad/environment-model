#pragma once

#include "shape.h"
#include <commonroad_cpp/geometry/geometric_operations.h>

/**
 * Class representing a rectangle.
 */
class Rectangle final : public Shape {
  public:
    /**
     * Constructor for rectangle.
     *
     * @param len Length of rectangle [m].
     * @param wid Width of rectangle [m].
     */
    explicit Rectangle(const double len = 4.5, const double wid = 1.8) : length(len), width(wid) {}

    /**
     * Setter for rectangle length.
     *
     * @param len Length of rectangle [m].
     */
    void setLength(double len) override;

    /**
     * Setter for rectangle width.
     *
     * @param wid Width of rectangle [m].
     */
    void setWidth(double wid) override;

    /**
     * Getter for rectangle length.
     *
     * @return Rectangle length [m].
     */
    [[nodiscard]] double getLength() const override;

    /**
     * Getter for rectangle width.
     *
     * @return Rectangle width [m].
     */
    [[nodiscard]] double getWidth() const override;

    /**
     * Getter for rectangle circumradius (half of diagonal).
     *
     * @return Rectangle circumradius [m].
     */
    [[nodiscard]] double getCircumradius() const override;

    /**
     * Getter for type.
     *
     * @return Rectangle shape type.
     */
    ShapeType getType() override;

    /**
     * Function for scaling a rectangle.
     *
     * @param factor Scaling factor.
     */
    void scaleShape(double factor) override;

    /**
     * Print function. Prints width and length on console output.
     */
    void printParameters() override;

    /**
     * Creates a string representation of a rectangle.
     *
     * @return String representation of rectangle.
     */
    std::string to_string() override;

  private:
    double length{}; //**< Length of shape. */
    double width{};  //**< Width of shape. */
};
