//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_SHAPE_H
#define ENV_MODEL_SHAPE_H

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include <string>

/**
 * Class representing a shape.
 */
class Shape {
  public:
    /**
     * Virtual getter for shape type.
     *
     * @return Shape type, e.g. circle or rectangle.
     */
    [[nodiscard]] virtual ShapeType getType() = 0;

    /**
     * Virtual function for scaling a shape.
     *
     * @param factor Scaling factor.
     */
    virtual void scaleShape(double factor) = 0;

    /**
     * Virtual print function. Prints general information about shape.
     */
    virtual void printParameters(){};
};

#endif // ENV_MODEL_SHAPE_H
