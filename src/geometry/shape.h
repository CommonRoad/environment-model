//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_SHAPE_H
#define ENV_MODEL_SHAPE_H

#include "../auxiliaryDefs/structs.h"
#include <string>

/**
 * Class representing a shape.
 */
class Shape {
    public:

        virtual void setLength(const double l){};
        virtual void setWidth(const double w){};
        virtual void setRadius(const double rad){};
        virtual void setCenter(const double x, const double y){};

        /**
         * Virtual getter for shape length. Function cannot be used for circles.
         *
         * @return Shape length [m].
        */
        [[nodiscard]] virtual double getLength() const { return 0.0; };

        /**
         * Virtual getter for shape width. Function cannot be used for circles.
         *
         * @return Shape width [m].
        */
        [[nodiscard]] virtual double getWidth() const { return 0.0; };

        /**
         * Virtual getter for shape radius. Function can only be used for circles.
         *
         * @return Shape radius [m].
        */
        [[nodiscard]] virtual double getRadius() const { return 0.0; };

        /**
         * Virtual getter for shape center. Function can only be used for circles.
         *
         * @return Shape center [m].
        */
        [[nodiscard]] virtual vertice getCenter() const {
            struct vertice v = {0, 0};
            return v;
        };

        /**
         * Virtual getter for shape center. Function can only be used for circles.
         *
         * @return Shape center [m].
        */
        virtual void scaleShape(double factor) {}

    /**
* Virtual getter for shape center. Function can only be used for circles.
*
* @return Shape center [m].
*/
        virtual void printParameters() {};

        /**
         * Virtual getter for shape center. Function can only be used for circles.
         *
         * @return Shape center [m].
        */
        virtual ShapeType getType() = 0;
};


#endif //ENV_MODEL_SHAPE_H
