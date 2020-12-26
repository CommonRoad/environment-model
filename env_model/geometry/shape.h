//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_SHAPE_H
#define ENV_MODEL_SHAPE_H

#include "../auxiliaryDefs/structs.h"
#include <string>

class Shape {
public:
    /*
     * setter functions
     */
    virtual void setLength(const double l){};
    virtual void setWidth(const double w){};
    virtual void setRadius(const double rad){};
    virtual void setCenter(const double x, const double y){};

    /*
     * getter functions
     */
    [[nodiscard]] virtual double getLength() const { return 0.0; };
    [[nodiscard]] virtual double getWidth() const { return 0.0; };
    [[nodiscard]] virtual double getRadius() const { return 0.0; };
    [[nodiscard]] virtual vertice getCenter() const {
        struct vertice v = {0, 0};
        return v;
    };

    virtual void scaleShape(double factor) {}
    virtual void printParameters(){};
    virtual std::string getType() = 0;
};


#endif //ENV_MODEL_SHAPE_H
