/*
 * parent class for shapes
 */

#ifndef HEADER_SHAPES
#define HEADER_SHAPES

#include "../auxiliaryDefs/structs.h"
#include <string>

class shape {
  public:
    /*
     * setter functions
     */
    virtual void setLength(const double &l){};
    virtual void setWidth(const double &w){};
    virtual void setLength_raw(const double &raw_l){};
    virtual void setWidth_raw(const double &raw_w){};
    virtual void setRadius(const double rad){};
    virtual void setCenter(const double x, const double y){};

    /*
     * getter functions
     */
    virtual double getLength() const { return 0.0; };
    virtual double getWidth() const { return 0.0; };
    virtual double getRawLength() const { return 0.0; };
    virtual double getRawWidth() const { return 0.0; };
    virtual double getRadius() const { return 0.0; };
    virtual vertice getCenter() const {
        struct vertice v = {0, 0};
        return v;
    };

    virtual void scaleShape(double factor) {}
    virtual void printParameters(){};
    virtual std::string getType() = 0;
};

#endif
