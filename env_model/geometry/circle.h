/*
 * geometry class for circles
 */

#ifndef HEADER_CIRCLE
#define HEADER_CIRCLE

#include "../auxiliaryDefs/structs.h"
#include "shape.h"

class circle : public shape {
  public:
    circle() {
        radius = 0;
        center = vertice{.x = 0, .y = 0};
    }
    circle(double &rad) {
        radius = rad;
        center = {0.0, 0.0};
    }
    circle(double &rad, vertice &vert) {
        radius = rad;
        center = vert;
    }
    circle(const circle &) = default;            // copy constructor
    circle &operator=(const circle &) = default; // copy assignment
    circle(circle &&) = default;                 // move constructor
    circle &operator=(circle &&) = default;      // move assignment
    ~circle() = default;                         // virtual destructor

    /*
     * setter functions
     */
    void setRadius(const double rad);
    void setCenter(const double x, const double y);

    /*
     * getter functions
     */
    double getRadius() const;
    vertice getCenter() const;

    void scaleShape(double factor);
    std::string getType() override;

  private:
    double radius;
    vertice center;
};

#endif
