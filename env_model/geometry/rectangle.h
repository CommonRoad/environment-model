/*
 * geometry class for rectangles
 */

#ifndef HEADER_RECTANGLE
#define HEADER_RECTANGLE

#include "shape.h"

class rectangle : public shape {
  public:
    rectangle() {
        length = 4.5;
        width = 1.8;
        raw_length = 0;
        raw_width = 0;
    }

    rectangle(const double &l, const double &w) {
        length = l;
        width = w;
        raw_length = 0;
        raw_width = 0;
    }

    rectangle(const double &l, const double &w, const double &raw_l, const double &raw_w) : shape() {
        length = l;
        width = w;
        raw_length = raw_l;
        raw_width = raw_w;
    }
    rectangle(const rectangle &) = default;            // copy constructor
    rectangle &operator=(const rectangle &) = default; // copy assignment
    rectangle(rectangle &&) = default;                 // move constructor
    rectangle &operator=(rectangle &&) = default;      // move assignment
    ~rectangle() = default;                            // virtual destructor

    /*
     * setter functions
     */
    void setLength(const double &l);
    void setWidth(const double &w);
    void setLength_raw(const double &raw_l);
    void setWidth_raw(const double &raw_w);

    /*
     * getter functions
     */
    double getLength() const;
    double getWidth() const;
    double getRawLength() const;
    double getRawWidth() const;

    void scaleShape(double factor);
    void printParameters();
    std::string getType() override;

  private:
    double length;
    double width;
    double raw_length;
    double raw_width;
};

#endif
