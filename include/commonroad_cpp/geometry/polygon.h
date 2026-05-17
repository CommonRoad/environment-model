#pragma once

#include "shape.h"
#include <commonroad_cpp/geometry/geometric_operations.h>

#include <utility>

/**
 * Class representing a polygon.
 */
class Polygon final : public Shape {
  public:
    /**
     * Constructor for polygon.
     *
     * @param polygon Polygon representation.
     */
    explicit Polygon(polygon_type polygon) : polygon(std::move(polygon)) {}

    /**
     * Constructor for polygon.
     *
     * @param pol Polygon representation as list of vertices.
     */
    explicit Polygon(const std::vector<vertex> &pol);

    /**
     * Getter for polygon.
     *
     * @return Polygon.
     */
    polygon_type getPolygon() { return polygon; }

    /**
     * Getter for polygon vertices.
     *
     * Vertices are computed lazily from the underlying boost polygon and then
     * cached. The cache assumes the polygon is immutable after construction
     * (no setters / mutators are exposed on Polygon).
     *
     * @return Polygon vertices.
     */
    [[nodiscard]] const std::vector<vertex> &getPolygonVertices() const;

    /**
     * Getter for type.
     *
     * @return Polygon shape type.
     */
    ShapeType getType() override;

    /**
     * Print function. Prints vertices on console output.
     */
    void printParameters() override;

    /**
     * Creates a string representation of a polygon.
     *
     * @return String representation of polygon.
     */
    std::string to_string() override;

  private:
    polygon_type polygon; // boost polygon representation
    mutable std::vector<vertex> verticesCache_;
    mutable bool verticesCacheValid_ = false;
};
