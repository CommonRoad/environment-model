#include <iostream>

#include <commonroad_cpp/geometry/polygon.h>

void Polygon::printParameters() {
    std::cout << "--- Polygon Shape ---" << std::endl;
    for (const auto &vertex : this->getPolygon().outer())
        std::cout << "x: " << vertex.get<0>() << ", y: " << vertex.get<1>() << std::endl;
}

ShapeType Polygon::getType() { return ShapeType::polygon; }

Polygon::Polygon(const std::vector<vertex> &pol) {
    this->polygon.outer().resize(pol.size());
    size_t idx{0};
    for (const auto &left : pol) {
        this->polygon.outer()[idx] = point_type{left.x, left.y};
        idx++;
    }
}

[[nodiscard]] const std::vector<vertex> &Polygon::getPolygonVertices() const {
    // Cache assumes Polygon is immutable after construction (no public mutators).
    if (!verticesCacheValid_) {
        verticesCache_.clear();
        verticesCache_.reserve(polygon.outer().size());
        for (const auto &point : polygon.outer()) {
            verticesCache_.emplace_back(vertex{point.x(), point.y()});
        }
        verticesCacheValid_ = true;
    }
    return verticesCache_;
}

std::string Polygon::to_string() {
    std::string result = "Polygon: ";
    for (const auto &point : polygon.outer()) {
        result += "{" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + "}; ";
    }
    return result;
}
