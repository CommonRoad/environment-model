#pragma once
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <commonroad_cpp/geometry/types.h>

/**
 * SensorParameters includes information regarding obstacle sensors
 */
class SensorParameters {
    double fieldOfViewRear{100.0};  //**< length of field of view provided by rear sensors */
    double fieldOfViewFront{150.0}; //**< length of field of view provided by front sensors */
    // TODO update default fov values
    std::vector<vertex> fovVertices{{0.0, -400.0}, {282.8427, -282.8427}, {400.0, 0.0},  {282.8427, 282.8427},
                                    {0.0, 400.0},  {-282.8427, 282.8427}, {-400.0, 0.0}, {-282.8427, -282.8427},
                                    {0.0, -400.0}}; //**< field of view region */
    polygon_type fovPolygon;                        //**< field of view as boost polygon */

  public:
    /**
     * Default constructor.
     */
    SensorParameters() = default;

    /**
     * Complete constructor for SensorParameters.
     *
     * @param fieldOfViewRear length of field of view provided by rear sensors
     * @param fieldOfViewFront length of field of view provided by front sensors
     */
    SensorParameters(double fieldOfViewRear, double fieldOfViewFront)
        : fieldOfViewRear{fieldOfViewRear}, fieldOfViewFront{fieldOfViewFront} {}

    /**
     * Getter for rear field of view.
     *
     * @return Rear field of view.
     */
    [[nodiscard]] double getFieldOfViewRear() const noexcept { return fieldOfViewRear; }

    /**
     * Getter for front field of view.
     *
     * @return Front field of view.
     */
    [[nodiscard]] double getFieldOfViewFront() const noexcept { return fieldOfViewFront; }

    /**
     * Getter for field of view vertices.
     * @return List of vertices spanning field of view.
     */
    [[nodiscard]] std::vector<vertex> getFieldOfViewVertices() const noexcept { return fovVertices; }

    /**
     * Getter for field of view polygon object.
     *
     * @return Boost polygon.
     */
    [[nodiscard]] polygon_type getFieldOfViewPolygon() noexcept {
        if (fovPolygon.outer().empty())
            setFov(fovVertices);
        return fovPolygon;
    }

    /**
     * Setter of field of fiew.
     *
     * @param fovVertices Vertices representing field of view.
     */
    void setFov(const std::vector<vertex> &fovVertices) {
        polygon_type polygon;
        polygon.outer().resize(fovVertices.size());
        size_t idx{0};
        for (const auto &left : fovVertices) {
            polygon.outer()[idx] = point_type{left.x, left.y};
            idx++;
        }

        fovPolygon = polygon;
        boost::geometry::simplify(polygon, fovPolygon, 0.01);
        boost::geometry::unique(fovPolygon);
        boost::geometry::correct(fovPolygon);
    }

    /**
     * Default sensor parameters for vehicles:
     * fieldOfViewRear = 100m, fieldOfViewFront = 150m.
     *
     *
     * @return Default dynamic obstacle sensor parameters.
     */
    static SensorParameters dynamicDefaults() { return SensorParameters{100.0, 150.0}; }

    /**
     * Default sensor parameters for static obstacles:
     * fieldOfViewRear = fieldOfViewFront = 0.0 m.
     *
     * @return Default static obstacle sensor parameters.
     */
    static SensorParameters staticDefaults() { return SensorParameters{0.0, 0.0}; }
};
