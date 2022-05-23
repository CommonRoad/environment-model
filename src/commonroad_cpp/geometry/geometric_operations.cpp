//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/geometry/geometric_operations.h>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <cmath>
#include <commonroad_cpp/geometry/types.h>
#include <geometry/curvilinear_coordinate_system.h>

using CurvilinearCoordinateSystem = geometry::CurvilinearCoordinateSystem;
using boost::geometry::get;

std::vector<vertex> geometric_operations::addObjectDimensions(std::vector<vertex> qVertex, double length,
                                                              double width) {
    std::vector<vertex> pVertex;

    // check for special cases
    if (qVertex.size() == 1) // exactly one vertex
    {
        pVertex.resize(4);
        // add the dimension around the point qVertex
        vertex p1Vertex{}, p2Vertex{}, p3Vertex{}, p4Vertex{};
        p1Vertex.x = qVertex.front().x + (-0.5 * length);
        p1Vertex.y = qVertex.front().y + (0.5 * width);
        p2Vertex.x = qVertex.front().x + (0.5 * length);
        p2Vertex.y = qVertex.front().y + (0.5 * width);
        p3Vertex.x = qVertex.front().x + (0.5 * length);
        p3Vertex.y = qVertex.front().y + (-0.5 * width);
        p4Vertex.x = qVertex.front().x + (-0.5 * length);
        p4Vertex.y = qVertex.front().y + (-0.5 * width);
        pVertex[0] = p1Vertex;
        pVertex[1] = p2Vertex;
        pVertex[2] = p3Vertex;
        pVertex[3] = p4Vertex;
    } else if (qVertex.size() == 4) // exactly 4 vertices
    {
        pVertex.resize(4);
        vertex p1Vertex{}, p2Vertex{}, p3Vertex{}, p4Vertex{};
        p1Vertex.x = qVertex[0].x + (-0.5 * length);
        p1Vertex.y = qVertex[0].y + (0.5 * width);
        p2Vertex.x = qVertex[1].x + (0.5 * length);
        p2Vertex.y = qVertex[1].y + (0.5 * width);
        p3Vertex.x = qVertex[2].x + (0.5 * length);
        p3Vertex.y = qVertex[2].y + (-0.5 * width);
        p4Vertex.x = qVertex[3].x + (-0.5 * length);
        p4Vertex.y = qVertex[3].y + (-0.5 * width);
        pVertex[0] = p1Vertex;
        pVertex[1] = p2Vertex;
        pVertex[2] = p3Vertex;
        pVertex[3] = p4Vertex;
    } else if (qVertex.size() == 6) // exactly six vertices
    {
        pVertex.resize(6);
        // add the dimensions to all six vertices qVertex (Theorem 1)
        vertex p1Vertex{}, p2Vertex{}, p3Vertex{}, p4Vertex{}, p5Vertex{}, p6Vertex{};
        p1Vertex.x = qVertex[0].x + (-0.5 * length);
        p1Vertex.y = qVertex[0].y + (0.5 * width);
        p2Vertex.x = qVertex[1].x + (-0.5 * length);
        p2Vertex.y = qVertex[1].y + (0.5 * width);
        p3Vertex.x = qVertex[2].x + (0.5 * length);
        p3Vertex.y = qVertex[2].y + (0.5 * width);
        p4Vertex.x = qVertex[3].x + (0.5 * length);
        p4Vertex.y = qVertex[3].y + (-0.5 * width);
        p5Vertex.x = qVertex[4].x + (-0.5 * length);
        p5Vertex.y = qVertex[4].y + (-0.5 * width);
        p6Vertex.x = qVertex[5].x + (-0.5 * length);
        p6Vertex.y = qVertex[5].y + (-0.5 * width);
        pVertex[0] = p1Vertex;
        pVertex[1] = p2Vertex;
        pVertex[2] = p3Vertex;
        pVertex[3] = p4Vertex;
        pVertex[4] = p5Vertex;
        pVertex[5] = p6Vertex;
    } else if (!qVertex.empty()) // arbitrary polygon
    {
        // add the dimensions to all vertices qVertex:
        // (left up, right up, left down, right down)
        std::vector<vertex> p_LU = qVertex;
        std::vector<vertex> p_RU = qVertex;
        std::vector<vertex> p_LD = qVertex;
        std::vector<vertex> p_RD = qVertex;
        std::vector<vertex> p_all(qVertex.size() * 4);
        for (size_t i = 0; i < qVertex.size(); i++) {
            p_LU[i].x -= 0.5 * length;
            p_LU[i].y += 0.5 * width;

            p_RU[i].x += 0.5 * length;
            p_RU[i].y += 0.5 * width;

            p_LD[i].x -= 0.5 * length;
            p_LD[i].y -= 0.5 * width;

            p_RD[i].x += 0.5 * length;
            p_RD[i].y -= 0.5 * width;
        }
        size_t idx = 0;
        for (size_t i = 0; i < qVertex.size(); i++) {
            p_all[idx] = p_LU[i];
            idx++;
        }
        for (size_t i = 0; i < qVertex.size(); i++) {
            p_all[idx] = p_RU[i];
            idx++;
        }
        for (size_t i = 0; i < qVertex.size(); i++) {
            p_all[idx] = p_LD[i];
            idx++;
        }
        for (size_t i = 0; i < qVertex.size(); i++) {
            p_all[idx] = p_RD[i];
            idx++;
        }

        // Construct polygon
        polygon_type polygon;
        polygon.outer().resize(p_all.size() + 1);
        for (size_t j{0}; j < p_all.size(); j++)
            polygon.outer()[j] = point_type{p_all[j].x, p_all[j].y};

        polygon.outer().back() = point_type{p_all[0].x, p_all[0].y}; // close polygon

        polygon_type hull;
        boost::geometry::convex_hull(polygon, hull);

        std::vector<point_type> const &points = hull.outer();
        pVertex.resize(points.size());
        for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i)
            pVertex[i] = vertex{double(get<0>(points[i])), double(get<1>(points[i]))};

    } else {
        throw std::runtime_error("Input vector is not a 2D row of vertices.");
    }
    return pVertex;
}

std::vector<vertex> geometric_operations::rotateAndTranslateVertices(std::vector<vertex> &vertices, vertex refPosition,
                                                                     double refOrientation) {
    double cosine = cos(refOrientation);
    double sinus = sin(refOrientation);
    std::vector<vertex> transVertices(vertices.size());
    // rotation
    for (size_t i = 0; i < vertices.size(); i++) {
        transVertices[i] =
            vertex{cosine * vertices[i].x - sinus * vertices[i].y, sinus * vertices[i].x + cosine * vertices[i].y};
    }

    // translation
    for (auto &transVertex : transVertices) {
        transVertex.x = transVertex.x + refPosition.x;
        transVertex.y = transVertex.y + refPosition.y;
    }

    return transVertices;
}

std::vector<double> geometric_operations::computeOrientationFromPolyline(std::vector<vertex> polyline) {
    if (polyline.size() < 2)
        throw std::logic_error("geometric_operations computeOrientationFromPolyline: "
                               "Cannot create orientation from polyline of length < 2");
    std::vector<double> orientation;
    orientation.reserve(polyline.size());

    for (size_t idx{0}; idx < polyline.size() - 1; ++idx)
        orientation.push_back(atan2(polyline[idx + 1].y - polyline[idx].y, polyline[idx + 1].x - polyline[idx].x));

    for (size_t idx{polyline.size() - 1}; idx < polyline.size(); ++idx)
        orientation.push_back(atan2(polyline[idx].y - polyline[idx - 1].y, polyline[idx].x - polyline[idx - 1].x));

    return orientation;
}

std::vector<double> geometric_operations::computePathLengthFromPolyline(const std::vector<vertex> &polyline) {
    std::vector<double> distance(polyline.size(), 0.0);
    for (size_t idx{1}; idx < polyline.size(); ++idx) {
        double xDistance{polyline[idx].x - polyline[idx - 1].x};
        double yDistance{polyline[idx].y - polyline[idx - 1].y};
        distance[idx] = distance[idx - 1] + std::sqrt((xDistance * xDistance + yDistance * yDistance));
    }
    return distance;
}

double geometric_operations::interpolate(double xDistance, const std::vector<double> &polyline1,
                                         const std::vector<double> &polyline2) {
    // taken from https://bulldozer00.blog/2016/05/10/linear-interpolation-in-c/

    // Ensure that no 2 adjacent x values are equal,
    // lest we try to divide by zero when we interpolate.
    const double EPSILON{1.0E-8};
    for (size_t i{1}; i < polyline1.size(); ++i) {
        double deltaX{std::abs(polyline1[i] - polyline1[i - 1])};
        if (deltaX < EPSILON) {
            std::string err{"Potential Divide By Zero: Points " + std::to_string(i - 1) + " And " + std::to_string(i) +
                            " Are Too Close In Value"};
            throw std::range_error(err);
        }
    }

    // Define a lambda that returns true if the xDistance value
    // of a point pair is < the caller's xDistance value
    auto lessThan = [](const double value, double xDistance) { return value < xDistance; };

    // Find the first table entry whose value is >= caller's xDistance value
    auto index{static_cast<size_t>(
        std::distance(polyline1.begin(), std::lower_bound(polyline1.begin(), polyline1.end(), xDistance, lessThan)))};

    // If the caller's X value is greater than the largest
    // X value in the table, we can't interpolate.
    if (index == polyline1.size() - 1)
        return (polyline2[polyline2.size() - 1]);

    // If the caller's X value is less than the smallest X value in the table,
    // we can't interpolate.
    if (index == 0 and xDistance <= polyline1.front()) {
        return polyline2.front();
    }

    // We can interpolate!
    double upperX{polyline1[index]};
    double upperY{polyline2[index]};
    double lowerX{polyline1[index - 1]};
    double lowerY{polyline2[index - 1]};

    double deltaY{upperY - lowerY};
    double deltaX{upperX - lowerX};

    return lowerY + ((xDistance - lowerX) / deltaX) * deltaY;
}

double geometric_operations::subtractOrientations(double lhs, double rhs) {
    return std::fmod((lhs - rhs) + M_PI * 3, 2 * M_PI) - M_PI;
}

double geometric_operations::euclideanDistance2Dim(const vertex &pointA, const vertex &pointB) {
    double xDifference{pointA.x - pointB.x};
    double yDifference{pointA.y - pointB.y};
    return sqrt(pow(xDifference, 2) + pow(yDifference, 2));
}

std::vector<double> geometric_operations::computeDistanceFromPolylines(const std::vector<vertex> &polylineA,
                                                                       const std::vector<vertex> &polylineB) {
    if (polylineA.size() < 2 or polylineB.size() < 2 or polylineA.size() != polylineB.size())
        throw std::logic_error(
            "geometric_operations computeOrientationFromPolyline: "
            "Cannot create distance from polyline of length < 2 or of polylines with different size");
    std::vector<double> width;
    width.reserve(polylineA.size());

    for (size_t idx{0}; idx < polylineA.size(); ++idx)
        width.push_back(euclideanDistance2Dim(polylineA[idx], polylineB[idx]));

    return width;
}

double geometric_operations::scalarProduct(const vertex &vertA, const vertex &vertB) {
    return vertA.x * vertB.x + vertA.y * vertB.y;
}