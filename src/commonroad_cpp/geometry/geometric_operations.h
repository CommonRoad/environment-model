//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <vector>

#include "circle.h"
#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include <geometry/curvilinear_coordinate_system.h>
#include <vector>

using CurvilinearCoordinateSystem = geometry::CurvilinearCoordinateSystem;

namespace geometric_operations {

/**
 * Add the dimensions of the object (length and width) to the polygon vertices q in the object's coordinate frame.
 *
 * @param qVertex Vertices for which the dimension should be adapted.
 * @param length Length to add.
 * @param width Width to add.
 * @return Vertices of polygon.
 */
std::vector<vertex> addObjectDimensions(std::vector<vertex> qVertex, double length, double width);

/**
 * Rotate and translate the vertices from the special relative coordinates to the reference position and orientation
 * (transfer local coordinates to global coordinates)
 *
 * @param vertices Vertices which should be rotated and translated.
 * @param refPosition Translation factor.
 * @param refOrientation Rotation factor.
 * @return Vertices of shape.
 */
std::vector<vertex> rotateAndTranslateVertices(std::vector<vertex> &vertices, vertex refPosition,
                                               double refOrientation);

/**
 * Computes orientation along polyline.
 *
 * @param polyline Polyline for which orientation should be calculated.
 * @return List of orientations along polyline.
 */
std::vector<double> computeOrientationFromPolyline(std::vector<vertex> polyline);

/**
 * Computes path length along polyline.
 *
 * @param polyline Polyline for which orientation should be calculated.
 * @return Path length along polyline.
 */
std::vector<double> computePathLengthFromPolyline(const std::vector<vertex> &polyline);

/**
 * Computes Euclidean distance in 2D space.
 *
 * @param pointA First vertex.
 * @param pointB Second vertex.
 * @return Distance.
 */
double euclideanDistance2Dim(const vertex &pointA, const vertex &pointB);

/**
 * Computes distance between each vertex of two polylines. The polylines must be of same length.
 *
 * @param polylineA First polyline.
 * @param polylineB Second polyline.
 * @return Width along polylines.
 */
std::vector<double> computeDistanceFromPolylines(const std::vector<vertex> &polylineA,
                                                 const std::vector<vertex> &polylineB);

/**
 * Interpolates value based on two polylines.
 * @param value Position where to evaluate value.
 * @param xValues x-coordinates of data points. Must be in ascending order.
 * @param yValues y-coordinates of data points.
 *
 * @return Interpolated value.
 */
double interpolate(double value, const std::vector<double> &xValues, const std::vector<double> &yValues);

/**
 * Return the signed difference between angles lhs and rhs
 *
 * @param lhs Left-hand side of the subtraction
 * @param rhs Right-hand side of the subtraction
 * @return  ``(lhs - rhs)``, the value will be within ``[-math.pi, math.pi)``.
    Both ``lhs`` and ``rhs`` may either be zero-based (within
    ``[0, 2*math.pi]``), or ``-pi``-based (within ``[-math.pi, math.pi]``).
 */
double subtractOrientations(double lhs, double rhs);

} // namespace geometric_operations
