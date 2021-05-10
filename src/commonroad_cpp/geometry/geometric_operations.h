//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_GEOMETRIC_OPERATIONS_H
#define ENV_MODEL_GEOMETRIC_OPERATIONS_H

#include "circle.h"
#include "commonroad_cpp/auxiliaryDefs/structs.h"

/**
 * Add the dimensions of the object (length and width) to the polygon vertices q in the object's coordinate frame.
 *
 * @param q Vertices for which the dimension should be adapted.
 * @param length Length to add.
 * @param width Width to add.
 * @return Vertices of polygon.
 */
std::vector<vertex> addObjectDimensions(std::vector<vertex> q, double length, double width);

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

#endif // ENV_MODEL_GEOMETRIC_OPERATIONS_H
