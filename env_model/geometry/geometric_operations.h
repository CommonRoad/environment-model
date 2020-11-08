//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_GEOMETRIC_OPERATIONS_H
#define ENV_MODEL_GEOMETRIC_OPERATIONS_H

#include "../auxiliaryDefs/structs.h"
#include "circle.h"

/*
 * add the dimensions of the object (length and width)
 * to the polygon vertices q in the object's coordinate frame
 */
std::vector<vertice> addObjectDimensions(std::vector<vertice> &&q, double length, double width);

/*
 * rotateAndTranslateVertices - rotate and translate the vertices from
 * the special relative coordinates to the reference position and orientation
 * (transfer local coordinates to global coordinates)
 */
std::vector<vertice> rotateAndTranslateVertices(std::vector<vertice> &vertices, vertice refPosition,
                                                double refOrientation);

#endif //ENV_MODEL_GEOMETRIC_OPERATIONS_H
