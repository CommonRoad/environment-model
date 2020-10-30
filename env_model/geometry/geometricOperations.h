/*
 * geometric operations
 */

#ifndef HEADER_GEOMETRICOPERATIONS
#define HEADER_GEOMETRICOPERATIONS

#include "../auxiliaryDefs/structs.h"
#include "circle.h"

/*
 * add the dimensions of the object (length and width)
 * to the polygon vertices q in the object's coordinate frame
 */
std::vector<vertice> addObjectDimensions(std::vector<vertice> &q, double length, double width);
std::vector<vertice> addObjectDimensions(std::vector<vertice> &&q, double length, double width);

/*
 * rotateAndTranslateVertices - rotate and translate the vertices from
 * the special relative coordinates to the reference position and orientation
 * (transfer local coordinates to global coordinates)
 */
std::vector<vertice> rotateAndTranslateVertices(std::vector<vertice> &vertices, vertice refPosition,
                                                double refOrientation);
std::vector<vertice> rotateAndTranslateVertices(std::vector<vertice> &&vertices, vertice refPosition,
                                                double refOrientation);

/*
 * finds the vertex of the given polyline which
 * has the minimal distance to the given position
 */
size_t findClosestVertexOfPosition(const std::vector<vertice> &v, vertice pos);
size_t findClosestVertexOfPosition(std::vector<vertice> &&v, vertice pos);

/*
 * calculates the angle (orientation) of
 * the line segment of the vertices which has the minimal distance to the
 * given position.
 * The angle is measured in the closed interval [-pi, pi] relative to the
 * global x-axis (counter-clockwise is positive).
 */
double calcAngleOfVerticesAtPosition(const std::vector<vertice> &v, vertice pos);
double calcAngleOfVerticesAtPosition(std::vector<vertice> &&v, vertice pos);

/*
 * find the vertice of the inner bound which has the
 * minimal distance to the object's position
 */
//std::vector<size_t> findInnerVerticeOfPosition(vertice position, const ShortestPath &shPath,
//                                               const std::vector<vertice> &leftBorderVertices,
//                                               const std::vector<vertice> &rightBorderVertices);
//std::vector<size_t> findInnerVerticeOfPosition(vertice position, ShortestPath &&shPath,
//                                               std::vector<vertice> &&leftBorderVertices,
//                                               std::vector<vertice> &&rightBorderVertices);
size_t findVerticeOfPosition(vertice position, const std::vector<vertice> &centerVertices);
size_t findVerticeOfPosition(vertice position, std::vector<vertice> &&centerVertices);

/*
 * project the position on the bound such that it is
 * perpendicular to the pseudo target and calculate the distance of the
 * projection point to vertice i
 */
double calcProjectedDistance(size_t i, border &bound, vertice position);
double calcProjectedDistance(size_t i, std::vector<vertice> vertices, std::vector<double> distances, vertice position);

/*
 * returns the next index in the specified direction while
 * omitting vertices with zero distance
 */
size_t getNextIndex(const std::vector<vertice> &vertices, size_t currentIndex, std::string direction,
                    const std::vector<double> &distances);

/*
 * calculate the distance (L2-norm) between all adjacent
 * points (x,y) of the polyline
 */
std::vector<double> calcPathDistances(const std::vector<vertice> &vertices);

// calculate the well-known signed curvature of the polyline
std::vector<double> calcCurvature(const std::vector<vertice> &vertices);

// exact method for finding the curvature through three points
double curvatureOfPoints(vertice a, vertice b, vertice c);

/*
 * returns boolean array whether the points are on the left of the
 * lines (function accepts single/multiple points and single/multiple lines)
 */
bool isLeft(vertice linesStart, vertice linesEnd, vertice points);

/*
 * calculate the parameters alpha and beta
 * such that the two lines intersect: a + alpha*b = c + beta*d
 * (lines are give in vector equation: r = OA + lamda*AB)
 */
double calcVectorIntersectionPoint(vertice a, vertice b, vertice c, vertice d);
double calcVectorIntersectionPointAlpha(vertice a, vertice b, vertice c, vertice d);

/*
 * calculates the central difference for interior data points.
 * The Algorithm used can be is equivalent to the gradient function from Matlab
 */
std::vector<double> gradient(std::vector<double> data);

// Wrap angle in radians to [(minus)pi, pi]
double wrapToPi(double rad);

// determines minmia and regions of constant values along a curve
//void getLocalConstantRegions(std::vector<double> *radius, std::vector<size_t> *minima, std::vector<region> *regions);

// circumscribedPolygon of circle - function to compute circumscribing polygon with num_vertices corners
std::vector<vertice> encloseByPolygon(circle geometry, double num_vertices, double orientation = 0);

/*
 * function to compute circumscribing polygon
 * portion with num_vertices corners, i.e. over-approximation of circle
 * sector specified by the start and end orientation
 */
std::vector<vertice> encloseSectionByPolygon(circle geometry, double num_vertices, double start_orientation,
                                             double end_orientation);

/*
 * A B and P. So we want to move the co-ordinate system so that AB is the x axis, with A at the origin, and "project" P
 * onto it. The x-co-ordinate of the projected P then tells you how long we want to move along the line AB, so finding
 * the point is then trivial.
 */
vertice closestPointOnLine(vertice A, vertice B, vertice P);

// calculate signed orientation difference of two angles
double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle);

/*
 * returns 1 if otherAngle is to the right of sourceAngle,
 *         0 if the angles are identical
 *         -1 if otherAngle is to the left of sourceAngle
 */
int compareAngles(double sourceAngle, double otherAngle);

/*
 * find the shortest possible path through the lane (network)
 * by always following the inner bound (i.e. corresponding path, Definition 8)
 */
//ShortestPath findShortestPath(const border &leftBorder, const border &rightBorder);

/*
 * keep following the inner bound of the lane until the next
 * inflection point and compute the path variable xi of the shortest path
 */
//ShortestPath followBound(size_t iStart, ShortestPath &shortestPath, const border &innerBound, const border &outerBound);

#endif
