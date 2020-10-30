#include "geometricOperations.h"

#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/math/tools/precision.hpp>
#include <cmath>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
using boost::geometry::get;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

std::vector<vertice> addObjectDimensions(std::vector<vertice> &q, double length, double width) {
    std::vector<vertice> p;

    // check for special cases
    if (q.size() == 1) // exactly one vertice
    {
        p.resize(4);
        // add the dimension around the point q
        vertice p1, p2, p3, p4;
        p1.x = q.front().x + (-0.5 * length);
        p1.y = q.front().y + (0.5 * width);
        p2.x = q.front().x + (0.5 * length);
        p2.y = q.front().y + (0.5 * width);
        p3.x = q.front().x + (0.5 * length);
        p3.y = q.front().y + (-0.5 * width);
        p4.x = q.front().x + (-0.5 * length);
        p4.y = q.front().y + (-0.5 * width);
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
    } else if (q.size() == 4) // exactly 4 vertices
    {
        p.resize(4);
        vertice p1, p2, p3, p4;
        p1.x = q[0].x + (-0.5 * length);
        p1.y = q[0].y + (0.5 * width);
        p2.x = q[1].x + (0.5 * length);
        p2.y = q[1].y + (0.5 * width);
        p3.x = q[2].x + (0.5 * length);
        p3.y = q[2].y + (-0.5 * width);
        p4.x = q[3].x + (-0.5 * length);
        p4.y = q[3].y + (-0.5 * width);
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
    } else if (q.size() == 6) // exactly six vertices
    {
        p.resize(6);
        // add the dimensions to all six vertices q (Theorem 1)
        vertice p1, p2, p3, p4, p5, p6;
        p1.x = q[0].x + (-0.5 * length);
        p1.y = q[0].y + (0.5 * width);
        p2.x = q[1].x + (-0.5 * length);
        p2.y = q[1].y + (0.5 * width);
        p3.x = q[2].x + (0.5 * length);
        p3.y = q[2].y + (0.5 * width);
        p4.x = q[3].x + (0.5 * length);
        p4.y = q[3].y + (-0.5 * width);
        p5.x = q[4].x + (-0.5 * length);
        p5.y = q[4].y + (-0.5 * width);
        p6.x = q[5].x + (-0.5 * length);
        p6.y = q[5].y + (-0.5 * width);
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        p[4] = p5;
        p[5] = p6;
    } else if (q.size() != 0) // arbitrary polygon
    {
        // add the dimensions to all vertices q:
        // (left up, right up, left down, right down)
        std::vector<vertice> p_LU = q;
        std::vector<vertice> p_RU = q;
        std::vector<vertice> p_LD = q;
        std::vector<vertice> p_RD = q;
        std::vector<vertice> p_all(q.size() * 4);
        for (size_t i = 0; i < q.size(); i++) {
            p_LU[i].x -= 0.5 * length;
            p_LU[i].y += 0.5 * width;

            p_RU[i].x += 0.5 * length;
            p_RU[i].y += 0.5 * width;

            p_LD[i].x -= 0.5 * length;
            p_LD[i].y -= 0.5 * width;

            p_RD[i].x += 0.5 * length;
            p_RD[i].y -= 0.5 * width;
        }
        int idx = 0;
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_LU[i];
            idx++;
        }
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_RU[i];
            idx++;
        }
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_LD[i];
            idx++;
        }
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_RD[i];
            idx++;
        }

        // Construct polygon
        polygon_type polygon;
        polygon.outer().resize(p_all.size() + 1);
        for (size_t m = 0; m < p_all.size(); m++) {
            polygon.outer()[m] = point_type{p_all[m].x, p_all[m].y};
        }
        polygon.outer().back() = point_type{p_all[0].x, p_all[0].y}; // close polygon

        polygon_type hull;
        boost::geometry::convex_hull(polygon, hull);

        std::vector<point_type> const &points = hull.outer();
        p.resize(points.size());
        for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i) {
            p[i] = vertice{double(get<0>(points[i])), double(get<1>(points[i]))};
        }
    } else {
        throw std::runtime_error("Input vector is not a 2D row of vertices.");
    }
    return p;
}

std::vector<vertice> addObjectDimensions(std::vector<vertice> &&q, double length, double width) {
    std::vector<vertice> p;

    // check for special cases
    if (q.size() == 1) // exactly one vertice
    {
        p.resize(4);
        // add the dimension around the point q
        vertice p1, p2, p3, p4;
        p1.x = q.front().x + (-0.5 * length);
        p1.y = q.front().y + (0.5 * width);
        p2.x = q.front().x + (0.5 * length);
        p2.y = q.front().y + (0.5 * width);
        p3.x = q.front().x + (0.5 * length);
        p3.y = q.front().y + (-0.5 * width);
        p4.x = q.front().x + (-0.5 * length);
        p4.y = q.front().y + (-0.5 * width);
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
    } else if (q.size() == 4) // exactly 4 vertices
    {
        p.resize(4);
        vertice p1, p2, p3, p4;
        p1.x = q[0].x + (-0.5 * length);
        p1.y = q[0].y + (0.5 * width);
        p2.x = q[1].x + (0.5 * length);
        p2.y = q[1].y + (0.5 * width);
        p3.x = q[2].x + (0.5 * length);
        p3.y = q[2].y + (-0.5 * width);
        p4.x = q[3].x + (-0.5 * length);
        p4.y = q[3].y + (-0.5 * width);
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
    } else if (q.size() == 6) // exactly six vertices
    {
        p.resize(6);
        // add the dimensions to all six vertices q (Theorem 1)
        vertice p1, p2, p3, p4, p5, p6;
        p1.x = q[0].x + (-0.5 * length);
        p1.y = q[0].y + (0.5 * width);
        p2.x = q[1].x + (-0.5 * length);
        p2.y = q[1].y + (0.5 * width);
        p3.x = q[2].x + (0.5 * length);
        p3.y = q[2].y + (0.5 * width);
        p4.x = q[3].x + (0.5 * length);
        p4.y = q[3].y + (-0.5 * width);
        p5.x = q[4].x + (-0.5 * length);
        p5.y = q[4].y + (-0.5 * width);
        p6.x = q[5].x + (-0.5 * length);
        p6.y = q[5].y + (-0.5 * width);
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        p[4] = p5;
        p[5] = p6;
    } else if (q.size() != 0) // arbitrary polygon
    {
        // add the dimensions to all vertices q:
        // (left up, right up, left down, right down)
        std::vector<vertice> p_LU = q;
        std::vector<vertice> p_RU = q;
        std::vector<vertice> p_LD = q;
        std::vector<vertice> p_RD = q;
        std::vector<vertice> p_all(q.size() * 4);
        for (size_t i = 0; i < q.size(); i++) {
            p_LU[i].x -= 0.5 * length;
            p_LU[i].y += 0.5 * width;

            p_RU[i].x += 0.5 * length;
            p_RU[i].y += 0.5 * width;

            p_LD[i].x -= 0.5 * length;
            p_LD[i].y -= 0.5 * width;

            p_RD[i].x += 0.5 * length;
            p_RD[i].y -= 0.5 * width;
        }
        int idx = 0;
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_LU[i];
            idx++;
        }
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_RU[i];
            idx++;
        }
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_LD[i];
            idx++;
        }
        for (size_t i = 0; i < q.size(); i++) {
            p_all[idx] = p_RD[i];
            idx++;
        }

        // Construct polygon
        polygon_type polygon;
        polygon.outer().resize(p_all.size() + 1);
        for (size_t m = 0; m < p_all.size(); m++) {
            polygon.outer()[m] = point_type{p_all[m].x, p_all[m].y};
        }
        polygon.outer().back() = point_type{p_all[0].x, p_all[0].y}; // close polygon

        polygon_type hull;
        boost::geometry::convex_hull(polygon, hull);

        std::vector<point_type> const &points = hull.outer();
        p.resize(points.size());
        for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i) {
            p[i] = vertice{double(get<0>(points[i])), double(get<1>(points[i]))};
        }
    } else {
        throw std::runtime_error("Input vector is not a 2D row of vertices.");
    }
    return p;
}

std::vector<vertice> rotateAndTranslateVertices(std::vector<vertice> &vertices, vertice refPosition,
                                                double refOrientation) {
    double cosinus = cos(refOrientation);
    double sinus = sin(refOrientation);
    std::vector<vertice> transVertices(vertices.size());
    // rotation
    for (size_t i = 0; i < vertices.size(); i++) {
        transVertices[i] =
            vertice{cosinus * vertices[i].x - sinus * vertices[i].y, sinus * vertices[i].x + cosinus * vertices[i].y};
    }

    // translation
    for (size_t i = 0; i < transVertices.size(); i++) {
        transVertices[i].x = transVertices[i].x + refPosition.x;
        transVertices[i].y = transVertices[i].y + refPosition.y;
    }

    return transVertices;
}

std::vector<vertice> rotateAndTranslateVertices(std::vector<vertice> &&vertices, vertice refPosition,
                                                double refOrientation) {
    double cosinus = cos(refOrientation);
    double sinus = sin(refOrientation);
    std::vector<vertice> transVertices(vertices.size());
    // rotation
    for (size_t i = 0; i < vertices.size(); i++) {
        transVertices[i] =
            vertice{cosinus * vertices[i].x - sinus * vertices[i].y, sinus * vertices[i].x + cosinus * vertices[i].y};
    }

    // translation
    for (size_t i = 0; i < transVertices.size(); i++) {
        transVertices[i].x = transVertices[i].x + refPosition.x;
        transVertices[i].y = transVertices[i].y + refPosition.y;
    }

    return transVertices;
}

size_t findClosestVertexOfPosition(const std::vector<vertice> &v, vertice pos) {
    size_t id = 0;
    double distance = boost::math::tools::max_value<double>();
    double temp;
    for (size_t i = 0; i < v.size(); i++) {
        temp = std::pow(std::pow(v[i].x - pos.x, 2) + std::pow(v[i].y - pos.y, 2), 0.5);
        if (temp < distance) {
            distance = temp;
            id = i;
        }
    }
    if (v.size() == 0) {
        throw std::runtime_error("Received vector is empty!");
    }
    return id;
}

size_t findClosestVertexOfPosition(std::vector<vertice> &&v, vertice pos) {
    size_t id = 0;
    double distance = boost::math::tools::max_value<double>();
    double temp;
    for (size_t i = 0; i < v.size(); i++) {
        temp = std::pow(std::pow(v[i].x - pos.x, 2) + std::pow(v[i].y - pos.y, 2), 0.5);
        if (temp < distance) {
            distance = temp;
            id = i;
        }
    }
    if (v.size() == 0) {
        throw std::runtime_error("Received vector is empty!");
    }
    return id;
}

double calcAngleOfVerticesAtPosition(const std::vector<vertice> &v, vertice pos) {
    // find closest vertex
    size_t indexClosestVertex = findClosestVertexOfPosition(v, pos);

    if (indexClosestVertex == v.size() - 1) {
        indexClosestVertex--;
    }

    double angle = std::atan2(v[indexClosestVertex + 1].y - v[indexClosestVertex].y,
                              v[indexClosestVertex + 1].x - v[indexClosestVertex].x);
    return angle;
}

double calcAngleOfVerticesAtPosition(std::vector<vertice> &&v, vertice pos) {
    // find closest vertex
    size_t indexClosestVertex = findClosestVertexOfPosition(v, pos);

    if (indexClosestVertex == v.size() - 1) {
        indexClosestVertex--;
    }

    double angle = std::atan2(v[indexClosestVertex + 1].y - v[indexClosestVertex].y,
                              v[indexClosestVertex + 1].x - v[indexClosestVertex].x);
    return angle;
}

//std::vector<size_t> findInnerVerticeOfPosition(vertice position, const ShortestPath &shPath,
//                                               const std::vector<vertice> &leftBorderVertices,
//                                               const std::vector<vertice> &rightBorderVertices) {
//
//    size_t iBorder;
//    std::vector<size_t> ids;
//    double minDistance = boost::math::tools::max_value<double>();
//    size_t iPath_min = 0;
//    double dist;
//
//    // calculate distance between the inner bound and the object position for
//    // all vertices (to find the global minima)
//    for (size_t iPath = 0; iPath < shPath.side.size(); iPath++) {
//        iBorder = shPath.indexBorder[iPath];
//        if (shPath.side[iPath]) // i.e. left
//        {
//            dist = std::pow(std::pow(leftBorderVertices[iBorder].x - position.x, 2) +
//                                std::pow(leftBorderVertices[iBorder].y - position.y, 2),
//                            0.5);
//        } else // i.e. right
//        {
//            dist = std::pow(std::pow(rightBorderVertices[iBorder].x - position.x, 2) +
//                                std::pow(rightBorderVertices[iBorder].y - position.y, 2),
//                            0.5);
//        }
//        if (dist < minDistance) {
//            minDistance = dist;
//            iPath_min = iPath;
//        }
//    }
//
//    // the innner vertice of the minimal distance in respect to the border indexes
//    size_t iBorder_min = shPath.indexBorder[iPath_min];
//    ids.push_back(iPath_min);
//    ids.push_back(iBorder_min);
//    return ids;
//}
//
//std::vector<size_t> findInnerVerticeOfPosition(vertice position, ShortestPath &&shPath,
//                                               std::vector<vertice> &&leftBorderVertices,
//                                               std::vector<vertice> &&rightBorderVertices) {
//
//    size_t iBorder;
//    std::vector<size_t> ids;
//    double minDistance = boost::math::tools::max_value<double>();
//    size_t iPath_min = 0;
//    double dist;
//
//    // calculate distance between the inner bound and the object position for
//    // all vertices (to find the global minima)
//    for (size_t iPath = 0; iPath < shPath.side.size(); iPath++) {
//        iBorder = shPath.indexBorder[iPath];
//        if (shPath.side[iPath]) // i.e. left
//        {
//            dist = std::pow(std::pow(leftBorderVertices[iBorder].x - position.x, 2) +
//                                std::pow(leftBorderVertices[iBorder].y - position.y, 2),
//                            0.5);
//        } else // i.e. right
//        {
//            dist = std::pow(std::pow(rightBorderVertices[iBorder].x - position.x, 2) +
//                                std::pow(rightBorderVertices[iBorder].y - position.y, 2),
//                            0.5);
//        }
//        if (dist < minDistance) {
//            minDistance = dist;
//            iPath_min = iPath;
//        }
//    }
//
//    // the innner vertice of the minimal distance in respect to the border indexes
//    size_t iBorder_min = shPath.indexBorder[iPath_min];
//    ids.push_back(iPath_min);
//    ids.push_back(iBorder_min);
//    return ids;
//}
//
//size_t findVerticeOfPosition(vertice position, const std::vector<vertice> &centerVertices) {
//    // calculate distance between the inner bound and the object position for
//    // all vertices (to find the global minima)
//    size_t iPath_min = 0;
//    double minDistance = 9999999;
//    for (size_t iPath = 0; iPath < centerVertices.size(); iPath++) {
//        double dist = sqrt(std::pow(centerVertices[iPath].x - position.x, 2.0) +
//                           std::pow(centerVertices[iPath].y - position.y, 2.0));
//        if (dist < minDistance) {
//            minDistance = dist;
//            iPath_min = iPath;
//        }
//    }
//
//    return iPath_min;
//}
//
//size_t findVerticeOfPosition(vertice position, std::vector<vertice> &&centerVertices) {
//    // calculate distance between the inner bound and the object position for
//    // all vertices (to find the global minima)
//    size_t iPath_min = 0;
//    double minDistance = boost::math::tools::max_value<double>();
//    for (size_t iPath = 0; iPath < centerVertices.size(); iPath++) {
//        double dist = sqrt(std::pow(centerVertices[iPath].x - position.x, 2.0) +
//                           std::pow(centerVertices[iPath].y - position.y, 2.0));
//        if (dist < minDistance) {
//            minDistance = dist;
//            iPath_min = iPath;
//        }
//    }
//
//    return iPath_min;
//}

size_t getNextIndex(const std::vector<vertice> &vertices, size_t currentIndex, std::string direction,
                    const std::vector<double> &distances) {
    size_t nextIndex = 0;

    // define epsilon for distance check
    double epsilon = 10e-6;

    // find next vertex in the specified direction
    if (direction == "backward") {
        if (currentIndex == 0) {
            nextIndex = currentIndex;
        } else {
            size_t k = currentIndex - 1;
            while (distances[k + 1] <= epsilon && k > 0) {
                k = k - 1;
            }
            nextIndex = k;
        }
    } else if (direction == "forward") {
        if (currentIndex == vertices.size() - 1) {
            nextIndex = currentIndex;
        } else {
            size_t k = currentIndex + 1;
            while (distances[k] <= epsilon && k < vertices.size() - 1) {
                k = k + 1;
            }
            nextIndex = k;
        }
    } else {
        // error
        throw std::runtime_error("Wrong Input!");
    }
    return nextIndex;
}

double calcProjectedDistance(size_t i, border &bound, vertice position) {

    // vertice_i of inner bound
    vertice vi = bound.vertices[i];

    /*
     * find the previous and next vertices on the lane bound:
     * (note that these vertices might not be the on the inner bound)
     */
    if (i == 0 && i == bound.vertices.size() + 1) {
        // error
        throw std::runtime_error("Error!");
    }
    size_t iminus1 = getNextIndex(bound.vertices, i, "backward", bound.distances);
    vertice viminus1 = bound.vertices[iminus1];
    vertice viminus2 = bound.vertices[getNextIndex(bound.vertices, iminus1, "backward", bound.distances)];

    size_t iplus1 = getNextIndex(bound.vertices, i, "forward", bound.distances);
    vertice viplus1 = bound.vertices[iplus1];
    vertice viplus2 = bound.vertices[getNextIndex(bound.vertices, iplus1, "forward", bound.distances)];

    /*
     * choose the segment in which the object's position is projected in:
     * calculate the angle between the points (vi, p, vi-1) and (vi, p, vi+1)
     */
    vertice a;
    a.x = vi.x - position.x;
    a.y = vi.y - position.y;
    vertice b;
    b.x = viminus1.x - position.x;
    b.y = viminus1.y - position.y;
    double beta_iminus1 = std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
    b.x = viplus1.x - position.x;
    b.y = viplus1.y - position.y;
    double beta_iplus1 = std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);

    /*
     * set the flag:
     * if position is between vertices v(i-1) and v(i) -> flag = 1
     * if position is between vertices v(i) and v(i+1) -> flag = 0
     */
    double flag_segement = abs(beta_iminus1) > abs(beta_iplus1);

    /*
     * construct the points (p) and tangents (t) of the base and the tip of the
     * segment
     */
    vertice pBase, tBase, pTip, tTip, ti;
    ti.x = viplus1.x - viminus1.x;
    ti.y = viplus1.y - viminus1.y;
    if (flag_segement) // v(i-1) -> v(i)
    {
        pBase = viminus1;
        tBase.x = vi.x - viminus2.x;
        tBase.y = vi.y - viminus2.y;
        pTip = vi;
        tTip = ti;
    } else // ~flag: v(i) -> v(i+1)
    {
        pBase = vi;
        tBase = ti;
        pTip = viplus1;
        tTip.x = viplus2.x - vi.x;
        tTip.y = viplus2.y - vi.y;
    }

    /*
     * transform the coordinate system:
     * translate by -pBase, such that pBase == 0
     * rotate by theta, such that pTip(2) == 0
     * (note that atan() does not work for all cases)
     */
    double theta = std::atan2(-(pTip.y - pBase.y), (pTip.x - pBase.x));
    double l = std::pow(std::pow((pTip.x - pBase.x), 2) + std::pow((pTip.y - pBase.y), 2), 0.5);

    /*
     * transform the tangent vectors into new coordinate system,
     * i.e. rotate with theta
     */
    vertice tBase_Rot, tTip_Rot;
    tBase_Rot.x = tBase.x * cos(theta) - tBase.y * sin(theta);
    tBase_Rot.y = tBase.x * sin(theta) + tBase.y * cos(theta);
    tTip_Rot.x = tTip.x * cos(theta) - tTip.y * sin(theta);
    tTip_Rot.y = tTip.x * sin(theta) + tTip.y * cos(theta);

    // transform the tangents such that t = [1; m], i.e. slopes m = ty / tx
    double mBase = tBase_Rot.y / tBase_Rot.x;
    double mTip = tTip_Rot.y / tTip_Rot.x;

    // transform the position of the object = [x, y] in new coordinate system:
    // translate
    double x_Trans = position.x - pBase.x;
    double y_Trans = position.y - pBase.y;
    // rotate
    double x = x_Trans * cos(theta) - y_Trans * sin(theta);
    double y = x_Trans * sin(theta) + y_Trans * cos(theta);

    // solve equations (1) - (4) for parameter lamda (equation (5))
    double lamda = (x + y * mBase) / (l - y * (mTip - mBase));
    double distance;
    // distance from pLamda to vi (distance is equal in both coordinate systems)
    if (flag_segement) // (vi == pTip)
    {
        // i.e. distance from pLamda to pTip
        distance = (1 - lamda) * l; // previous verified version
    } else                          // (vi == pBase)
    {
        // i.e. distance from pLamda to pBase
        distance = -lamda * l;
    }

    /*
    // projection vector nLamda (connecting pLamda perpendicular with position)
    double pLamda = lamda * pTip + (1-lamda) * pBase; // (equation (2))
    nLamda = position - pLamda;
    */

    return distance;
}

double calcProjectedDistance(size_t i, std::vector<vertice> vertices, std::vector<double> distances, vertice position) {
    vertice vertexIpath = vertices[i];

    double projectDist = 0;
    double dist = 0;
    if (i > 0) {
        vertice vertexPrev = vertices[getNextIndex(vertices, i, "backward", distances)];
        vertice closestVert1 = closestPointOnLine(vertexPrev, vertexIpath, position);
        dist = sqrt(std::pow(closestVert1.x - position.x, 2.0) + std::pow(closestVert1.y - position.y, 2.0));
        projectDist =
            -1 * sqrt(std::pow(vertexIpath.x - closestVert1.x, 2.0) + std::pow(vertexIpath.y - closestVert1.y, 2.0));
    }
    if (i < vertices.size() - 1) {
        vertice vertexNext = vertices[getNextIndex(vertices, i, "forward", distances)];

        vertice closestVert2 = closestPointOnLine(vertexIpath, vertexNext, position);
        double distTemp =
            sqrt(std::pow(vertexIpath.x - closestVert2.x, 2.0) + std::pow(vertexIpath.y - closestVert2.y, 2.0));
        if (i == 0 ||
            dist > sqrt(std::pow(closestVert2.x - position.x, 2.0) + std::pow(closestVert2.y - position.y, 2.0))) {
            projectDist = distTemp;
        }
    }

    return projectDist;
}

std::vector<double> calcPathDistances(const std::vector<vertice> &vertices) {
    std::vector<double> distances(vertices.size());
    distances[0] = 0;

    for (size_t i = 1; i < vertices.size(); i++) {
        // calculate the distance between the current and previous point
        distances[i] =
            sqrt(std::pow(vertices[i].x - vertices[i - 1].x, 2) + std::pow(vertices[i].y - vertices[i - 1].y, 2));
    }
    return distances;
}

std::vector<double> calcCurvature(const std::vector<vertice> &vertices) {
    std::vector<double> xVector, yVector, dx, ddx, dy, ddy;
    std::vector<double> curvature(vertices.size());

    // seperate x- and y-values
    for (size_t i = 0; i < vertices.size(); i++) {
        xVector.push_back(vertices[i].x);
        // std::cout << "temp: " << vertices[i].x << std::endl;
        yVector.push_back(vertices[i].y);
    }

    // calculate the gradients
    dx = gradient(xVector);
    ddx = gradient(dx);
    dy = gradient(yVector);
    ddy = gradient(dy);

    /*
    // check for mot zero values
    for (size_t i = 0; i < dx.size(); i++)
    {
            if (dx[i] == 0)
            {
                    xNotZero = false;
            }
            if (dy[i] == 0)
            {
                    yNotZero = false;
            }
    }
    */

    // calculate the signed curvature

    for (size_t i = 0; i < dx.size(); i++) {
        if (dx[i] != 0 && dy[i] != 0) {
            double temp = (dx[i] * ddy[i] - ddx[i] * dy[i]) / (std::pow(std::pow(dx[i], 2) + std::pow(dy[i], 2), 1.5));
            curvature[i] = temp;
        } else {
            curvature[i] = 0.0;
        }
    }
    return curvature;
}

bool isLeft(vertice linesStart, vertice linesEnd, vertice points) {
    // isLeft if (x1 - x0)*(y2 - y0) - (x2 - x0)*(y1 - y0) > 0
    bool res = (linesEnd.x - linesStart.x) * (points.y - linesStart.y) >
               (points.x - linesStart.x) * (linesEnd.y - linesStart.y);
    return res;
}

double curvatureOfPoints(vertice a, vertice b, vertice c) {
    double temp1 = std::pow((c.x - b.x), 2.0) + std::pow((c.y - b.y), 2.0);
    double temp2 = (std::pow((b.x - a.x), 2.0) + std::pow((b.y - a.y), 2.0)) *
                   (std::pow((c.x - a.x), 2.0) + std::pow((c.y - a.y), 2.0));
    double denominator = std::sqrt(temp2 * temp1);
    double numerator = 2 * std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));

    if (denominator == 0) {
        return 0;
    }

    return numerator / denominator;
}

double calcVectorIntersectionPoint(vertice a, vertice b, vertice c, vertice d) {
    // catch division by zero
    if (b.x < std::pow(10, -20)) {
        b.x += std::pow(10, -10);
    }
    // analytical solution of vector equation system
    double beta = (a.x * b.y - a.y * b.x - c.x * b.y + c.y * b.x) / (d.x * b.y - d.y * b.x);
    return beta;
}

double calcVectorIntersectionPointAlpha(vertice a, vertice b, vertice c, vertice d) {

    // catch division by zero
    if (std::abs(b.x) < std::pow(10, -10)) {
        if (b.x > 0) {
            b.x += std::pow(10, -10);
        } else {
            b.x -= std::pow(10, -10);
        }
    }
    // analytical solution of vector equation system
    double beta = (a.x * b.y - a.y * b.x - c.x * b.y + c.y * b.x) / (d.x * b.y - d.y * b.x);
    double alpha = (c.x + beta * d.x - a.x) / b.x;
    return alpha;
}

// implementation based on https://de.mathworks.com/help/matlab/ref/gradient.html
std::vector<double> gradient(std::vector<double> data) {
    std::vector<double> result(data.size());
    double temp;
    for (size_t i = 0; i < data.size(); i++) {
        if (i == 0) {
            temp = data[1] - data[0];
        } else if (i == data.size() - 1) {
            temp = data[i] - data[i - 1];
        } else {
            temp = 0.5 * (data[i + 1] - data[i - 1]);
        }
        result[i] = temp;
    }
    return result;
}

double wrapToPi(double rad) {
    // wraps it to the range [-pi, pi].
    if (rad > M_PI) {
        return rad - 2 * M_PI;
    } else if (rad < -M_PI) {
        return rad + 2 * M_PI;
    }
    return rad;
}

//void getLocalConstantRegions(std::vector<double> *radius, std::vector<size_t> *minima, std::vector<region> *regions) {
//    double d_i, d_i_1;
//    size_t i;
//    region newRegion;
//    std::vector<double> diff;
//
//    // check begin of curve
//    if ((*radius)[0] < (*radius)[1]) {
//
//        (*minima).push_back(0);
//    }
//
//    // calculate difference along curve
//    for (i = 0; i < (*radius).size() - 1; i++) {
//        diff.push_back((*radius)[i + 1] - (*radius)[i]);
//    }
//
//    i = 0;
//    while (i < diff.size() - 1) {
//        d_i = diff[i];
//        d_i_1 = diff[i + 1];
//
//        // if gradient direction changes => found minimum
//        if (d_i < 0 && d_i_1 > 0 && d_i_1 != 0) {
//            (*minima).push_back(i + 1);
//        }
//
//        // if value == 0 then a region of constant values starts
//        if (d_i == 0) {
//            newRegion.begin = i;
//            while (i < diff.size() && diff[i + 1] == 0.0) {
//                i++;
//            }
//            newRegion.end = i + 1;
//            (*regions).push_back(newRegion);
//        }
//        i++;
//    }
//
//    // check end of curve
//    if ((*radius)[radius->size() - 2] > radius->back()) {
//        (*minima).push_back(radius->size() - 1);
//    }
//}

std::vector<vertice> encloseByPolygon(circle geometry, double num_vertices, double orientation) {
    std::vector<vertice> vertices(num_vertices);

    // comupte circumradius of polygon determined by inradius and number of
    // corners of polygon
    double central_angle = 2.0 * M_PI / num_vertices;
    double circumradius = geometry.getRadius() / (cos(central_angle / 2.0));

    // generate polygon vertices clockwise
    int idx = 0;
    for (int i = num_vertices - 1; i >= 0; i--) {
        vertices[idx] = vertice{circumradius * (cos(orientation + i * central_angle)) + geometry.getCenter().x,
                                circumradius * (sin(orientation + i * central_angle)) + geometry.getCenter().y};
        idx++;
    }

    return vertices;
}

std::vector<vertice> encloseSectionByPolygon(circle geometry, double num_vertices, double start_orientation,
                                             double end_orientation) {

    std::vector<vertice> vertices(num_vertices);

    // compute angle of sector
    double sector_angle = end_orientation - start_orientation;

    // compute circumradius of polygon portion determined by in radius and number of
    // corner polygons
    double central_angle = sector_angle / (num_vertices - 1);
    double circumradius = geometry.getRadius() / cos(central_angle / 2.0);

    // generate polygon vertices clockwise
    int idx = 0;
    for (int i = num_vertices - 1; i >= 0; i--) {
        vertices[idx] = vertice{circumradius * (cos(start_orientation + i * central_angle)) + geometry.getCenter().x,
                                circumradius * (sin(start_orientation + i * central_angle)) + geometry.getCenter().y};
        idx++;
    }

    return vertices;
}

vertice closestPointOnLine(vertice A, vertice B, vertice P) {
    vertice closest_vert;

    vertice AP = vertice{P.x - A.x, P.y - A.y};
    vertice AB = vertice{B.x - A.x, B.y - A.y};
    double magAB2 = AB.x * AB.x + AB.y * AB.y;
    double ABdotAP = AB.x * AP.x + AB.y * AP.y;

    if (std::abs(magAB2) < 0.000001) {
        return A;
    }

    double t = ABdotAP / magAB2;

    if (t < 0) {
        closest_vert = A;
    } else if (t > 1) {
        closest_vert = B;
    } else {
        closest_vert = vertice{A.x + AB.x * t, A.y + AB.y * t};
    }

    return closest_vert;
}

double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle) {
    double difference = secondAngle - firstAngle;
    while (difference < -M_PI)
        difference += 2 * M_PI;
    while (difference > M_PI)
        difference -= 2 * M_PI;
    return difference;
}

int compareAngles(double sourceAngle, double otherAngle) {
    // sourceAngle and otherAngle should be in the range -180 to 180
    double difference = otherAngle - sourceAngle;

    if (difference < -M_PI)
        difference += 2 * M_PI;
    if (difference > M_PI)
        difference -= 2 * M_PI;

    if (difference > 0.0)
        return 1;
    if (difference < 0.0)
        return -1;

    return 0;
}

//ShortestPath findShortestPath(const border &leftBorder, const border &rightBorder) {
//    // start at the first vertice of the lane borders
//    size_t iFirst = 0;
//
//    ShortestPath result;
//    border innerBound, outerBound;
//
//    // find inner lane bound
//    if (leftBorder.curvature[iFirst] > 0) // (Definition 8)
//    {
//        // left bound is the inner lane bound
//        innerBound = leftBorder;
//        innerBound.side = 1; // flag for left
//        outerBound = rightBorder;
//        outerBound.side = 0; // flag for right
//    } else {
//        // right bound is the inner lane bound
//        innerBound = rightBorder;
//        innerBound.side = 0; // flag for right
//        outerBound = leftBorder;
//        outerBound.side = 1; // flag for left
//    }
//
//    // shortest path starts at the first vertice of the inner bound
//    result.xi.push_back(0);
//    result.indexBorder.push_back(iFirst);
//    result.side.push_back(innerBound.side);
//    result.curvature.push_back(innerBound.curvature[iFirst]);
//    // 2) recursively follow all inner bounds
//    result = followBound(iFirst, result, innerBound, outerBound);
//
//    return result;
//}
//
//ShortestPath followBound(size_t iStart, ShortestPath &shortestPath, const border &innerBound,
//                         const border &outerBound) {
//    size_t i = iStart;
//    size_t j;
//    while (i < innerBound.distances.size() - 1) {
//        /*
//         * check if at the next vertice the inner bound should change,
//         * i.e. curvature < 0 --> right OR curvature > 0 --> left
//         */
//
//        if ((((innerBound.side && innerBound.curvature[i + 1] < 0) ||
//              (innerBound.side == 0 && innerBound.curvature[i + 1] > 0)) &&
//             (std::abs(innerBound.curvature[i + 1]) > std::abs(outerBound.curvature[i + 1]))) ||
//            (((outerBound.side && outerBound.curvature[i + 1] > 0) ||
//              (outerBound.side == 0 && outerBound.curvature[i + 1] < 0)) &&
//             (std::abs(outerBound.curvature[i + 1]) > std::abs(innerBound.curvature[i + 1])))) {
//            vertice piminus1 =
//                innerBound.vertices[getNextIndex(innerBound.vertices, i, "backward", innerBound.distances)];
//            vertice piplus1 =
//                innerBound.vertices[getNextIndex(innerBound.vertices, i, "forward", innerBound.distances)];
//
//            //  construct tangent at vertice i of inner bound (h')
//            vertice hPrime;
//            hPrime.x = piplus1.x - piminus1.x;
//            hPrime.y = piplus1.y - piminus1.y;
//
//            // construct normal vector to tangent at vertice i of inner bound (g)
//            vertice g;
//            g.x = -hPrime.y;
//            g.y = hPrime.x;
//
//            // std::cout << "i: " << i << std::endl;
//            // std::cout << innerBound.vertices[i].x << std::endl;
//            // construct hCross = gamma + alpha*g
//            vertice gamma = innerBound.vertices[i];
//            vertice hCrossStart = gamma;
//            vertice hCrossEnd;
//            hCrossEnd.x = gamma.x + 1 * g.x;
//            hCrossEnd.y = gamma.y + 1 * g.y;
//
//            /*
//             * find point mu_j on outer bound, such that mu_j is the first
//             * vertice in front of hCross in driving direction
//             * (mu_j is an approximation of mu on the lane grid)
//             */
//            vertice mu_j;
//            for (size_t k = iStart; k < outerBound.vertices.size(); k++) {
//                j = k;
//                mu_j = outerBound.vertices[j];
//                // due to the specific orientation of hCross, mu_j is always on
//                // the right of hCross to be in front in driving direction
//                if (!(isLeft(hCrossStart, hCrossEnd, mu_j))) {
//                    break;
//                }
//            }
//            /*
//             * calculate the distance from mu to mu_j along outer bound
//             * (by intersecting hCross and hPrimeOuterBound)
//             * (vector equation: mu_j + beta*hPrimeOuterBound)
//             */
//            // std::cout << "j: " << j << std::endl;
//            vertice hPrimeOuterBound;
//            if (j > 0) {
//                vertice mu_jminus1 =
//                    outerBound.vertices[getNextIndex(outerBound.vertices, j, "backward", outerBound.distances)];
//                double norm =
//                    std::pow((std::pow((mu_jminus1.x - mu_j.x), 2) + std::pow((mu_jminus1.y - mu_j.y), 2)), 0.5);
//                hPrimeOuterBound.x = (mu_jminus1.x - mu_j.x) / norm;
//                hPrimeOuterBound.y = (mu_jminus1.y - mu_j.y) / norm;
//            } else {
//                vertice mu_jplus1 =
//                    outerBound.vertices[getNextIndex(outerBound.vertices, j, "forward", outerBound.distances)];
//                double norm =
//                    std::pow((std::pow((mu_jplus1.x - mu_j.x), 2) + std::pow((mu_jplus1.y - mu_j.y), 2)), 0.5);
//                hPrimeOuterBound.x = (mu_jplus1.x - mu_j.x) / norm;
//                hPrimeOuterBound.y = (mu_jplus1.y - mu_j.y) / norm;
//            }
//            double beta = calcVectorIntersectionPoint(gamma, g, mu_j, hPrimeOuterBound);
//
//            // update shortestPath:
//            size_t iStartNew;
//            if (j > i) // the new inner bound (j) is ahead of the old one (i)
//            {
//                // std::cout << "beta: " << beta << std::endl;
//                // add distance beta to path variable xi
//                shortestPath.xi.push_back(shortestPath.xi.back() + beta);
//                // the current index is j
//                iStartNew = j;
//            } else // % j <= i
//            {
//                // the new inner bound (j) is behind of the old one (i), so it
//                // must follow up, as we have already checked for inflection
//                // points until i+1 --> follow up on the outer bound to i+1:
//                // add distance beta (i.e. distance mu to mu_j) and the distances
//                // from mu_j until (i+1) to path variable xi
//                double sum = 0;
//                for (size_t z = j + 1; z <= i + 1; z++) {
//                    sum += outerBound.distances[z];
//                }
//                // std::cout << "beta: " << beta << std::endl;
//                // std::cout << "sum: " << sum << std::endl;
//                shortestPath.xi.push_back(shortestPath.xi.back() + beta + sum);
//                // the current index is i+1
//                iStartNew = i + 1;
//            }
//            // the current index of the border is the new starting index
//            shortestPath.indexBorder.push_back(iStartNew);
//            // after the inflection point, the outer bound is the new inner bound
//            shortestPath.side.push_back(outerBound.side);
//            // set the curvature
//            shortestPath.curvature.push_back(outerBound.curvature[iStartNew]);
//
//            // DEBUG: check if outer bound should really become the new inner bound
//            if (shortestPath.side.back()) // -> left
//            {
//                size_t temp;
//                if (outerBound.curvature[i + 1] > 0) {
//                    temp = 1;
//                } else {
//                    temp = -1;
//                }
//                if (temp < 0 && (std::abs(outerBound.curvature[i + 1]) > std::abs(innerBound.curvature[i + 1]))) {
//                    // warning
//                }
//            } else // ~shortestPath.side(end) -> right
//            {
//                size_t temp;
//                if (outerBound.curvature[i + 1] > 0) {
//                    temp = 1;
//                } else {
//                    temp = -1;
//                }
//                if (temp > 0 && (std::abs(outerBound.curvature[i + 1]) > std::abs(innerBound.curvature[i + 1]))) {
//                    // warning
//                }
//            }
//            /*
//             * 5) follow the new inner bound from vertice iStartNew with
//             * switched inner and outer lane bound(recursively call followBound() until the end of the inner bound is
//             * reached)
//             */
//            shortestPath = followBound(iStartNew, shortestPath, outerBound, innerBound);
//            // do not continue on the former inner bound, as an inflection point
//            // has been reached and the new inner bound has been followed
//            break;
//        } else // between the vertices i and (i+1) is no inflection point
//        {
//            // take one step along the current inner bound but omit vertices
//            // which are identical (which can be the case for inner bounds)
//            j = getNextIndex(innerBound.vertices, i, "forward", innerBound.distances);
//
//            double sum = 0;
//            for (size_t a = i + 1; a <= j; a++) {
//                sum += innerBound.distances[a];
//            }
//            // adding the distance between the ith and jth vertice to path variable xi
//            shortestPath.xi.push_back(shortestPath.xi.back() + sum);
//            // the current index is j
//            shortestPath.indexBorder.push_back(j);
//            // the side is the inner bound
//            shortestPath.side.push_back(innerBound.side);
//            // set the curvature
//            shortestPath.curvature.push_back(innerBound.curvature[j]);
//
//            // DEBUG: check if inner bound is really inner bound
//            if (shortestPath.side.back()) // -> left
//            {
//                if (innerBound.curvature[i + 1] < 0 &&
//                    std::abs(innerBound.curvature[i + 1]) > std::abs(outerBound.curvature[i + 1])) {
//                    // warning
//                }
//            } else // ~shortestPath.side(end) -> right
//            {
//                if (innerBound.curvature[i + 1] > 0 &&
//                    std::abs(innerBound.curvature[i + 1]) > std::abs(outerBound.curvature[i + 1])) {
//                }
//            }
//
//            // continue following the bound at j
//            i = j;
//        }
//    }
//    return shortestPath;
//}
