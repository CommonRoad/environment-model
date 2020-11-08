#include "geometric_operations.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <cmath>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
using boost::geometry::get;


std::vector<vertice> addObjectDimensions(std::vector<vertice> &&q, double length, double width) {
    std::vector<vertice> p;

    // check for special cases
    if (q.size() == 1) // exactly one vertice
    {
        p.resize(4);
        // add the dimension around the point q
        vertice p1{}, p2{}, p3{}, p4{};
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
        vertice p1{}, p2{}, p3{}, p4{};
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
        vertice p1{}, p2{}, p3{}, p4{}, p5{}, p6{};
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
    } else if (!q.empty()) // arbitrary polygon
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
    double cosine = cos(refOrientation);
    double sinus = sin(refOrientation);
    std::vector<vertice> transVertices(vertices.size());
    // rotation
    for (size_t i = 0; i < vertices.size(); i++) {
        transVertices[i] =
            vertice{cosine * vertices[i].x - sinus * vertices[i].y, sinus * vertices[i].x + cosine * vertices[i].y};
    }

    // translation
    for (auto & transVertice : transVertices) {
        transVertice.x = transVertice.x + refPosition.x;
        transVertice.y = transVertice.y + refPosition.y;
    }

    return transVertices;
}
