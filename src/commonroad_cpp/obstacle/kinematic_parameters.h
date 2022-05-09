#pragma once

#include <cassert>

struct KinematicParameters {
    //** maximum velocity of obstacle in m/s */
    double vMax;
    //** maximum absolute acceleration of obstacle in [m/s^2] */
    double aMax;
    //** maximal longitudinal acceleration of obstacle in [m/s^2] */
    double aMaxLong;
    //** minimal longitudinal acceleration of obstacle in [m/s^2] */
    double aMinLong;
    //** minimal (longitudinal) braking acceleration of obstacle in [m/s^2] */
    double aBraking;

    KinematicParameters(double vMax, double aMax, double aMaxLong, double aMinLong, double aBraking)
        : vMax{vMax}, aMax{aMax}, aMaxLong{aMaxLong}, aMinLong{aMinLong}, aBraking{aBraking} {
        assert(vMax > 0.0);
        assert(aMax > 0.0);
        assert(aMaxLong > 0.0);
        assert(aMinLong < 0.0);
        assert(aBraking < 0.0);
        assert(aMaxLong <= aMax);
        assert(std::abs(aBraking) <= std::abs(aMinLong));
        // assert(std::abs(aBraking) < aMax);
        // assert(std::abs(aMinLong) < aMax);
    }

    KinematicParameters(double vMax, double aMax) : KinematicParameters{vMax, aMax, aMax, -aMax, -aMax} {}

    static KinematicParameters vehicleDefaults() { return KinematicParameters{50.0, 3.0, 3.0, -10.0, -5.0}; }

    static KinematicParameters pedestrianDefaults() { return KinematicParameters{2.0, 0.6}; }
};

// default:
// vMax = 50.0
// aMax = 3.0
// aMaxLong = 3.0
// aMinLong = -10.0 (??)

// note:
// vMax > 0.0
// aMax > 0.0
// aMaxLong > 0.0
// aMinLong < 0.0
// aMaxLong < aMax
// abs(aMinLong) < aMax

// acc/v error
// orientation error
//
// a_min_long: based on
// a) pessimistic physical assumptions  (crash)
// b) mechanical assumptions (max braking)
//
//
// wheelbase
// max steering angle
//
//
// defaults based on type
//
// detect static limitation violation
