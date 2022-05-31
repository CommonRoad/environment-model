#pragma once

#include <string>

#include "csu.h"
#include "dimension.h"

template <typename Dimension, typename CSU = csu::SI, typename Repr = double> struct Quantity {
    Repr value{std::numeric_limits<Repr>::quiet_NaN()};

    explicit Quantity(Repr value) : value{value} {}

    [[deprecated("use explicit conversion method Quantity::value() instead of relying on implicit conversion")]]
    operator Repr() const noexcept {
        return value;
    }

    Repr value() const noexcept { return value; }

    Quantity<Dimension, CSU, Repr> &operator+=(const Quantity<Dimension, CSU, Repr> &other) noexcept {
        this->value += other.value;
        return *this;
    }

    Quantity<Dimension, CSU, Repr> &operator-=(const Quantity<Dimension, CSU, Repr> &other) noexcept {
        this->value -= other.value;
        return *this;
    }
};

template <typename Repr = double> using meter_t = Quantity<dimension::Position, csu::SI, Repr>;

template <typename Repr = double> using second_t = Quantity<dimension::Time, csu::SI, Repr>;

template <typename Repr = double> using meter_per_second_t = Quantity<dimension::Velocity, csu::SI, Repr>;

template <typename Repr = double> using meter_per_second_squared_t = Quantity<dimension::Acceleration, csu::SI, Repr>;

template <typename Dimension, typename CSU = csu::SI, typename Repr = double>
std::string unit_name(const Quantity<Dimension, CSU, Repr> &quantity) {
    static_assert(false, "template specialization required");
}

template <typename Repr> std::string unit_name(const Quantity<dimension::Position, csu::SI, Repr> &quantity) {
    return "m";
}

template <typename Repr> std::string unit_name(const Quantity<dimension::Velocity, csu::SI, Repr> &quantity) {
    return "m/s";
}

template <typename Repr> std::string unit_name(const Quantity<dimension::Time, csu::SI, Repr> &quantity) { return "s"; }

template <typename Repr> std::string unit_name(const Quantity<dimension::Acceleration, csu::SI, Repr> &quantity) {
    return "m/s^2";
}

template <typename Dimension, typename CSU = csu::SI, typename Repr = double>
Quantity<Dimension, CSU, Repr> make_quantity(const Repr &value) {
    return Quantity<Dimension, CSU, Repr>(value);
}

template <typename Dimension, typename CSU, typename Repr> Quantity<Dimension, CSU, Repr> &
operator+(const Quantity<Dimension, CSU, Repr> &lhs, const Quantity<Dimension, CSU, Repr> &rhs) {
    return Quantity<Dimension, CSU, Repr>(lhs.value + rhs.value);
}

template <typename Dimension, typename CSU, typename Repr> Quantity<Dimension, CSU, Repr> &
operator-(const Quantity<Dimension, CSU, Repr> &lhs, const Quantity<Dimension, CSU, Repr> &rhs) {
    return Quantity<Dimension, CSU, Repr>(lhs.value - rhs.value);
}

template <typename Dimension, typename CSU, typename Repr> Quantity<Dimension, CSU, Repr> &
operator*(const Quantity<Dimension, CSU, Repr> &lhs, const Quantity<Dimension, CSU, Repr> &rhs) {
    static_assert(false, "template specialization required");
}

template <typename CSU, typename Repr> Quantity<dimension::Position, CSU, Repr> &
operator*(const Quantity<dimension::Velocity, CSU, Repr> &lhs, const Quantity<dimension::Time, CSU, Repr> &rhs) {
    return Quantity<dimension::Position, CSU, Repr>(lhs.value * rhs.value);
}

template <typename CSU, typename Repr> Quantity<dimension::Position, CSU, Repr> &
operator*(const Quantity<dimension::Time, CSU, Repr> &lhs, const Quantity<dimension::Velocity, CSU, Repr> &rhs) {
    return Quantity<dimension::Position, CSU, Repr>(lhs.value * rhs.value);
}

template <typename Dimension, typename CSU, typename Repr> Quantity<Dimension, CSU, Repr> &
operator/(const Quantity<Dimension, CSU, Repr> &lhs, const Quantity<Dimension, CSU, Repr> &rhs) {
    static_assert(false, "template specialization required");
}

template <typename CSU, typename Repr> Quantity<dimension::Velocity, CSU, Repr> &
operator/(const Quantity<dimension::Position, CSU, Repr> &lhs, const Quantity<dimension::Time, CSU, Repr> &rhs) {
    return Quantity<dimension::Velocity, CSU, Repr>(lhs.value / rhs.value);
}

template <typename CSU, typename Repr> Quantity<dimension::Acceleration, CSU, Repr> &
operator/(const Quantity<dimension::Velocity, CSU, Repr> &lhs, const Quantity<dimension::Time, CSU, Repr> &rhs) {
    return Quantity<dimension::Acceleration, CSU, Repr>(lhs.value / rhs.value);
}
