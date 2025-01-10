#include <commonroad_cpp/obstacle/occupancy.h>

Occupancy::Occupancy(size_t timeStep, std::shared_ptr<Shape> shape) : shape(shape), timeStep(timeStep) {}

std::shared_ptr<Shape> Occupancy::getShape() const { return shape; }

void Occupancy::setShape(std::shared_ptr<Shape> shape) { this->shape = shape; }

size_t Occupancy::getTimeStep() const { return timeStep; }

void Occupancy::setTimeStep(size_t time) { timeStep = time; }
