#include <commonroad_cpp/auxiliaryDefs/timer.h>
#include <stdexcept>

using namespace std::chrono;

Timer::Timer() = default;

Timer::~Timer() = default;

std::chrono::high_resolution_clock::time_point Timer::start() { return high_resolution_clock::now(); }

long Timer::stop(std::chrono::high_resolution_clock::time_point startTime) {
    const auto time(high_resolution_clock::now());

    const auto diff = duration_cast<nanoseconds>(time - startTime).count();

    totalTime += diff;

    return diff;
}

void Timer::addTime(long nanoseconds) { totalTime += nanoseconds; }