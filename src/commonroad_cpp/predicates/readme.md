**0. General Information**
---------------------------
The current predicate "architecture/layout" is as follows:
C++ side:
- Each predicate has its own class derived from an interface
- Predicates have a function for boolean, constraint, and robust evaluation.
- The functions derived from the interface have the following parameter list: timeStep, world, obstacleK, obstacleP
- Those functions can be overloaded with functions using value-based parameters (e.g., providing directly acceleration, velocity, etc.).
- The Python interface for each predicate has the following parameter list: scenarioID, timeStep, obstacleK-ID, obstacleP-ID (the scenario must be registered beforehand and should also be removed after usage)

**1. Create New Branch and Merge Request**
------------------------------------------
If you want to add or change a predicate, please first create a branch and corresponding merge request. Add *Draft* or *WIP* into the name of the merge request until the predicate implementation is finished.

**2. Implementation**
---------------------
Some notes for adding/updating the predicate implementation:
- Every C++ predicate must be derived from the *[CommonRoadPredicate](../../../include/commonroad_cpp/predicates/commonroad_predicate.h)* interface class.
- Each predicate should have an own source and header file and should be located in the corresponding directory (braking, velocity, ...)

- Examples for predicates: [KeepsSafeDistancePrec](../../../include/commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h), [InFrontOf](../../../include/commonroad_cpp/predicates/position/in_front_of_predicate.h), [InSameLane](../../../include/commonroad_cpp/predicates/lane/in_same_lane_predicate.h)

- The constraint mode has currently only limited functionality (only real-valued constraints)

- It could be difficult to add a constraint or robust evaluation to each predicate (in general it should be possible, but the overhead would be quite large). If it should be skipped for the moment, add a runtime exception (not adding the function definition could also work, the linker would probably then raise an error).

- Not all predicates have/need a value-based parameter list (additionally to the object list defined in the general predicate interface class). Depending on the predicate, the parameter list could be quite long.

- The source file of a new predicate must be added in [predicates.cmake](../../../src/predicates.cmake)

The following may be added in the future:
- namespaces for predicates for better differentiation between velocity, braking, etc.

**3. Add/Update C++ Test Case**
-------------------------------
- The test cases should be located in a similar hierarchy as the predicate implementation under [tests/commonroad_cpp_tests/predicates](../../../tests/commonroad_cpp_tests/predicates)

- The source file of a new predicate test case must be added in the [CMakeLists.txt](../../../tests/CMakeLists.txt)

- Examples for predicate test cases: [KeepsSafeDistancePrec](../../../tests/commonroad_cpp_tests/predicates/braking/test_keeps_safe_distance_prec_predicate.h), [InFrontOf](../../../tests/commonroad_cpp_tests/predicates/position/test_in_front_of_predicate.h), [InSameLane](../../../tests/commonroad_cpp_tests/predicates/lane/test_in_same_lane_predicate.h)

- the function [create_road_network](../../../tests/commonroad_cpp_tests/predicates/utils_predicate_test.h) creates a road network with two adjacent lanelets (depending on the predicate this function must be adjusted)

**4. Add/Update Python Interface**
----------------------------------
- The Python interfaces are located under [src/bindings/python_interface_predicates.cpp](../../../src/bindings/python_interface_predicates.cpp)

- New interfaces can be implemented similar to the existing ones.

**5. Add/Update Python Test Case**
----------------------------------
- The Python test cases are located under [tests/python](../../../tests/python).

- New test cases can be implemented similar to the existing ones.

**6. Review Changes**
---------------------
Assign a maintainer to review and merge the changes.
