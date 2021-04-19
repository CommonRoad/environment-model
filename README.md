# Environment Model

CommonRoad C++ Environment Model

**Note: Still in development!!!** 
Not all functionality is finished and well tested.
When you add or change something always check whether a test case already exists. 
If no test case exists, please create one. 
Please assign all merge request to Sebastian Maierhofer.

The CommonRoad C++ Environment Model implemented in C++17 provides classes and methods to represent the CommonRoad format to read CommonRoad scenarios.
It contains also an interface to Python and predicates for evaluating traffic rules.
Note that the repository does not contain runtime-verification algorithms and code for evaluating traffic rules.

## Dependencies:
- [gtest](https://github.com/google/googletest) (should be automatically installed)
- [CommonRoad-IO](https://gitlab.lrz.de/cps/commonroad-io)
- [CommonRoad Curvilinear Coordinate System](https://gitlab.lrz.de/cps/commonroad-curvilinear-coordinate-system)

## Build and Compile

Tested with
- CMake 3.19.2
- GCC 10.2.0

  
## Build
Make a build folder:
```bash
mkdir build
```

Go into build folder:
```bash
cd build
```

Build with `cmake` and specify paths to external dependencies:
```
cmake \
-DCRCCOSY_LIBRARY_DIR=absolutPathToCurvilinearCoordinateSystem/CurvilinearCoordinateSystem
..
```