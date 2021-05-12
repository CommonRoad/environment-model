# CommonRoad C++ Environment Model

**Note: Still in development!!!**   
Not all functionalities are finished and well tested.  
When you add or change something always check whether a test case already exists. 
If no test case exists, please create one.   
Similarly, when you see a missing docstring for a class/function please add it.  
Please create always merge requests and assign them to Sebastian Maierhofer.  
For the coding style see [.clang-format](.clang-format). 

The CommonRoad C++ Environment Model provides classes and methods to represent the CommonRoad format in C++17.  
It contains an interface to Python and predicates for evaluating traffic rules (both not finished yet).  
Note that the repository does not contain runtime verification algorithms and code for evaluating traffic rules.


## Dependencies:
- [gtest](https://github.com/google/googletest) (should be automatically installed)
- [commonroad-io](https://gitlab.lrz.de/cps/commonroad-io)
- [CommonRoad Drivability Checker/Curvilinear Coordinate System](https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker) (use *Full Installation with Installation Script* for the installation)
- cmake > 3.16

## Build and Compile

Tested with
- CMake 3.17.5
- GCC 7.5.0

  
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
```bash
cmake -DCRCCOSY_LIBRARY_DIR=absolutPathToCurvilinearCoordinateSystem/DrivabilityChecker ..
```

To use the Python binding, run 
```bash
python setup.py install
```
or
```bash
pip install -e .
```
from the root directory.


## Documentation
The documentation can be generated with
```bash
cd build
make doc_doxygen
```