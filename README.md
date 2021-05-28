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
- Clang 10

For development the IDE [Clion](https://www.jetbrains.com/clion/?gclid=EAIaIQobChMI3-KEq9fk8AIVB853Ch2JdgNFEAAYASAAEgIChfD_BwE&gclsrc=aw.ds) is recommended.

  
### C++
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

### Python

To use the environment model within Python, run 
```bash
python setup.py develop --crccosy absolutPathToCurvilinearCoordinateSystem/DrivabilityChecker
```
from the root directory.

## Usage:

### C++
The code should not be used as C++-standalone version but instead be included as library in other C++ or Python tools which require are CommonRoad representation in C++ or the CommonRoad predicates.

### Python 
The subsequent code snippet shows important functions needed for using the predicates within Python:
```Python
import cpp_env_model
...
cpp_env_model.register_scenario(123, 0, lanelet_network, [obstacle_2, obstacle_3], [obstacle_1])

print("Safe Distance satisfied: {}".format(cpp_env_model.in_same_lane_boolean_evaluation(123, 4, 1, 3)))

cpp_env_model.remove_scenario(123)
```
Other predicates can be executed analgously. It is necessary to register and reomove each scenario before and after using the predicates.

For debugging the Python interface you can use the methods described [here](https://www.jetbrains.com/help/clion/debugging-python-extensions.html#debug-custom-py). 
For example, edit Run/Debug configurations for crenvmodel_python as follows:
target: cpp_env_model
executable: /your/python3/binary (path to anaconda environment)
program arguments: example.py (python file which should be executed)
working directory: $ProjectFileDir$
environment variables: PYTHONPATH=$ROOT/cmake-build

## Documentation
The documentation can be generated with
```bash
cd build
make doc_doxygen
```
