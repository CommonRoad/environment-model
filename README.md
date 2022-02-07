# CommonRoad C++ Environment Model

The CommonRoad C++ Environment Model provides classes and methods to represent the CommonRoad format in C++17.  
It contains an interface to Python and predicates for evaluating traffic rules.  
Note that the repository does not contain runtime verification algorithms and code for evaluating traffic rules.


## Dependencies

### CommonRoad dependencies

- [commonroad-io](https://gitlab.lrz.de/cps/commonroad-io)
- [CommonRoad Drivability Checker/Curvilinear Coordinate System](https://gitlab.lrz.de/cps/commonroad-drivability-checker) 
Note that the drivability-checker is installed automatically. Therefore, you need access to the linked repository. 
Additionally, an ssh key in your Gitlab account is required. 
See [here](https://docs.gitlab.com/ee/ssh/) for instructions to add an ssh key.

### Common dependencies

These dependencies should be available as a system package.

- cmake > 3.16
- Boost
- Eigen3
- spdlog
- OpenMP
- Doxygen (for building the documentation)
- Graphviz (for building the documentation)

#### Debian/Ubuntu

You require at least Ubuntu 20.04.
On Debian/Ubuntu, you'll most likely need to install the following packages:  
`build-essential git pkg-config wget libomp-dev libeigen3-dev libboost-all-dev uuid-dev libspdlog-dev`

For building the documentation, you'll need to install `doxygen` and `graphviz`.

For test coverage, you'll need to install `gcovr`.

## Build and Compile

Tested with
- CMake 3.19/20
- GCC 9.3.0
- Clang 10/12

For development the IDE [CLion](https://www.jetbrains.com/clion) is recommended.
You can also take a look at the Docker container, or the .gitlab-ci.yml file to see how the software can be installed. Both are located in the *ci* directory.
  
### C++

1. Make a build folder and change into it:
```bash
mkdir build-debug && cd build-debug
```
**NB:** You can in theory use `build` as a name
for the build folder, however keep in mind that Python's setuptools will always use the `build` folder for
their build process.
This normally shouldn't cause any issues since the file names
used by setuptools don't clash with any names CMake uses currently, but you should still consider using separate folders just in case.


Generate the build files with `cmake`.
Consider specifying the following options:
 * Specify the path where you installed the CommonRoad Drivability Checker using `-DCMAKE_PREFIX_PATH`.
 * Optionally specify an installation prefix where you want to install the Environment Model
   using `-DCMAKE_INSTALL_PREFIX`.
 * The recommended approach is to use a user-writable folder as the installation prefix.
   It is totally fine to use the same installation prefix for several tools.
 * Replace the build type if necessary

Example invocation:
```bash
cmake \
  -DCMAKE_INSTALL_PREFIX=/path/to/install/prefix \
  -DCMAKE_BUILD_TYPE=Debug \
  ..
```

Afterward build with make,
```bash
cmake --build . --parallel 4
```
where you can replace `4` in case more/fewer threads are available for the build.

Install the CommonRoad Drivability Checker library by running
```bash
cmake --install .
```

### Python

To use the environment model within Python, run 
```bash
python setup.py install
```
from the root directory, giving the
path to the Drivability Checker's install prefix
in the `CMAKE_PREFIX_PATH` environment variable.

It is not necessary to build the C++ standalone version first.

### Setting up Git hooks

For development, please install the pre-commit formatting check hook
by running
```bash
  ./setup_git_hooks.sh
```
in the root directory of this repository.

## Usage:

### C++
The main purpose of this code is to serve as library containing the main CommonRoad elements and predicates, e.g., for traffic rules.
The code can be used as C++-standalone version for extracting the predicate cost and satisfaction probability of predicates.
The standalone execution can be configured via the config file and be executed via the **env_model_example_node** which can be executed via
```bash
pathToExecutable/env_model_example_node --input-file pathToRepository/src/commonroad_cpp/default_config.yaml --t 6
```
where *--input-file* specifies the path to a configuration file based on the default configuration file and *--t* specifies the number of threads which should be used.

### Python 
The subsequent code snippet shows important functions needed for using the predicates within Python:
```Python
import commonroad_cpp
...
cpp_env_model.register_scenario(123, 0, lanelet_network, [obstacle_2, obstacle_3], [obstacle_1])

print("Safe Distance satisfied: {}".format(cpp_env_model.in_same_lane_boolean_evaluation(123, 4, 1, 3)))

cpp_env_model.remove_scenario(123)
```
Other predicates can be executed analgously.   
It is necessary to register and remove each scenario before and after using the predicates.  
You can also take a look at the Python test cases for further examples.

For debugging the Python interface you can use the methods described [here](https://www.jetbrains.com/help/clion/debugging-python-extensions.html#debug-custom-py). 
For example, edit Run/Debug configurations for crenvmodel_python as follows:
- target: `cpp_env_model`
- executable: `/your/python3/binary` (path to anaconda environment)
- program arguments: `example.py` (Python file which should be executed)
- working directory: `$ProjectFileDir$`
- environment variables: `PYTHONPATH=$ROOT/cmake-build`

## Documentation
Add the `-DBUILD_DOXYGEN=ON` to the cmake command above.
Afterward, the documentation can be generated with
```bash
cd build-debug
make doc_doxygen
```
