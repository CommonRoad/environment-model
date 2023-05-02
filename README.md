# CommonRoad C++ Environment Model

The CommonRoad C++ Environment Model provides classes and methods to represent the CommonRoad format in C++17.
It contains an interface to Python and predicates for evaluating traffic rules.
Note that the repository does not contain runtime verification algorithms and code for evaluating traffic rules.

**Table of Contents**
[[_TOC_]]

### Overview

The required steps to build or use the Environment Model differ a lot depending on
how you want to use it:
- You want to simply *install and use* the Python side of the Environment Model?
  Then the recommended option is to use the pre-built binary packages (wheels)
  provided by us.
  Look at the section on [Installing the Binary Wheels](#installing-the-binary-wheels)
- You want to integrate the Environment Model into another CMake project?
  Look at the section on [Integrating the Environment Model](#integrating-the-environment-model-into-another-c-project)
- You want to set up a development environment for the Environment Model itself?
  Look at the section on [Working on the Environment Model](#working-on-the-environment-model-itself).
  Specific instructions are provided for
  [the Python module](#build-instructions-for-the-python-module) and
  [the C++ library](#build-instructions-for-the-c-library).


## Installing the Environment Model

### Installing the Binary Wheels
Wheels are pre-built binary Python packages that save you the time of building
the Environment Model yourself. We automatically build these wheels for releases.

At the time of writing, the wheels are not yet available on PyPI, so you'll have
to instruct pip to download the packages from the Gitlab package repository.
You'll probably need to set up a Gitlab personal acces token first
if you don't already have one.

For each available package listed in the [package registry](https://gitlab.lrz.de/maierhofer/environment-model/-/packages),
Gitlab provides instructions on how to install it.
For example, the following command should automatically select and
install the most recent Environment Model version (replace `<your_personal_token>`
with your personal access token):
```
pip install commonroad-cpp --extra-index-url https://__token__:<your_personal_token>@gitlab.lrz.de/api/v4/projects/63826/packages/pypi/simple
```

#### Wheel Support Status

The following matrix shows the supported status for each combination of
platform and Python version.
If you're not sure which platform  you have, it will most likely be the
most common one, `manylinux-x86_64`.
In any case, don't worry about selecting the right wheel type as pip will automatically
figure that out. In case binary wheels are not (yet) available for your platform,
pip will build the package from source which might take a while.

Wheels are not built for Windows or macOS at the moment.

| Python version | manylinux-x86_64 | manylinux-i686 | musllinux-x86_64 | musllinux-i686
| --- | :-----------: | :-----------: | :-----------: | :-----------: |
| 3.6 or earlier | ✗ | ✗ | ✗ | ✗ |
| 3.7 | ✓ | ~ | ~ | ~ |
| 3.8 | ✓ | ~ | ~ | ~ |
| 3.9 | ✓ | ~ | ~ | ~ |
| 3.10 | ✓ | ~ | ~ | ~ |
| 3.11 | ✓ | ~ | ~ | ~ |

Legend:
| symbol | description |
| :---: | ----- |
| ✓ | built and tested |
| ~  | built but not tested (known issues) |
| ✗ | not supported |

#### Using the Python Module in Anaconda

No special steps are required in order to use the package with Anaconda.

### Integrating the Environment Model into Another C++ Project
Recent CMake provide the `FetchContent` module which vastly simplifies the integration
for common use scenarios.
Simply insert the following snippet somewhere in your `CMakeLists.txt`:
```cmake
include(FetchContent)

FetchContent_Declare(EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:commonroad-traffic-rules/environment-model.git
        # You can specify any reference here, but prefer specifying a concrete commit if possible
        # as that will speed up the build since Git won't need to check whether branch moved in the meantime
        GIT_TAG <reference to commit, branch, tag...>
)
FetchContent_MakeAvailable(EnvironmentModel)
```

Then add the Environment Model as a dependency to the targets which require it:
```cmake
target_link_libraries(<MyLibraryOrExecutable> PUBLIC EnvironmentModel::env_model)
```


## Using the Environment Model

### C++
The main purpose of this code is to serve as library containing the main CommonRoad elements and predicates, e.g., for traffic rules.
The code can be used as C++-standalone version for extracting the predicate cost and satisfaction probability of predicates.
The standalone execution can be configured via the config file and be executed via the **env_model_example_node** which can be executed via
```bash
pathToExecutable/env_model_example_node --input-file pathToRepository/src/commonroad_cpp/default_config.yaml --t 6
```
where *--input-file* specifies the path to a configuration file based on the default configuration file and *--t* specifies the number of threads which should be used.
**Attention**: The environment-model library is not developed for parallelization.
Therefore, we recommend to use copies inside threads.

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


## Working on the Environment Model itself

Before we begin, please note that the build system was recently overhauled
in order to simplify the process of building and using the Environment Model.
In most cases, building should be possible with no or minimal manual intervention.
Please report any issues or problems you experience.

### Using nix
The [Nix ecosystem](https://nixos.org/) simplifies the usually error-prone process
of obtaining and setting up build dependencies, especially in a C++ project.
If you are new to Nix, please consider reading through an introduction to Nix
such as the excellent [Zero to Nix guide](https://zero-to-nix.com/).

#### Examples using Nix

##### clangd for code completion
Clangd is a C/C++ language server supported by many editors such as Visual Studio Code,
Vim/Neovim and others.
The Nix flake contained in this project includes a shell definition for clangd.
Type the following command in order to enter a clangd-enabled environment:
```
nix develop ".#clangd"
```
And then launch your preferred editor. The `shellHook` will automatically setup
a `compile_commands.json` compilation database that allows clangd to discover the
correct flags.

##### Building the package and running tests
You can simply call `nix build .` to let nix build the environment model as a package
and run tests.

##### direnv: Transparently using the Nix environment in your shell
Consider using [direnv](https://determinate.systems/posts/nix-direnv) which
automatically sets up your shell environment so that all dependencies are available
using Nix.

### Common requirements
These requirements apply to both the Python module and the C++ library.

#### CommonRoad Drivability Checker
The Environment Model depends on the Curvilinear Coordinate System (`crccosy`)
which is part of the [CommonRoad Drivability Checker](https://gitlab.lrz.de/cps/commonroad-drivability-checker).
While the Drivability Checker is installed automatically,
we currently use an internal Git repository so you'll need to ensure
you have access to the repository.
Additionally, an SSH key in your Gitlab account is required.
See [here](https://docs.gitlab.com/ee/ssh/) for instructions to add an SSH key.

#### Compiler and Build System
A recent C/C++ compiler is required in any case.
CMake is required in order to build the C++ library.
For the Python module, you don't need to have CMake installed as pip will
automatically provide CMake to the build system.

### Accelerating the Build with System Packages
The build system can automatically download and configure all dependencies without
further intervention [^1].
However, if you install the dependencies on the system with your package manager,
you can speed up the build process since it won't be necessary to build our own
version of them, as long as the version is sufficient for our requirements.
Certain dependencies take a long time to build, so this is generally a good idea.
Try to install system packags at least for the following dependencies if possible:
- Boost
- Protobuf (the library `libprotobuf` and the compiler `protoc`)

These other dependencies do not take a long time to download and build,
but you might still want to install them to save some time:
- Eigen3
- spdlog
- pugixml
- yaml-cpp
- GoogleTest

The following optional tools are required only for certain tasks:
- For building the documentation
  * Doxygen
  * Graphviz
- For code coverage
  * gcovr

[Specific installation instructions](#installing-dependencies-on-common-distributions) are provided for common distributions.

[^1]: For C++, there is one exception - CMake will need to be installed
on the system.

### Build Instructions for the Python Module
In general, the following command should be sufficient to build the Python module:
```
pip install .
```

You might want to add the verbose flag `-v` in order to see some build output.
Without it, the build might appear to be stuck
(`Building wheel for commonroad-cpp (pyproject.toml) ...`).
```
pip install -v .
```

#### Caveat: Deprecated Commands
**Do not use** bare `setup.py` invocations like the following:
```bash
python setup.py install
```
They do not consider modern Python packaging standards and can cause various issues.

#### Accelerating Repeated Builds
It is possible to significantly speed up repeated builds
of the Python module by specifying the `--no-build-isolation` flag:
```
pip install -v --no-build-isolation .
```
This is completely optional. It might require some additional steps outlined below.

Explanation: When build isolation is enabled (default in recent pip versions),
pip will provide the Python interpreter files in a temporary directory.
The name of the temporary changes for every invocation of `pip install`
even if the files are identical.
Therefore CMake will decide to reconfigure the project, requiring recompilation
of all source files.
By disabling build isolation for development builds,
CMake won't need to reconfigure as the paths to the Python installation stay
the same. Therefore CMake can reuse files from the previous build,
making it the build faster.

However, if you want to use `--no-build-isolation` you need to ensure all
Python build requirements (PEP 518 requirements) as specified in
`pyproject.toml` are already installed.
To install them, run the following:
```
pip install setuptools>=61.0 wheel scikit-build~=0.15.0 cmake~=3.24.0 ninja pybind11~=2.10.0 setuptools_scm[toml]>=6.2
```

### Build Instructions for the C++ Library

#### Modern Build using the Ninja Multi-Config Generator (recommended)
The Ninja Multi-Config generator is a recent addition to CMake with some improvements over
the classical Makefile generator:
- The Ninja build system itself is generally faster than Make
- Ninja automatically selects the number of parallel jobs based on the number of cores -
  fiddling with `-j` options is no longer necessary
- You can build any configuration from a single build directory.

In order to use the Ninja Multi-Config generator, you need to have Ninja installed
on your system.

```bash
# Configuration step
cmake -G "Ninja Multi-Config" -S . -B build

# Build default targets, default configuration (debug)
cmake --build build
# Build default targets, release configuration
cmake --build build --config Release
# Build and run tests
cmake --build build --target test

# Alternatively, you can invoke Ninja directly:
cd build

# Build default targets, default configuration (debug)
ninja
# Build default targets, release configuration
ninja -f build-Release.ninja
# Build and run tests
ninja test
```

#### Classical Build using the Makefile Generator
You can also build the Environment Model using the Makefile generator.
```bash
cmake -S . -B build
# Or specify the build type:
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# Build default targets
cmake --build build
# Replace 4 with the number of parallel jobs you want (generally, the CPU core count)
cmake --build build --parallel 4
# Build and run tests
cmake --build build --target test

# Alternatively, you can invoke Make directly:
cd build

# Build default targets
make
# Replace 4 with the number of parallel jobs you want (generally, the CPU core count)
make -j4
# Build and run tests
make test
```

### Setting up Git Hooks

For development, please install the pre-commit formatting check hook
by running
```bash
  ./setup_git_hooks.sh
```
in the root directory of this repository.
We suggest using the git hook only when you plan to commit/push from command line.
The git hook does not work together with the commit/push function of Clion.
In case you want to use Clion or another IDE, we recommend using the built-in functionality of the IDE.

### Building the Documentation
Add the `-DBUILD_DOXYGEN=ON` to the cmake command above.
Afterwards, the documentation can be generated with
```bash
cmake --build build --target doc_doxygen
```

## Installing Dependencies on Common Distributions

### Debian/Ubuntu

We recommend Ubuntu 20.04 or newer.
#### Ubuntu 20.04
You need to install the following packages:
`build-essential git pkg-config wget libomp-dev libeigen3-dev libboost-all-dev uuid-dev libspdlog-dev`
All other required packages should be part of the standard Ubuntu installation.

#### Ubuntu 18.04
- You need to install the following packages:
`build-essential git pkg-config wget libomp-dev libeigen3-dev libboost-all-dev uuid-dev`
- Install gcc-9 and g++-9: because [filesystem header](https://askubuntu.com/questions/1256440/how-to-get-libstdc-with-c17-filesystem-headers-on-ubuntu-18-bionic):
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9 g++-9
```
- Install [spdlog](https://github.com/gabime/spdlog)
```bash
git clone https://github.com/gabime/spdlog.git
cd spdlog && mkdir build && cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && make -j
sudo make install
```
- Install
  Override default gcc and g++ in current shell:
```bash
export CC=/usr/bin/gcc-9
export CXX=/usr/bin/g++-9
```
  Alternatively, specify the compiler paths in the CMake configuration command:
```
...
-DCMAKE_C_COMPILER=/usr/bin/gcc-9
-DCMAKE_CXX_COMPILER=/usr/bin/g++-9
...
```
Keep in mind that you'll need to use a fresh CMake cache
whenever you change the compiler. Either delete your build folder or
add the `--fresh` flag to your CMake configuration command line.

**IMPORTANT:** Make sure that `PATH` includes the folder where gcc-9 and g++-9 are located.

### Arch Linux

```bash
sudo pacman -S base-devel boost boost-libs pugixml spdlog yaml-cpp protobuf eigen gtest
```

Optional dependencies:
```bash
sudo pacman -S doxygen graphviz gcovr
```
