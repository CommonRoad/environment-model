# CommonRoad Environment Model

The CommonRoad C++ Environment Model provides classes and methods to represent the CommonRoad format in C++17.
It contains an interface to Python and predicates for evaluating traffic rules.

## Overview

### Dependencies to other repositories

The following CommonRoad repositories are downloaded automatically during the build:
- [CommonRoad Curvilinear Coordinate System (CLCS)](https://github.com/CommonRoad/commonroad-clcs)
- [commonroad-io](https://github.com/CommonRoad/commonroad-io) (Python dependency)

### Detailed Instructions
We extracted the relevant information in separate readmes depending on the way you want to use the environment-model:

- You want to *install and use* the Python side of the Environment Model: [Python Usage](python.md)
- You want to *develop* the Python side of the Environment Model: [Python Development](python_dev.md)
- You want to integrate the Environment Model into another CMake project: [C++ Usage](cpp.md)
- You want to set up a development environment for the Environment Model itself: [C++ Development](cpp_dev.md)
- You want to generate the docu of the environment-model: [Documentation Generation](doc.md)
- You want to set up git hooks for the repository: [GitHooks](git.md)
