# CommonRoad Environment Model

The CommonRoad C++ Environment Model provides classes and methods to represent the [CommonRoad](https://commonroad.in.tum.de/) format in C++17.
It contains an interface to Python and predicates for evaluating traffic rules.

## Installation

### Python

The Python package `commonroad-cpp` (import name `crcpp`) can be installed directly from this repository:

```bash
pip install git+https://github.com/CommonRoad/environment-model.git
```

A C++17 compiler and CMake ≥ 3.24 are required to build the package from source.
Further dependencies are downloaded automatically during the build.
See [Python Usage](./docs/python.md) for details.

### C++

The Environment Model can be integrated into other CMake projects via `FetchContent`.
See [C++ Usage](./docs/cpp.md) for details.

### Dependencies to other CommonRoad repositories

The following repositories are downloaded automatically during the build:
- [CommonRoad Curvilinear Coordinate System (CLCS)](https://github.com/CommonRoad/commonroad-clcs)
- [commonroad-io](https://github.com/CommonRoad/commonroad-io) (Python dependency)

## Documentation

We extracted the relevant information in separate readmes depending on the way you want to use the environment-model:

- You want to *install and use* the Python side of the Environment Model: [Python Usage](./docs/python.md)
- You want to *develop* the Python side of the Environment Model: [Python Development](./docs/python_dev.md)
- You want to integrate the Environment Model into another CMake project: [C++ Usage](./docs/cpp.md)
- You want to set up a development environment for the Environment Model itself: [C++ Development](./docs/cpp_dev.md)
- You want to generate the docu of the environment-model: [Documentation Generation](./docs/doc.md)
- You want to set up git hooks for the repository: [GitHooks](./docs/git.md)
- You want to add or update a predicate: [Predicates](./src/commonroad_cpp/predicates/readme.md)

## Authors

Contribution (in alphabetic order by last name): Kerim Anater, Yannick Ballnath, Florian Finkeldei, Ioan-Albert Florea,
Josefine Gaßner, Xuyuan Han, Moritz Klischat, Hanna Krasowski, Isabel Kühner, Florian Lercher, Sebastian Maierhofer,
Tobias Markus, Valentin Merle, Martin Rau, Tim Slooten, Roland Stolz, Martin Waxenberger, Bowen Wu

## License

The CommonRoad Environment Model is licensed under the [BSD 3-Clause License](./LICENSE.txt).
It includes and depends on third-party software and data, see [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md).
