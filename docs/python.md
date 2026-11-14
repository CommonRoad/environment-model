## Python Interface

### Installing the Python Package
The package is not yet available on PyPI.
Install it directly from the GitHub repository instead:
```
pip install git+https://github.com/CommonRoad/environment-model.git
```

#### Wheel Support Status

The following matrix shows the supported status for each combination of platform and Python version.
In any case, don't worry about selecting the right wheel type as pip will automatically figure that out.
In case binary wheels are not (yet) available for your platform, pip will build the package from source which
might take a while.


| Python version | manylinux-x86_64 | manylinux-arm | windows-x86_64 | macOS-arm | macOS-x86_64 |
|----------------|:----------------:|:-------------:| :-----------: | :-----------: |:-----------:|
| 3.9            |        ✓         | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.10           |        ✓         | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.11           |        ✓         | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.12           |        x         | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.13           |        x         | ✗ | ✗ | ✗ | ✗ | ✗ |

#### Using the Python Module in Anaconda

No special steps are required in order to use the package with Anaconda.

### Working with the environment-model in Python
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
