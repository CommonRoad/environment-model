## Python Interface

### Installing the Binary Wheels
Wheels are pre-built binary Python packages that save you the time of building the Environment Model yourself. 
We automatically build these wheels for releases.

At the time of writing, the wheels are not yet available on PyPI, so you'll have to instruct pip to download the 
packages from the Gitlab package repository.
You'll probably need to set up a Gitlab personal access token first if you don't already have one.

For each available package listed in the [package registry](https://gitlab.lrz.de/maierhofer/environment-model/-/packages),
Gitlab provides instructions on how to install it.
For example, the following command should automatically select and install the most recent Environment Model 
version (replace `<your_personal_token>` with your personal access token):
```
pip install commonroad-cpp --extra-index-url https://__token__:<your_personal_token>@gitlab.lrz.de/api/v4/projects/63826/packages/pypi/simple
```

#### Wheel Support Status

The following matrix shows the supported status for each combination of platform and Python version.
In any case, don't worry about selecting the right wheel type as pip will automatically figure that out. 
In case binary wheels are not (yet) available for your platform, pip will build the package from source which 
might take a while.


| Python version | manylinux-x86_64 | manylinux-arm | windows-x86_64 | macOS-arm | macOS-x86_64 |
|----------------| :-----------: |:-------------:| :-----------: | :-----------: |:-----------:|
| 3.7 or earlier | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.8            | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.9            | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.10           | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |
| 3.11           | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |

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
