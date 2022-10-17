include(FetchContent)

# GIT_TAG yaml-cpp-0.7.0
# GIT_SHALLOW    true

if(CMAKE_VERSION VERSION_LESS "3.24.0")
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG        4ae4cb7309c07b2d8623d7e4f01efa6321441366
        GIT_PROGRESS   true

        FIND_PACKAGE_ARGS 0.6.0
    )
else()
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG        4ae4cb7309c07b2d8623d7e4f01efa6321441366
        GIT_PROGRESS   true

        FIND_PACKAGE_ARGS 0.6.0
    )
endif()


set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# set(YAML_CPP_BUILD_TOOLS OFF)
set(YAML_BUILD_SHARED_LIBS OFF)
set(YAML_CPP_BUILD_CONTRIB OFF)
set(YAML_CPP_INSTALL OFF)
# set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml-cpp)


# yaml-cpp::yaml-cpp is only present in some configurations, so add it if it is not already present
if(NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()