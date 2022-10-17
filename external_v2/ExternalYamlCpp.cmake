include(FetchContent)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    #GIT_TAG        c73ee34704c512ebe915b283645aefa9f424a22f
    #GIT_TAG        4ae4cb7309c07b2d8623d7e4f01efa6321441366
    GIT_TAG master
    # yaml-cpp-0.7.0
    # GIT_SHALLOW    true
    GIT_PROGRESS   true
)


set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# set(YAML_CPP_BUILD_TOOLS OFF)
set(YAML_BUILD_SHARED_LIBS OFF)
set(YAML_CPP_BUILD_CONTRIB OFF)
set(YAML_CPP_INSTALL OFF)
# set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml-cpp)

