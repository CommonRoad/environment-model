
message(STATUS "DrivabilityChecker - using bundled version")
FetchContent_Declare(
crdc
GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
# GIT_TAG "wip-skbuild"
GIT_TAG fe34dcc46c1d85610aee24c1cc4def877820c5f6
GIT_PROGRESS   true
# GIT_SUBMODULES third_party/gpc # not triangle
GIT_SHALLOW    true
)

FetchContent_MakeAvailable(crdc)
FetchContent_GetProperties(crdc SOURCE_dir)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)