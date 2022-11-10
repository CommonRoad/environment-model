include(FetchContent)

message(STATUS "DrivabilityChecker - using bundled version")
FetchContent_Declare(
crdc
GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
# GIT_TAG "wip-skbuild"
GIT_TAG 032550726bbef63e696b72258ee776b52e564b08
# GIT_SUBMODULES third_party/gpc # not triangle
GIT_SHALLOW    true
)

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)