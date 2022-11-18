include(FetchContent)

message(STATUS "DrivabilityChecker - using bundled version")
FetchContent_Declare(
    crdc
    GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
    GIT_TAG "wip-skbuild"
    GIT_SUBMODULES third_party/gpc # triangle not required
)

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)