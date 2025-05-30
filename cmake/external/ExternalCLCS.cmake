include(FetchContent)
include(utils/FetchContentHelper)

find_package(Threads REQUIRED)

# Required for LINK_LIBRARIES_ONLY_TARGETS (gtest links directly to pthread)
add_library(pthread ALIAS Threads::Threads)



FetchContent_Declare_Fallback(
    CommonRoadCLCS

    GIT_REPOSITORY  git@gitlab.lrz.de:cps/commonroad/commonroad-clcs.git
    #GIT_TAG f0905d3aeeb1d62584d67a73604601f5c948f3f2
    GIT_TAG         develop

    SYSTEM
)

FetchContent_MakeAvailable(CommonRoadCLCS)



mark_as_advanced(
    ADD_MODULE_GEOMETRY
   # BUILD_S11N
)
