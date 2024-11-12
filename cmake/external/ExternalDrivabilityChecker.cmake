include(FetchContent)
include(utils/FetchContentHelper)

find_package(Threads REQUIRED)

# Required for LINK_LIBRARIES_ONLY_TARGETS (gtest links directly to pthread)
add_library(pthread ALIAS Threads::Threads)

option(LOCAL_CRDC "Use bundled version of Drivability Checker/crccosy" OFF)
option(EXTERNAL_CRDC_FORCE "Force using the external version of Drivability Checker/crccosy" OFF)

if(NOT EXTERNAL_CRDC_FORCE AND (LOCAL_CRDC OR DEFINED ENV{CIBUILDWHEEL}))
    set(ENV_MODEL_USE_LOCAL_CRDC ON)
    message(STATUS "Using local version of Drivability Checker/crccosy")
else()
    set(ENV_MODEL_USE_LOCAL_CRDC OFF)
endif()

if(ENV_MODEL_USE_LOCAL_CRDC)
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/crdc)

    set_property(DIRECTORY ${PROJECT_SOURCE_DIR}/third_party/crdc PROPERTY EXCLUDE_FROM_ALL ON)
else()
    FetchContent_Declare_Fallback(
        crdc

        GIT_REPOSITORY  git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
        GIT_TAG f0905d3aeeb1d62584d67a73604601f5c948f3f2
        #GIT_TAG        development

        SYSTEM
    )

    FetchContent_MakeAvailable(crdc)

    set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
endif()

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)
