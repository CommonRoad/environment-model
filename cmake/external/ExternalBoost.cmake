include(FetchContent)

FetchContent_Declare(boost_src
        URL https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz
        URL_HASH SHA256=4b2136f98bdd1f5857f1c3dea9ac2018effe65286cf251534b6ae20cc45e1847
)
FetchContent_MakeAvailable(boost_src)
FetchContent_GetProperties(boost_src SOURCE_DIR)

set(Boost_NO_SYSTEM_PATHS ON)
# set(Boost_DEBUG ON)
set(BOOST_ROOT ${boost_src_SOURCE_DIR})
set(Boost_NO_BOOST_CMAKE ON)

find_package(Boost 1.80.0 REQUIRED
        # OPTIONAL_COMPONENTS program_options
        )
message(STATUS "Boost - found: ${Boost_FOUND}")
message(STATUS "Boost - Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost - Boost_VERSION: ${Boost_VERSION}")

# FIXME: Remove these, added as a hack for Drivability Checker but not actually required
add_library(Boost::geometry ALIAS Boost::headers)
add_library(Boost::align ALIAS Boost::headers)
add_library(Boost::polygon ALIAS Boost::headers)
add_library(Boost::foreach ALIAS Boost::headers)
