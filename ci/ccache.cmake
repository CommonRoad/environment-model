include(FetchContent)

if(NOT (CMAKE_SYSTEM_NAME MATCHES "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64"))
    message(STATUS "Skipping ccache setup: Unsupported architecture or OS")
    return()
endif()

set(CCACHE_VERSION "4.7.1")

FetchContent_Declare(
    ccache

    URL https://github.com/ccache/ccache/releases/download/v${CCACHE_VERSION}/ccache-${CCACHE_VERSION}-linux-x86_64.tar.xz
    URL_HASH SHA256=a4240c3fefdba3ddf9ec95138b9eedd65c3655cac63026cc3bb8aff11a2ccd81
    )

FetchContent_MakeAvailable(ccache)

set(CMAKE_C_COMPILER_LAUNCHER ${ccache_SOURCE_DIR}/ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ${ccache_SOURCE_DIR}/ccache)

message(STATUS "C launcher: ${CMAKE_C_COMPILER_LAUNCHER}")
message(STATUS "CXX launcher: ${CMAKE_CXX_COMPILER_LAUNCHER}")

set(FETCHCONTENT_BASE_DIR ${PROJECT_SOURCE_DIR}/_deps)
set(ENV{CCACHE_BASEDIR} ${PROJECT_SOURCE_DIR})
