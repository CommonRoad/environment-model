include(FetchContent)

if(NOT (CMAKE_SYSTEM_NAME MATCHES "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64"))
    message(STATUS "Skipping sccache setup: Unsupported architecture or OS")
    return()
endif()

set(SCCACHE_VERSION "0.3.0")

FetchContent_Declare(
    sccache

    URL https://github.com/mozilla/sccache/releases/download/v${SCCACHE_VERSION}/sccache-v${SCCACHE_VERSION}-x86_64-unknown-linux-musl.tar.gz
    URL_HASH SHA256=e6cd8485f93d683a49c83796b9986f090901765aa4feb40d191b03ea770311d8
    )

FetchContent_MakeAvailable(sccache)

set(CMAKE_C_COMPILER_LAUNCHER ${sccache_SOURCE_DIR}/sccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ${sccache_SOURCE_DIR}/sccache)

message(STATUS "C launcher: ${CMAKE_C_COMPILER_LAUNCHER}")
message(STATUS "CXX launcher: ${CMAKE_CXX_COMPILER_LAUNCHER}")