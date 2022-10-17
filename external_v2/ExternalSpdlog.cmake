include(FetchContent)

FetchContent_Declare(
    spdlog

    URL https://github.com/gabime/spdlog/archive/refs/tags/v1.10.0.tar.gz
    URL_HASH SHA256=697f91700237dbae2326b90469be32b876b2b44888302afbc7aceb68bcfe8224
    # GIT_REPOSITORY https://github.com/gabime/spdlog.git
    # GIT_TAG        v1.10.0
    # GIT_PROGRESS   true
    # GIT_SHALLOW    true
    )
set(SPDLOG_BUILD_SHARED OFF)

FetchContent_MakeAvailable(spdlog)

# spdlog::spdlog is only present in some configurations, so add it if it is not already present
if(NOT TARGET spdlog::spdlog)
    add_library(spdlog::spdlog ALIAS spdlog)
endif()