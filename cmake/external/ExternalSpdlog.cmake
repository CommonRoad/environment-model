include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    spdlog

    # NOTE - URL download should be preferred:
    # HTTP downloads are faster than Git clones and therefore reduce configuration time

    # Git version for reference
    # GIT_REPOSITORY https://github.com/gabime/spdlog.git
    # GIT_TAG        v1.10.0

    URL https://github.com/gabime/spdlog/archive/refs/tags/v1.10.0.tar.gz
    URL_HASH SHA256=697f91700237dbae2326b90469be32b876b2b44888302afbc7aceb68bcfe8224

    SYSTEM
    FIND_PACKAGE_ARGS 1.8.0
    )

set(SPDLOG_BUILD_SHARED OFF)

FetchContent_MakeAvailable(spdlog)

# spdlog::spdlog is only present in some configurations, so add it if it is not already present
if(NOT TARGET spdlog::spdlog)
    add_library(spdlog::spdlog ALIAS spdlog)
endif()