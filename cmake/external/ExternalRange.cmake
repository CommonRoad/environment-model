include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    range-v3
    GIT_REPOSITORY https://github.com/ericniebler/range-v3
    # GIT_TAG        0.12.0
    GIT_TAG        a81477931a8aa2ad025c6bda0609f38e09e4d7ec
    GIT_SHALLOW    true
    GIT_SUBMODULES ""

    SYSTEM
    FIND_PACKAGE_ARGS
    )

FetchContent_MakeAvailable(range-v3)