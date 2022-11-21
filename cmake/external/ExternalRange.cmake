include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    range-v3

    # NOTE - URL download should be preferred:
    # HTTP downloads are faster than Git clones and therefore reduce configuration time

    # Git version for reference
    #GIT_REPOSITORY https://github.com/ericniebler/range-v3
    #GIT_TAG        0.12.0
    # or via commit:
    #GIT_TAG        a81477931a8aa2ad025c6bda0609f38e09e4d7ec
    #GIT_SUBMODULES ""

    URL https://github.com/ericniebler/range-v3/archive/refs/tags/0.12.0.tar.gz
    URL_HASH SHA256=015adb2300a98edfceaf0725beec3337f542af4915cec4d0b89fa0886f4ba9cb

    # SYSTEM
    # NOTE: range-v3 only allows exact version matches
    FIND_PACKAGE_ARGS 0.12
    )

FetchContent_MakeAvailable(range-v3)