include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    tsl-robin-map

    SYSTEM

    # NOTE - URL download should be preferred:
    # HTTP downloads are faster than Git clones and therefore reduce configuration time

    # Git version for reference
    #GIT_REPOSITORY https://github.com/Tessil/robin-map.git
    #GIT_TAG        v1.0.1
    # or via commit:
    #GIT_TAG        784245b49780f218996573c521c88aaae4960913

    URL https://github.com/Tessil/robin-map/archive/refs/tags/v1.0.1.tar.gz
    URL_HASH SHA256=b2ffdb623727cea852a66bddcb7fa6d938538a82b40e48294bb581fe086ef005

    FIND_PACKAGE_ARGS 1.0.1
)

FetchContent_MakeAvailable(tsl-robin-map)
