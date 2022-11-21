include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    pugixml

    # NOTE - URL download should be preferred:
    # HTTP downloads are faster than Git clones and therefore reduce configuration time

    # Git version for reference
    # GIT_REPOSITORY https://github.com/zeux/pugixml.git
    # GIT_TAG        v1.13

    URL http://github.com/zeux/pugixml/releases/download/v1.13/pugixml-1.13.tar.gz
    URL_HASH SHA256=40c0b3914ec131485640fa57e55bf1136446026b41db91c1bef678186a12abbe

    # SYSTEM
    FIND_PACKAGE_ARGS 1.11
    )

# Install rule for pugixml headers
# install(
#         DIRECTORY ${pugixml_SOURCE_DIR}/src/
#         DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#         FILES_MATCHING PATTERN "*.hpp"
# )

FetchContent_MakeAvailable(pugixml)