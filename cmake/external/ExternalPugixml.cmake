include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    pugixml

    SYSTEM

    # NOTE - URL download should be preferred:
    # HTTP downloads are faster than Git clones and therefore reduce configuration time

    # Git version for reference
    # GIT_REPOSITORY https://github.com/zeux/pugixml.git
    # GIT_TAG        v1.13

    URL http://github.com/zeux/pugixml/releases/download/v1.15/pugixml-1.15.tar.gz
    URL_HASH SHA256=655ade57fa703fb421c2eb9a0113b5064bddb145d415dd1f88c79353d90d511a

    FIND_PACKAGE_ARGS 1.15
)

# Install rule for pugixml headers
# install(
#         DIRECTORY ${pugixml_SOURCE_DIR}/src/
#         DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#         FILES_MATCHING PATTERN "*.hpp"
# )

FetchContent_MakeAvailable(pugixml)
