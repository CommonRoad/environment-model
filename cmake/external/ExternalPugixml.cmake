include(FetchContent)
include(FetchContentHelper)

# GIT_REPOSITORY https://github.com/zeux/pugixml.git
# GIT_TAG        v1.12.1
# GIT_SHALLOW    true

FetchContent_Declare_Fallback(
    pugixml

    URL http://github.com/zeux/pugixml/releases/download/v1.12/pugixml-1.12.tar.gz
    URL_HASH SHA256=fd6922a4448ec2f3eb9db415d10a49660e5d84ce20ce66b8a07e72ffc84270a7

    SYSTEM
    FIND_PACKAGE_ARGS 1.11
    )

# Install rule for pugixml headers
# install(
#         DIRECTORY ${pugixml_SOURCE_DIR}/src/
#         DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#         FILES_MATCHING PATTERN "*.hpp"
# )

FetchContent_MakeAvailable(pugixml)