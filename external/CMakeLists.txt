include(FetchContent)
include(GNUInstallDirs)

FetchContent_Declare(
  external_pugixml

  GIT_REPOSITORY https://github.com/zeux/pugixml.git
  GIT_TAG        v1.11.4
)

FetchContent_MakeAvailable(external_pugixml)

install(
  DIRECTORY ${external_pugixml_SOURCE_DIR}/src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp"
  )