# Code taken from https://vicrucann.github.io/tutorials/quick-cmake-doxygen/

include(FetchContent)

# Current release: 20190607 (update if required)
# Check on https://en.cppreference.com/w/Cppreference:Archives
FetchContent_Declare(
  cppreference_tags
  URL      https://upload.cppreference.com/mwiki/images/1/16/html_book_20190607.tar.xz
  URL_HASH SHA256=8f97b2baa749c748a2e022d785f1a2e95aa851a3075987dfcf38baf65e0e486d
)


FetchContent_MakeAvailable(cppreference_tags)

# check if Doxygen is installed
find_package(Doxygen COMPONENTS dot)

if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Pass path to cppreference tag file
    set(DOXYGEN_CPPREFERENCE_TAG_FILE "${cppreference_tags_SOURCE_DIR}/cppreference-doxygen-web.tag.xml")

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            DEPENDS "${DOXYGEN_CPPREFERENCE_TAG_FILE}" "${DOXYGEN_OUT}"
            VERBATIM )
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)