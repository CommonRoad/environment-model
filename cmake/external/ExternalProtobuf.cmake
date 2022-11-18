include(FetchContent)

option(COMMONROAD_SYSTEM_PROTOBUF "Use system Protobuf" ON)

# set(Protobuf_DEBUG ON)

if(COMMONROAD_SYSTEM_PROTOBUF)
    find_package(Protobuf 3.6)
endif()

if(COMMONROAD_SYSTEM_PROTOBUF AND Protobuf_FOUND)
    message(STATUS "Protobuf - SYSTEM")

    # NOTE: The following Protobuf check was adapted from FindProtobuf.cmake

    # Check Protobuf compiler version to be aligned with libraries version
    execute_process(COMMAND ${Protobuf_PROTOC_EXECUTABLE} --version
                    OUTPUT_VARIABLE _PROTOBUF_PROTOC_EXECUTABLE_VERSION)

    if("${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}" MATCHES "libprotoc ([0-9.]+)")
        set(_PROTOBUF_PROTOC_EXECUTABLE_VERSION "${CMAKE_MATCH_1}")
    endif()

    if(Protobuf_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
            "${Protobuf_PROTOC_EXECUTABLE} reveals version ${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}")
    endif()

    if(NOT "${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}" VERSION_EQUAL "${Protobuf_VERSION}")
        message(FATAL_ERROR "Unfortunately, the system protobuf installation is unusable "
            "due to a detected mismatch between the Protobuf compiler and the Protobuf libraries. "
            "We can't proceed at this point since CMake already added the corresponding targets, "
            "and there is no way to remove those targets now. "
            "Please rerun the configuration, adding the -DCOMMONROAD_SYSTEM_PROTOBUF=OFF to the "
            "CMake command line.")
    endif()

    return()
endif()

message(STATUS "Protobuf - falling back to external version")

# set(ZLIB_USE_STATIC_LIBS ON)
# find_package(ZLIB REQUIRED)
set(protobuf_WITH_ZLIB OFF CACHE BOOL "" FORCE)

set(ABSL_USE_EXTERNAL_GOOGLETEST ON CACHE BOOL "" FORCE)
set(protobuf_USE_EXTERNAL_GTEST ON CACHE BOOL "" FORCE)

set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# IMPORTANT: Needs to be enabled because of a Protobuf bug:
# cmake/libprotobuf.cmake incorrectly includes a source-local path
# in INTERFACE_INCLUDE_DIRECTORIES.
set(protobuf_INSTALL ON CACHE BOOL "" FORCE)

# set(protobuf_MODULE_COMPATIBLE ON CACHE BOOL "")

#find_package(Protobuf REQUIRED)
set(Protobuf_VERSION "3.21.5")

# Protobuf_VERSION:              3.21.5
# PROTOBUF_TAG:                 v3.21.5
# PROTOBUF_ALT_VERSION:           v21.5
# PROTOBUF_ALT_NUMERIC_VERSION:    21.5
message(VERBOSE "Protobuf ver=${Protobuf_VERSION}")
set(PROTOBUF_TAG "v${Protobuf_VERSION}")
message(VERBOSE "Protobuf tag=${PROTOBUF_TAG}")
if (Protobuf_VERSION VERSION_GREATER_EQUAL "3.21.0")
        string(REGEX MATCH "3.([0-9.]+)" _IGNORED_ ${Protobuf_VERSION})
        set(PROTOBUF_ALT_VERSION "v${CMAKE_MATCH_1}")
        set(PROTOBUF_ALT_NUMERIC_VERSION "${CMAKE_MATCH_1}")
else()
        set(PROTOBUF_ALT_VERSION ${PROTOBUF_TAG})
        set(PROTOBUF_ALT_NUMERIC_VERSION ${Protobuf_VERSION})
endif()

message(VERBOSE "Protobuf alt version=${PROTOBUF_ALT_VERSION}")

set(PROTOBUF_BASE_URL "https://github.com/protocolbuffers/protobuf/releases/download")

set(USE_BINARY_PROTOC TRUE)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(PROTOC_ARCH "linux-x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
        set(PROTOC_ARCH "linux-x86_32")
    else()
        message(WARNING "Unknown system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
        set(USE_BINARY_PROTOC FALSE)
    endif()
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
    # Untested!
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(PROTOC_ARCH "win64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
        set(PROTOC_ARCH "win32")
    else()
        message(WARNING "Unknown system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
        set(USE_BINARY_PROTOC FALSE)
    endif()
else()
    message(WARNING "Unknown system name: ${CMAKE_SYSTEM_NAME}")
    set(USE_BINARY_PROTOC FALSE)
endif()

if(NOT USE_BINARY_PROTOC)
    message(FATAL_ERROR "External protoc download is not yet supported for your OS/architecture")
endif()

if(USE_BINARY_PROTOC)
    set(protobuf_BUILD_LIBPROTOC OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_PROTOBUF_BINARIES OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_PROTOC_BINARIES OFF CACHE BOOL "" FORCE)

    message(STATUS "Trying to use binary protoc")
    message(VERBOSE "Protoc system/architecture: ${PROTOC_ARCH}")
    message(VERBOSE "Protoc download URL: ${PROTOBUF_BASE_URL}/v${PROTOBUF_ALT_NUMERIC_VERSION}/protoc-${PROTOBUF_ALT_NUMERIC_VERSION}-${PROTOC_ARCH}.zip")

    FetchContent_Declare(
    external_protobuf_protoc
        URL  ${PROTOBUF_BASE_URL}/v${PROTOBUF_ALT_NUMERIC_VERSION}/protoc-${PROTOBUF_ALT_NUMERIC_VERSION}-${PROTOC_ARCH}.zip
    )
    FetchContent_MakeAvailable(external_protobuf_protoc)

    find_program(PROTOC
            NAMES protoc
            HINTS ${external_protobuf_protoc_SOURCE_DIR}/bin
            DOC "protobuf compiler"
            REQUIRED
            NO_DEFAULT_PATH
            )

    execute_process(COMMAND ${PROTOC} --version
        RESULT_VARIABLE _PROTOBUF_PROTOC_RESULT)

    if(_PROTOBUF_PROTOC_RESULT EQUAL "0")
        add_executable(protobuf::protoc IMPORTED)
        set_property(TARGET protobuf::protoc PROPERTY IMPORTED_LOCATION ${PROTOC})
    else()
        message(WARNING "Binary protoc not usable for some reason. "
            "The most likely is that we're running on Linux distribution based on musllibc, "
            "e.g. Alpine Linux.")
        # Binary protoc not usable for some reason (most likely because we're running on Alpine Linux)
        set(USE_BINARY_PROTOC FALSE)
    endif()
endif()

if(NOT USE_BINARY_PROTOC)
    message(WARNING "Binary protoc is not available or usable for your OS/architecture. "
        "We will need to compile protoc from source. This may take a while.")

    set(protobuf_BUILD_LIBPROTOC ON CACHE BOOL "" FORCE)
    set(protobuf_BUILD_PROTOBUF_BINARIES ON CACHE BOOL "" FORCE)
    set(protobuf_BUILD_PROTOC_BINARIES ON CACHE BOOL "" FORCE)
endif()

FetchContent_Declare(
        external_protobuf

        URL ${PROTOBUF_BASE_URL}/${PROTOBUF_ALT_VERSION}/protobuf-cpp-${Protobuf_VERSION}.tar.gz
        URL_HASH SHA256=58c8a18b4ec22655535c493155c5465a8903e8249094ceead87e00763bdbc44f
        # needs 3.24
        # DOWNLOAD_EXTRACT_TIMESTAMP false
        )


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

FetchContent_MakeAvailable(external_protobuf)

set_property(DIRECTORY ${external_protobuf_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

# set_property(TARGET libprotobuf PROPERTY POSITION_INDEPENDENT_CODE ON)

if(NOT TARGET protobuf::libprotobuf)
    add_library(protobuf::libprotobuf ALIAS libprotobuf)
endif()
