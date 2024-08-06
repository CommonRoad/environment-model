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

    # Fix mismatch between version systems in newer Protobuf versions
    if(Protobuf_VERSION VERSION_GREATER_EQUAL "4.0.0" AND Protobuf_VERSION VERSION_LESS "5.0.0")
        set(_protoc_version "4.${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}")
    else()
        set(_protoc_version ${_PROTOBUF_PROTOC_EXECUTABLE_VERSION})
    endif()

    # NOTE: Protobuf_VERSION is the one found by find_package(Protobuf) above
    if(NOT "${_protoc_version}" VERSION_EQUAL "${Protobuf_VERSION}")
        message(STATUS "Found protoc version:        ${_protoc_version}")
        message(STATUS "Found libprotobuf version:   ${Protobuf_VERSION}")
        message(FATAL_ERROR "Unfortunately, the system protobuf installation is unusable "
            "due to a detected mismatch between the Protobuf compiler and the Protobuf libraries. "
            "We can't proceed at this point since CMake already added the corresponding targets, "
            "and there is no way to remove those targets now. "
            "Please rerun the configuration, adding -DCOMMONROAD_SYSTEM_PROTOBUF=OFF to the "
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

set(Protobuf_VERSION "3.21.9")

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
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64")
        set(PROTOC_ARCH "win64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
        set(PROTOC_ARCH "win32")
    else()
        message(WARNING "Unknown system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
        set(USE_BINARY_PROTOC FALSE)
    endif()
elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(PROTOC_ARCH "osx-x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
        set(PROTOC_ARCH "osx-aarch_64")
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

set(protobuf_base_url "https://github.com/protocolbuffers/protobuf/releases/download")
set(protobuf_url ${protobuf_base_url}/${PROTOBUF_ALT_VERSION}/protobuf-cpp-${Protobuf_VERSION}.tar.gz)

function(declare_arch arch hash)
    set(protoc_url ${protobuf_base_url}/v${PROTOBUF_ALT_NUMERIC_VERSION}/protoc-${PROTOBUF_ALT_NUMERIC_VERSION}-${arch}.zip)
    FetchContent_Declare(
        protoc_bin_${arch}
        URL ${protoc_url}
        URL_HASH ${hash}
    )
endfunction()

# set(PROTOC_ARCH "win64")

declare_arch("linux-x86_64" SHA256=3cd951aff8ce713b94cde55e12378f505f2b89d47bf080508cf77e3934f680b6)
declare_arch("linux-x86_32" SHA256=2cd017cf7d0a75bd95e65c0b70ff27603fafa87a39230e0f5bee5f8cc79a436d)
declare_arch("win32" SHA256=0f9197cdda89c92dcab414540afe4afa01dc6be41495f83bec6042d45d0b7eb6)
declare_arch("win64" SHA256=784d100b65c8eeb841bffdb885332391321740064865ead1ebc29561ed66cee1)
declare_arch("osx-x86_64" SHA256=a6419520a063242b0dedd433cbfc617424da2e8357ef96bf694d6ba3bca51887)
declare_arch("osx-aarch_64" SHA256=d935f396a05cb02d4a1338db181c78f47884466a9f57d5ed4b7a4811816b69cf)

if(USE_BINARY_PROTOC)
    set(protobuf_BUILD_LIBPROTOC OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_PROTOBUF_BINARIES OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_PROTOC_BINARIES OFF CACHE BOOL "" FORCE)

    message(STATUS "Trying to use binary protoc")
    message(VERBOSE "Protoc system/architecture: ${PROTOC_ARCH}")
    message(VERBOSE "Protoc download URL: ${protoc_url}")

    FetchContent_MakeAvailable(protoc_bin_${PROTOC_ARCH})

    find_program(PROTOC
            NAMES protoc
            HINTS ${protoc_bin_${PROTOC_ARCH}_SOURCE_DIR}/bin
            DOC "protobuf compiler"
            REQUIRED
            NO_DEFAULT_PATH
            )

    execute_process(COMMAND ${PROTOC} --version
        OUTPUT_QUIET
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

        URL ${protobuf_url}
        URL_HASH SHA256=bddc5dd16da45c89a510704683e02ba08c30af78fe092d255cf25b9b01259405
        )


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

FetchContent_MakeAvailable(external_protobuf)

set_property(DIRECTORY ${external_protobuf_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

# set_property(TARGET libprotobuf PROPERTY POSITION_INDEPENDENT_CODE ON)

if(NOT TARGET protobuf::libprotobuf)
    add_library(protobuf::libprotobuf ALIAS libprotobuf)
endif()
#Protobuf
