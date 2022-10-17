include(FetchContent)

#set(protobuf_BUILD_SHARED_LIBS OFF)
#set(protobuf_BUILD_TESTS OFF)
#set(protobuf_BUILD_PROTOC_BINARIES OFF)
#set(protobuf_BUILD_EXAMPLES ON CACHE BOOL "")
#set(protobuf_BUILD_LIBPROTOC ON CACHE BOOL "")
#set(protobuf_BUILD_TESTS ON CACHE BOOL "")
#set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "")
#set(protobuf_BUILD_PROTOC_BINARIES ON CACHE BOOL "")
#set(protobuf_MODULE_COMPATIBLE ON CACHE BOOL "")


set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_LIBPROTOC OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_PROTOBUF_BINARIES OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_PROTOC_BINARIES OFF CACHE BOOL "" FORCE)
set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
set(protobuf_WITH_ZLIB OFF CACHE BOOL "" FORCE)
# set(protobuf_MODULE_COMPATIBLE ON CACHE BOOL "")

set(ABSL_USE_EXTERNAL_GOOGLETEST ON CACHE BOOL "" FORCE)
set(protobuf_USE_EXTERNAL_GTEST ON CACHE BOOL "" FORCE)


find_library(libatomic atomic)
add_library(atomic INTERFACE IMPORTED ${libatomic})
# find_library(lib_dl ${CMAKE_DL_LIBS})
message(STATUS "libatomic:  ${libatomic}")

#find_package(Protobuf REQUIRED)
set(Protobuf_VERSION "3.21.5")

# Protobuf_VERSION:      3.21.5
# PROTOBUF_TAG:         v3.21.5
# PROTOBUF_ALT_VERSION:   v21.5
message(STATUS "Protobuf ver=${Protobuf_VERSION}")
set(PROTOBUF_TAG "v${Protobuf_VERSION}")
message(VERBOSE "Protobuf tag=${PROTOBUF_TAG}")
if (Protobuf_VERSION VERSION_GREATER_EQUAL "3.21.0")
        string(REGEX MATCH "3.([0-9.]+)" _IGNORED_ ${Protobuf_VERSION})
        set(PROTOBUF_ALT_VERSION "v${CMAKE_MATCH_1}")
else()
        set(PROTOBUF_ALT_VERSION ${PROTOBUF_TAG})
endif()
# assert(

message(VERBOSE "Protobuf alt version=${PROTOBUF_ALT_VERSION}")

set(PROTOBUF_BASE_URL "https://github.com/protocolbuffers/protobuf/releases/download")

FetchContent_Declare(
  external_protobuf_protoc
  URL  https://github.com/protocolbuffers/protobuf/releases/download/v21.6/protoc-21.6-linux-x86_64.zip
)
FetchContent_MakeAvailable(external_protobuf_protoc)
FetchContent_GetProperties(external_protobuf_protoc)
message(STATUS "looking at: ${external_protobuf_protoc_SOURCE_DIR}/bin")

find_program(PROTOC
        NAMES protoc
        HINTS ${external_protobuf_protoc_SOURCE_DIR}/bin
        DOC "protobuf compiler"
        REQUIRED
        NO_DEFAULT_PATH
        )

FetchContent_Declare(
        external_protobuf

        URL ${PROTOBUF_BASE_URL}/${PROTOBUF_ALT_VERSION}/protobuf-cpp-${Protobuf_VERSION}.tar.gz
        # URL_HASH SHA256=58c8a18b4ec22655535c493155c5465a8903e8249094ceead87e00763bdbc44f
        # needs 3.24
        # DOWNLOAD_EXTRACT_TIMESTAMP false
        #URL https://github.com/protocolbuffers/protobuf/releases/download/v21.6/protobuf-cpp-3.21.6.tar.gz
        # GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
        #GIT_TAG       v21.6
        #GIT_TAG        0264866ce665911¹ħ787ba5ca0faf07a5efe2899dd
        #GIT_TAG        ${PROTOBUF_TAG}
        #GIT_PROGRESS   true
        #GIT_SHALLOW    true
        # GIT_SUBMODULES_RECURSE true
        )

#set(protobuf_BUILD_SHARED_LIBS OFF)
#set(protobuf_BUILD_TESTS OFF)
#set(protobuf_BUILD_PROTOC_BINARIES OFF)
#set(protobuf_MODULE_COMPATIBLE ON)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

FetchContent_MakeAvailable(external_protobuf)
FetchContent_GetProperties(external_protobuf SOURCE_DIR)
message(STATUS "epSD: ${external_protobuf_SOURCE_DIR}")

set_property(DIRECTORY ${external_protobuf_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
#set_property(TARGET libprotobuf-lite PROPERTY EXCLUDE_FROM_ALL ON)

# set_property(TARGET libprotobuf PROPERTY POSITION_INDEPENDENT_CODE ON)

# list(APPEND CMAKE_MODULE_PATH ${external_protobuf_SOURCE_DIR}/cmake)

#include(protobuf-generate)

add_library(protobuf::libprotobuf ALIAS libprotobuf)

add_executable(protobuf::protoc IMPORTED)
set_property(TARGET protobuf::protoc PROPERTY IMPORTED_LOCATION ${PROTOC})