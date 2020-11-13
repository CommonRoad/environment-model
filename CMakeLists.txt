cmake_minimum_required(VERSION 3.13)
project(env_model)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(CMAKE_CXX_STANDARD 17)

if(NOT EXISTS "${CRCCOSY_LIBRARY_DIR}")
    message(SEND_ERROR "Can't find the CommonRoad-Curvilinear-Coordinate System library in ${CRCCOSY_LIBRARY_DIR}")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

set(CRCCOSY_LIBRARY ${CRCCOSY_LIBRARY_DIR}/libcrccosy.a)
set(LIB_ENV_MODEL_INCLUDE "." ${CRCCOSY_LIBRARY_DIR}/cpp)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

file(GLOB_RECURSE env_files
        "env_model/road_network/*.h"
        "env_model/road_network/*.cpp"
        "env_model/road_network/lanelet/*.h"
        "env_model/road_network/lanelet/*.cpp"
        "env_model/road_network/regulatory_elements/*.h"
        "env_model/road_network/regulatory_elements/*.cpp"
        "env_model/road_network/intersection/*.h"
        "env_model/road_network/intersection/*.cpp"
        "env_model/obstacle/*.cpp"
        "env_model/obstacle/*.h"
        "env_model/geometry/*.h"
        "env_model/geometry/*.cpp"
        "env_model/interfaces/commonroad/pugi_xml/*.h"
        "env_model/interfaces/commonroad/pugi_xml/*.cpp"
        "env_model/interfaces/commonroad/*.h"
        "env_model/interfaces/commonroad/*.cpp"
        "env_model/auxiliaryDefs/*.h"
        "env_model/auxiliaryDefs/*.cpp"
        "env_model/interfaces/standalone/command_line_input.cpp"
        "env_model/interfaces/standalone/command_line_input.h"
        "env_model/predicates/*.h"
        "env_model/predicates/*.cpp"
        )

add_executable(env_model ${env_files} env_model/interfaces/standalone/main.cpp)
target_link_libraries (env_model Boost::filesystem Boost::program_options)

target_include_directories(env_model PUBLIC ${LIB_ENV_MODEL_INCLUDE})
target_link_libraries(env_model ${CRCCOSY_LIBRARY})

