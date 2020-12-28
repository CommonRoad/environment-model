file(
        GLOB_RECURSE
        SRC_GEN
        "./*.cpp"
        "./*.h"
)


# Include Curvilinear Coordinate System
if(NOT EXISTS "${CRCCOSY_LIBRARY_DIR}")
    message(SEND_ERROR "Can't find the CommonRoad-Curvilinear-Coordinate System library in ${CRCCOSY_LIBRARY_DIR}")
endif()
set(CRCCOSY_LIBRARY ${CRCCOSY_LIBRARY_DIR}/libcrccosy.a)
set(LIB_CRCCOSY_INCLUDE ${CRCCOSY_LIBRARY_DIR}/cpp)

# Include Eigen3, Boost filesystem, and OpenMP
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(OpenMP)
if(NOT OpenMP_CXX_FOUND)
    message(SEND_ERROR "Can't find the OpenMP package")
endif()

# Create environment model library
#set(LIB_CR_ENV_MODEL_SRC ${ENV_MODEL_SRC_FILES})
add_library(env_model STATIC ${SRC_GEN})
target_compile_features(env_model
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET env_model PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET env_model PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
target_include_directories(env_model PUBLIC ".")
target_link_libraries (env_model
        ${CRCCOSY_LIBRARY}
        Boost::filesystem
        Boost::program_options
        OpenMP::OpenMP_CXX
        Eigen3::Eigen)

# Create environment model test node
add_executable(env_model_example_node interfaces/standalone/main.cpp)
#target_compile_features(env_model_example_node
#        PUBLIC cxx_lambdas cxx_auto_type
#        PRIVATE cxx_lambdas cxx_auto_type)
#set_property(TARGET env_model_example_node PROPERTY POSITION_INDEPENDENT_CODE ON)
#set_property(TARGET env_model_example_node PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
target_link_libraries (env_model_example_node env_model)
