# Create environment model test node
add_executable(env_model_example_node
    src/command_line_input.cpp
    src/predicate_manager.cpp
    src/main.cpp
    )

# Add definition for toolchain::dl
include(toolchain/ToolchainLibraries)

target_link_libraries(env_model_example_node
    PRIVATE
        OpenMP::OpenMP_CXX
        env_model
        Boost::program_options
        Boost::filesystem
        spdlog::spdlog
        yaml-cpp::yaml-cpp
        Threads::Threads
        toolchain::dl
    )


if(APPLE)
    # Required to find library -lomp on mac
    # MAC_LIBOMP_PATH defined in root CMakeLists.txt
    target_link_directories(env_model_example_node PUBLIC "${MAC_LIBOMP_PATH}/lib")
endif()

# this adds many warnings to the build. They usually help to find some bugs
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    if(ENV_MODEL_BUILD_EXECUTABLE)
        target_compile_options(env_model_example_node PRIVATE -Wall -Wextra -Wconversion -pedantic
                -Wfatal-errors)
    endif(ENV_MODEL_BUILD_EXECUTABLE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    if(ENV_MODEL_BUILD_EXECUTABLE)
        target_compile_options(env_model_example_node PRIVATE /W3 /WX)
    endif(ENV_MODEL_BUILD_EXECUTABLE)
endif()
