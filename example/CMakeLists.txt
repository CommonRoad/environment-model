# Create environment model test node
add_executable(env_model_example_node
    src/command_line_input.cpp
    src/predicate_manager.cpp
    src/main.cpp
    )

# Add a proper target for dl
# We use LINK_LIBRARIES_ONLY_TARGETS in order to only allow targets to be specified in target_link_libraries.
# However, there is no proper target for libdl as it is provided by the toolchain (glibc) itself.
# We create this toolchain library in order to still have a proper target for dl.
# See https://cmake.org/cmake/help/v3.25/prop_tgt/LINK_LIBRARIES_ONLY_TARGETS.html
add_library(toolchain::dl INTERFACE IMPORTED)
set_property(TARGET toolchain::dl PROPERTY IMPORTED_LIBNAME ${CMAKE_DL_LIBS})

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


