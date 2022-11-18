# Provides configure_package_config_file
include(CMakePackageConfigHelpers)
# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)

set(export_name ${PROJECT_NAME}_Targets)

# Citing CMake documentation:
# For regular executables, static libraries and shared libraries,
# the DESTINATION argument is not required.
# https://cmake.org/cmake/help/v3.25/command/install.html#targets
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    install(TARGETS commonroad_protobuf
        EXPORT ${export_name}
        FILE_SET commonroad_protobuf_headers)
    install(TARGETS env_model_core env_model_predicates
            EXPORT ${export_name}
            FILE_SET env_model_core_headers
            FILE_SET env_model_predicates_headers
            LIBRARY ARCHIVE RUNTIME)
    install(TARGETS env_model
            EXPORT ${export_name})
else()
    install(TARGETS env_model env_model_core env_model_predicates
            EXPORT ${export_name}
            LIBRARY ARCHIVE RUNTIME)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
endif()

set(env_model_install_cmakedir ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

set(env_model_config_file ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake)

configure_package_config_file(
        ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
        ${env_model_config_file}
        INSTALL_DESTINATION ${env_model_install_cmakedir})

# Export target configuration (for installation)
install(EXPORT ${export_name}
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${env_model_install_cmakedir}
        )


# Export target configuration (allows find_package to find local build tree without
# first installing it)
export(EXPORT ${export_name}
        FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        )

# Required for export()
foreach(foreign_target spdlog::spdlog yaml-cpp::yaml-cpp protobuf::libprotobuf)
    get_target_property(aliased_target ${foreign_target} ALIASED_TARGET)
    if(aliased_target)
        set(foreign_target ${aliased_target})
    endif()

    get_target_property(target_imported ${foreign_target} IMPORTED)
    if(NOT target_imported)
        message(VERBOSE "Exporting ${foreign_target} because it is built locally")
        install(TARGETS ${foreign_target}
            EXPORT ${export_name}
            LIBRARY ARCHIVE RUNTIME)
    endif()
endforeach()
