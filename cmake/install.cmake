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
        FILE_SET commonroad_protobuf_headers DESTINATION include
        )
    install(TARGETS env_model_core env_model_predicates
            EXPORT ${export_name}
            FILE_SET env_model_core_headers DESTINATION include
            FILE_SET env_model_predicates_headers DESTINATION include
            LIBRARY ARCHIVE RUNTIME)
    install(TARGETS env_model
            EXPORT ${export_name})
else()
    install(TARGETS env_model env_model_core env_model_predicates commonroad_protobuf
            EXPORT ${export_name}
            LIBRARY ARCHIVE RUNTIME)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
endif()

if(LOCAL_CRDC OR DEFINED ENV{CIBUILDWHEEL})
    # Using bundled crdc
    install(TARGETS crccosy EXPORT ${export_name})
endif()

# Define INSTALL_..._FROM_SYSTEM for each dependency depending on whether we're using
# our own version (via FetchContent) or a system version (via find_package)
# This information is used in the installed config file.
include(FetchContent)
foreach(fc_target Eigen3 spdlog yaml-cpp protobuf boost_src pugixml range-v3 tsl-robin-map)
    FetchContent_GetProperties(${fc_target})

    message(DEBUG "check ${fc_target} (variable name: ${fc_target}_SOURCE_DIR)")
    if(DEFINED ${fc_target}_SOURCE_DIR)
        message(DEBUG "defined: ${${fc_target}_SOURCE_DIR}")
        set(INSTALL_${fc_target}_FROM_SYSTEM OFF)
    else()
        message(DEBUG "not defined")
        set(INSTALL_${fc_target}_FROM_SYSTEM ON)
    endif()
endforeach()

set(env_model_install_cmakedir ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

set(env_model_config_file ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake)
set(env_model_version_file ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake)

configure_package_config_file(
        ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
        ${env_model_config_file}
        INSTALL_DESTINATION ${env_model_install_cmakedir})

write_basic_package_version_file(${env_model_version_file}
        COMPATIBILITY AnyNewerVersion)

install(FILES ${env_model_config_file}
              ${env_model_version_file}
        DESTINATION  ${env_model_install_cmakedir})

# Export target configuration (for installation)
install(EXPORT ${export_name}
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${env_model_install_cmakedir}
        )

# Export is disabled by default because it can cause issues with FetchContent in surprising ways
set(_enable_package_export FALSE)

if(_enable_package_export)
    # Export target configuration (allows find_package to find local build tree without
    # first installing it)
    export(EXPORT ${export_name}
            FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
            NAMESPACE ${PROJECT_NAME}::
            )
elseif(TARGET eigen)
    # Workaround for required Eigen export
    export(TARGETS eigen
            FILE ${PROJECT_BINARY_DIR}/eigen-export-private.cmake
            NAMESPACE ${PROJECT_NAME}_private::
            )
endif()

# Required for export()
foreach(foreign_target spdlog::spdlog yaml-cpp::yaml-cpp protobuf::libprotobuf Eigen3::Eigen)
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