include(FetchContent)
include(FetchContentHelper)

FetchContent_Declare_Fallback(
    googletest

    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.12.1
    GIT_PROGRESS   true

    SYSTEM
    FIND_PACKAGE_ARGS NAMES GTest
)
set(INSTALL_GTEST OFF)
FetchContent_MakeAvailable(googletest)

if(NOT GTest_FOUND)
    # Add google-test
    mark_as_advanced(
            BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
            gmock_build_tests gtest_build_samples gtest_build_tests
            gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
    )

    if(TARGET gtest AND TARGET gtest_main AND TARGET gmock AND TARGET gmock_main)
        set_target_properties(gtest PROPERTIES FOLDER extern)
        set_target_properties(gtest_main PROPERTIES FOLDER extern)
        set_target_properties(gmock PROPERTIES FOLDER extern)
        set_target_properties(gmock_main PROPERTIES FOLDER extern)
    else()
        message(AUTHOR_WARNING "Could not find local GTest targets, please report this")
    endif()
endif()