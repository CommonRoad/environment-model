include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    Eigen3
    GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
    # We need the fix provided by 68e03ab240aa340b91f0b6fea8d382ef5cfb9258
    GIT_TAG 23299632c246b77937fb78e8607863a2f02e191b

    # SYSTEM
    FIND_PACKAGE_ARGS 3.3.7
)

set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_BUILD_CMAKE_PACKAGE OFF CACHE INTERNAL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE INTERNAL "" FORCE)

FetchContent_MakeAvailable(Eigen3)

if(TARGET eigen)
    install(TARGETS eigen
            EXPORT EnvironmentModel_Targets
            )
endif()
