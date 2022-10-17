include(FetchContent)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
  # We need the fix provided by 68e03ab240aa340b91f0b6fea8d382ef5cfb9258
  GIT_TAG 23299632c246b77937fb78e8607863a2f02e191b
  GIT_PROGRESS TRUE
  OVERRIDE_FIND_PACKAGE
  )

set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_BUILD_CMAKE_PACKAGE OFF CACHE INTERNAL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE INTERNAL "" FORCE)
#set(EIGEN_BUILD_CMAKE_PACKAGE OFF)

find_package(Eigen3 REQUIRED)

#FetchContent_MakeAvailable(Eigen3)
#FetchContent_GetProperties(Eigen3 SOURCE_DIR)

# set(Eigen3_FOUND ON CACHE BOOL "" FORCE)
# set(EIGEN3_INCLUDE_DIR "${Eigen3_SOURCE_DIR}" CACHE PATH "" FORCE)

install(TARGETS eigen
        EXPORT EnvironmentModel_Targets
        )
install(TARGETS eigen
        EXPORT fcl-targets
        )
install(TARGETS eigen
        EXPORT DrivabilityChecker_Targets
        )