cmake_minimum_required(VERSION 3.14)
project(gazebo_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

# dimos LCM message headers (fetched at configure time)
include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# gz-transport14 + gz-msgs11 + embedded gz-sim9 (Gazebo Ionic)
find_package(gz-transport14 REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(gz-sim9 REQUIRED)

# Headers shared with hardware native modules (NativeModule arg parser, etc.)
if(NOT DIMOS_COMMON_DIR)
  set(DIMOS_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../hardware/sensors/lidar/common)
endif()

add_executable(gazebo_native main.cpp)

target_include_directories(gazebo_native PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${DIMOS_COMMON_DIR}
)

target_link_libraries(gazebo_native PRIVATE
  ${LCM_LIBRARIES}
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-sim9::gz-sim9
  sodium  # transitive via zeromq → libgz-transport14, but not exported by config
)

target_link_directories(gazebo_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS gazebo_native DESTINATION bin)
