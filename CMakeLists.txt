project(rimesa CXX C)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)

# Dependencies 
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# Dependency - The actual riSAM implementation
include(FetchContent)
FetchContent_Declare(
  risam
  GIT_REPOSITORY  https://github.com/rpl-cmu/risam-v2.git
  GIT_TAG         main
)
FetchContent_MakeAvailable(risam)


message(STATUS "================  rimesa  ======================")
# Build the rimesa library
file(GLOB_RECURSE rimesa_srcs "*.cpp" "*.h")
add_library(rimesa ${rimesa_srcs})
target_link_libraries(rimesa gtsam risam)
target_include_directories(rimesa PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/rimesa/include")