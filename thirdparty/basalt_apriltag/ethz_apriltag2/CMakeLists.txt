cmake_minimum_required(VERSION 2.8)

project(ethz_apriltag2)

find_package(catkin REQUIRED COMPONENTS cmake_modules)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

# catkin_package(
#     DEPENDS eigen opencv
#     INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
#     LIBRARIES ${PROJECT_NAME}
# )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

# find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

# if(NOT TARGET tkData::data)
#     find_package(tkData REQUIRED)
# endif()

if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 3.4 REQUIRED)
endif()

add_definitions(-fPIC -O3)
# include_directories(include  ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} Eigen3::Eigen)


