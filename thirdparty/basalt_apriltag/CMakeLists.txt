cmake_minimum_required(VERSION 3.2...3.18)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

#file(GLOB APRILTAG_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/apriltag-2016-12-01/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/apriltag-2016-12-01/common/*.c")
#include_directories(apriltag-2016-12-01)

include_directories(${OpenCV_INCLUDE_DIR})

file(GLOB APRILTAG_SRCS "ethz_apriltag2/src/*.cc")
include_directories(ethz_apriltag2/include)

# if(NOT TARGET tkData::data)
#     find_package(tkData REQUIRED)
# endif()

if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen REQUIRED)
endif()


include_directories(../../include)

add_library(basalt_apriltag STATIC ${APRILTAG_SRCS} include/basalt/utils/apriltag.h src/apriltag.cpp)

target_include_directories(basalt_apriltag PUBLIC include)
target_link_libraries(basalt_apriltag PUBLIC ${OpenCV_LIBS} Sophus::Sophus Eigen3::Eigen ) 
target_include_directories(basalt_apriltag
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)

