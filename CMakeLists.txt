cmake_minimum_required(VERSION 3.10...3.18)


project(basalt)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

function(report_dependency_location DEP_NAME DEP_TARGET)
  if(${DEP_NAME}_DIR)
    # this is set if dependency is found via find_package
    set(ACTUAL_DEP_CMAKE_DIR ${${DEP_NAME}_DIR})
  else()
    if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.19.0")
      get_target_property(ACTUAL_DEP_CMAKE_DIR ${DEP_TARGET} SOURCE_DIR)
    else()
      # SOURCE_DIR works for interface targets only since 3.19
      set(ACTUAL_DEP_CMAKE_DIR "")
    endif()
  endif()
  get_target_property(ACTUAL_DEP_INCLUDE_DIR ${DEP_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
  message(STATUS "Using ${DEP_NAME} from: ${ACTUAL_DEP_CMAKE_DIR} (include: ${ACTUAL_DEP_INCLUDE_DIR})")
endfunction()


if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 3.4 REQUIRED)
endif()
report_dependency_location(Eigen3 Eigen3::Eigen)


if(NOT TARGET Sophus::Sophus)
  find_package(Sophus REQUIRED)
endif()
report_dependency_location(Sophus Sophus::Sophus)


if(NOT TARGET cereal::cereal)
  find_package(cereal REQUIRED)
endif()
report_dependency_location(cereal cereal::cereal)

if(NOT TARGET TBB::tbb)
  find_package(TBB REQUIRED)
endif()
report_dependency_location(TBB TBB::tbb)

find_package(
    OpenCV
    REQUIRED
    COMPONENTS
        core
        imgproc
        calib3d
        highgui
)
add_subdirectory(thirdparty/basalt_apriltag)


add_library(
    basalt
    SHARED
    src/calibration/aprilgrid.cpp
    src/calibration/calibraiton_helper.cpp
    src/calibration/cam_calib.cpp

)

target_link_libraries(
    basalt
    PUBLIC
        TBB::tbb
        Eigen3::Eigen
        Sophus::Sophus
        cereal::cereal
    PRIVATE
        basalt_apriltag
        opengv
        ${OpenCV_LIBS}
)

target_include_directories(
    basalt
    SYSTEM
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(basalt PUBLIC -DBASALT_INSTANTIATIONS_DOUBLE=On)


#install
install(TARGETS basalt
EXPORT BasaltTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

  install(DIRECTORY include/ DESTINATION include)
