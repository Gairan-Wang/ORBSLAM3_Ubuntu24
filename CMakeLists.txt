cmake_minimum_required(VERSION 3.28)
project(my_orb)
SET(CMAKE_BUILD_TYPE Release)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" )
# add_definitions(-DCOMPILEDWITHC17)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.6)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.6 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Sophus REQUIRED)
# find_package(G2O REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/Camera
${PROJECT_SOURCE_DIR}/include/DUtils
# ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
${Sophus_INCLUDE_DIRS}
# ${G2O_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${DBoW2_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cpp
src/Tracking.cpp
src/LocalMapping.cpp
src/LoopClosing.cpp
src/ORBextractor.cpp
src/ORBmatcher.cpp
src/FrameDrawer.cpp
src/Converter.cpp
src/MapPoint.cpp
src/KeyFrame.cpp
src/Atlas.cpp
src/Map.cpp
src/MapDrawer.cpp
src/Optimizer.cpp
src/Frame.cpp
src/KeyFrameDatabase.cpp
src/Sim3Solver.cpp
src/Viewer.cpp
src/ImuTypes.cpp
src/G2oTypes.cpp
src/Camera/Pinhole.cpp
src/Camera/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cpp
src/TwoViewReconstruction.cpp
src/Config.cpp
src/Settings.cpp
src/DUtils/Random.cpp
src/DUtils/Timestamp.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${EIEGN3_LIBS}
   #  /usr/local/lib/libg2o_core.so 
   #  /usr/local/lib/libg2o_solver_eigen.so 
   #  /usr/local/lib/libg2o_solver_dense.so 
   #  /usr/local/lib/libg2o_types_sim3.so 
   #  /usr/local/lib/libg2o_types_sba.so
   #  /usr/local/lib/libg2o_csparse_extension.so
   #  /usr/local/lib/libg2o_types_slam3d.so
   #  /usr/local/lib/libg2o_opengl_helper.so
    /usr/local/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    /usr/lib/x86_64-linux-gnu/libboost_serialization.so 
    /usr/lib/x86_64-linux-gnu/libboost_wserialization.so 
    /usr/lib/x86_64-linux-gnu/libboost_filesystem.so 
    /usr/lib/x86_64-linux-gnu/libboost_system.so 
    /usr/lib/x86_64-linux-gnu/libboost_thread.so
    -lcrypto
    -lpthread
    )
   
#Monocular inertial Apps
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Apps/Monocular-Inertial)

# add_executable(mono_inertial_euroc
#                Apps/Monocular-Inertial/mono_inertial_euroc.cpp)
# target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

# add_executable(mono_inertial_tum_vi
#                Apps/Monocular-Inertial/mono_inertial_tum_vi.cpp)
# target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(mono_inertial_realsense_t265
#          Apps/Monocular-Inertial/mono_inertial_realsense_t265.cpp)
#     target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

#     add_executable(mono_inertial_realsense_D435i
#             Apps/Monocular-Inertial/mono_inertial_realsense_D435i.cpp)
#     target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
# endif()

# Stereo inertial Apps
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Apps/Stereo-Inertial)

add_executable(stereo_inertial_euroc
        Apps/Stereo-Inertial/stereo_inertial_euroc.cpp)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

# add_executable(stereo_inertial_tum_vi
#         Apps/Stereo-Inertial/stereo_inertial_tum_vi.cpp)
# target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

# Monocular examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Apps/Monocular)

# add_executable(mono_tum
#         Apps/Monocular/mono_tum.cpp)
# target_link_libraries(mono_tum ${PROJECT_NAME})

# add_executable(mono_kitti
#         Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})

# add_executable(mono_euroc
#         Apps/Monocular/mono_euroc.cpp)
# target_link_libraries(mono_euroc ${PROJECT_NAME})

# add_executable(mono_tum_vi
#         Examples/Monocular/mono_tum_vi.cc)
# target_link_libraries(mono_tum_vi ${PROJECT_NAME})