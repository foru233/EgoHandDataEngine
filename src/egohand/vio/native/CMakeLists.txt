cmake_minimum_required(VERSION 3.25)
project(egohand_vio_bridge LANGUAGES CXX)

if(NOT MSVC)
  message(FATAL_ERROR "This bridge currently reuses the existing MSVC ORB-SLAM3 build.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS OFF)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)
set(ORB_ROOT "${PROJECT_ROOT}/third_party/ORB_SLAM3")
if(NOT DEFINED CONDA_PREFIX)
  set(CONDA_PREFIX "$ENV{USERPROFILE}/miniconda3/envs/egohand-vio")
endif()
set(CONDA_LIB "${CONDA_PREFIX}/Library/lib")
set(CONDA_BIN "${CONDA_PREFIX}/Library/bin")
set(ORB_BUILD "${ORB_ROOT}/build_msvc_initial")

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT EXISTS "${ORB_BUILD}/ORB_SLAM3.lib")
  message(FATAL_ERROR "Existing ORB_SLAM3.lib not found: ${ORB_BUILD}")
endif()
if(NOT EXISTS "${ORB_ROOT}/Thirdparty/DBoW2/lib/DBoW2.lib")
  message(FATAL_ERROR "DBoW2 import library not found")
endif()
if(NOT EXISTS "${ORB_ROOT}/Thirdparty/g2o/bin/g2o.lib")
  message(FATAL_ERROR "g2o static library not found")
endif()

add_library(ORB_SLAM3 STATIC IMPORTED GLOBAL)
set_target_properties(ORB_SLAM3 PROPERTIES
  IMPORTED_LOCATION "${ORB_BUILD}/ORB_SLAM3.lib"
  INTERFACE_INCLUDE_DIRECTORIES "${ORB_ROOT};${ORB_ROOT}/include;${ORB_ROOT}/include/CameraModels;${ORB_ROOT}/Thirdparty/Sophus;${ORB_ROOT}/Thirdparty/DBoW2;${ORB_ROOT}/Thirdparty/g2o"
)

add_library(DBoW2 STATIC IMPORTED GLOBAL)
set_target_properties(DBoW2 PROPERTIES
  IMPORTED_LOCATION "${ORB_ROOT}/Thirdparty/DBoW2/lib/DBoW2.lib"
)

add_library(g2o STATIC IMPORTED GLOBAL)
set_target_properties(g2o PROPERTIES
  IMPORTED_LOCATION "${ORB_ROOT}/Thirdparty/g2o/bin/g2o.lib"
)

set(PANGO_LIBS
  "${CONDA_LIB}/pango_glgeometry.lib"
  "${CONDA_LIB}/pango_geometry.lib"
  "${CONDA_LIB}/pango_plot.lib"
  "${CONDA_LIB}/pango_python.lib"
  "${CONDA_LIB}/pango_scene.lib"
  "${CONDA_LIB}/pango_tools.lib"
  "${CONDA_LIB}/pango_display.lib"
  "${CONDA_LIB}/pango_vars.lib"
  "${CONDA_LIB}/pango_video.lib"
  "${CONDA_LIB}/pango_packetstream.lib"
  "${CONDA_LIB}/pango_windowing.lib"
  "${CONDA_LIB}/pango_opengl.lib"
  "${CONDA_LIB}/pango_image.lib"
  "${CONDA_LIB}/pango_core.lib"
  "${CONDA_LIB}/glew32.lib"
  opengl32.lib
  glu32.lib
  "${CONDA_LIB}/tinyobj.lib"
)
set(BOOST_LIBS
  "${CONDA_LIB}/boost_serialization.lib"
  "${CONDA_LIB}/boost_thread.lib"
  "${CONDA_LIB}/boost_atomic.lib"
  "${CONDA_LIB}/boost_chrono.lib"
  "${CONDA_LIB}/boost_date_time.lib"
  "${CONDA_LIB}/boost_container.lib"
)
set(OPENSSL_LIB "${CONDA_LIB}/libcrypto.lib")

add_library(egohand_vio_bridge SHARED vio_bridge.cpp vio_bridge.h)
target_compile_definitions(egohand_vio_bridge PRIVATE
  EGOHAND_VIO_BRIDGE_BUILD
  COMPILEDWITHC11 WINDOWS _WINDOWS _WIN_
  HAVE_EIGEN HAVE_GLEW
  BOOST_ATOMIC_DYN_LINK BOOST_ATOMIC_NO_LIB
  BOOST_CHRONO_DYN_LINK BOOST_CHRONO_NO_LIB
  BOOST_CONTAINER_DYN_LINK BOOST_CONTAINER_NO_LIB
  BOOST_DATE_TIME_DYN_LINK BOOST_DATE_TIME_NO_LIB
  BOOST_SERIALIZATION_DYN_LINK BOOST_SERIALIZATION_NO_LIB
  BOOST_THREAD_DYN_LINK BOOST_THREAD_NO_LIB
  "PANGO_DEFAULT_WIN_URI=\"winapi\""
)
target_compile_options(egohand_vio_bridge PRIVATE /bigobj /utf-8)
target_include_directories(egohand_vio_bridge PRIVATE
  "${ORB_ROOT}"
  "${ORB_ROOT}/include"
  "${ORB_ROOT}/include/CameraModels"
  "${ORB_ROOT}/Thirdparty/Sophus"
  "${ORB_ROOT}/Thirdparty/DBoW2"
  "${ORB_ROOT}/Thirdparty/g2o"
  "${CONDA_PREFIX}/Library/include"
  "${CONDA_PREFIX}/Library/include/eigen3"
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(egohand_vio_bridge PRIVATE
  ORB_SLAM3
  ${PANGO_LIBS}
  DBoW2
  ${OpenCV_LIBS}
  g2o
  ${BOOST_LIBS}
  "${OPENSSL_LIB}"
)

set_target_properties(egohand_vio_bridge PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
)

message(STATUS "ORB-SLAM3 import: ${ORB_BUILD}/ORB_SLAM3.lib")
message(STATUS "OpenCV: ${OpenCV_VERSION}")
message(STATUS "Conda runtime DLL directory: ${CONDA_BIN}")
