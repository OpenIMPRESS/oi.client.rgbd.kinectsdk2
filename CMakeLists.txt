CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
endif()


IF(NOT DEFINED CMAKE_BUILD_TYPE)
  # No effect for multi-configuration generators (e.g. for Visual Studio)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF()

PROJECT(oi.client.rgbd.kinectsdk2)
SET(CMAKE_CXX_STANDARD 11)

SET(MY_DIR ${PROJECT_SOURCE_DIR})

#SET(asio_ROOT_DIR "${MY_DIR}/../asio" CACHE STRING "Root folder of asio source.")
SET(DEPENDS_DIR "${MY_DIR}/../../dependencies" CACHE STRING "OpenIMPRESS dependency directory")

SET(LIBAV_ROOT_DIR "${DEPENDS_DIR}/libav" CACHE STRING "LIBAV directory")

SET(SHARED_OI_DIR "${MY_DIR}/../oi.client.rgbd.libfreenect2" CACHE STRING "OpenIMPRESS Shared")

FIND_PACKAGE(PkgConfig)
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" ) # oi.client.rgbd.libfreenect2 Modules

FIND_PACKAGE(ASIO REQUIRED)
FIND_PACKAGE(TurboJPEG REQUIRED)
FIND_PACKAGE(KinectSDK2 REQUIRED)
FIND_PACKAGE(LibAV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE=YES")
SET(application_src ${PROJECT_SOURCE_DIR}/src/main.cpp)

INCLUDE_DIRECTORIES(${ASIO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/headers)      
INCLUDE_DIRECTORIES(${TurboJPEG_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${SHARED_OI_DIR}/headers)
INCLUDE_DIRECTORIES(${LIBAV_INCLUDE_DIRS})

if(!KinectSDK2_FOUND)
  message( FATAL_ERROR "Kinect SDK could not be found." )
endif()

INCLUDE_DIRECTORIES(${KinectSDK2_INCLUDE_DIRS})
LINK_DIRECTORIES(${KinectSDK2_LIBRARY_DIRS})
LIST(APPEND application_LIBRARIES ${KinectSDK2_LIBRARIES})
LIST(APPEND application_LIBRARIES ${LIBAV_AVCODEC_LIBRARIES})
LIST(APPEND application_LIBRARIES ${LIBAV_AVUTIL_LIBRARIES})
LIST(APPEND application_LIBRARIES ${LIBAV_LIBRARIES})

LIST(APPEND application_LIBRARIES ${TurboJPEG_LIBRARIES})
LIST(APPEND application_headers
  ${PROJECT_SOURCE_DIR}/headers/config.h
  ${PROJECT_SOURCE_DIR}/headers/json.hpp
  ${PROJECT_SOURCE_DIR}/headers/stb_dxt.h
  ${SHARED_OI_DIR}/headers/UDPBase.hpp
  ${SHARED_OI_DIR}/headers/UDPConnector.hpp
  ${SHARED_OI_DIR}/headers/RGBDStreamer.hpp
  ${SHARED_OI_DIR}/headers/RecordState.hpp
  ${PROJECT_SOURCE_DIR}/headers/KinectSDK2Streamer.hpp
  ${PROJECT_SOURCE_DIR}/headers/AVEncodeTest.hpp
  
)
LIST(APPEND application_DLLS)
LIST(APPEND application_src
  ${SHARED_OI_DIR}/src/UDPBase.cpp
  ${SHARED_OI_DIR}/src/UDPConnector.cpp
  ${SHARED_OI_DIR}/src/RGBDStreamer.cpp
  ${SHARED_OI_DIR}/src/RecordState.cpp
  ${PROJECT_SOURCE_DIR}/src/KinectSDK2Streamer.cpp
  ${PROJECT_SOURCE_DIR}/src/AVEncodeTest.cpp
)
LIST(APPEND application_LIBRARIES
)

SOURCE_GROUP("Header Files" FILES ${application_headers})

ADD_EXECUTABLE(oi.client.rgbd.kinectsdk2
  ${application_src}
  ${application_headers}
)

TARGET_LINK_LIBRARIES(oi.client.rgbd.kinectsdk2
  ${application_LIBRARIES}
)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "oi.client.rgbd.kinectsdk2" )
ADD_CUSTOM_COMMAND(TARGET "oi.client.rgbd.kinectsdk2" POST_BUILD ${KinectSDK2_COMMANDS})

#SET(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations")
