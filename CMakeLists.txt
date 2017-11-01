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

SET(asio_ROOT_DIR "${MY_DIR}/../asio" CACHE STRING "Root folder of asio source.")

SET(MY_DIR ${PROJECT_SOURCE_DIR})

FIND_PACKAGE(PkgConfig)

FIND_PACKAGE(ASIO REQUIRED)
FIND_PACKAGE(TurboJPEG REQUIRED)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE=YES" )

INCLUDE_DIRECTORIES(${ASIO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/headers)      
INCLUDE_DIRECTORIES(${TurboJPEG_INCLUDE_DIRS})


SET(application_src ${PROJECT_SOURCE_DIR}/src/main.cpp)

if(WIN32)
ELSE()
ENDIF()

LIST(APPEND application_LIBRARIES ${TurboJPEG_LIBRARIES})

LIST(APPEND application_headers
  ${PROJECT_SOURCE_DIR}/headers/config.h
  ${PROJECT_SOURCE_DIR}/headers/json.hpp
  ${PROJECT_SOURCE_DIR}/headers/stb_dxt.h
  ${PROJECT_SOURCE_DIR}/headers/viewer.h
)
LIST(APPEND application_DLLS)
LIST(APPEND application_src
  ${PROJECT_SOURCE_DIR}/src/UDPConnector.cpp
  ${PROJECT_SOURCE_DIR}/src/viewer.cpp
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

#SET(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations")
