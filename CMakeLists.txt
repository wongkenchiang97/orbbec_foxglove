cmake_minimum_required(VERSION 3.20)
project(orbbec_foxglove_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ORBBEC_SDK_ROOT "" CACHE PATH "Path to Orbbec SDK root (must contain include/libobsensor and lib)")
set(FOXGLOVE_SDK_ROOT "" CACHE PATH "Path to Foxglove C++ SDK root (must contain include, src, lib)")

if(NOT EXISTS "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
  message(FATAL_ERROR
    "ORBBEC_SDK_ROOT is invalid. Expected: ${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
endif()

if(NOT EXISTS "${FOXGLOVE_SDK_ROOT}/include/foxglove/server.hpp")
  message(FATAL_ERROR
    "FOXGLOVE_SDK_ROOT is invalid. Expected: ${FOXGLOVE_SDK_ROOT}/include/foxglove/server.hpp")
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Threads REQUIRED)

find_library(ORBBEC_SDK_LIBRARY
  NAMES OrbbecSDK libOrbbecSDK
  PATHS
    "${ORBBEC_SDK_ROOT}/lib"
    "${ORBBEC_SDK_ROOT}/lib/x64"
    "${ORBBEC_SDK_ROOT}/bin"
  NO_DEFAULT_PATH
)

if(NOT ORBBEC_SDK_LIBRARY)
  message(FATAL_ERROR
    "Could not find Orbbec SDK library in ${ORBBEC_SDK_ROOT}/lib or ${ORBBEC_SDK_ROOT}/lib/x64")
endif()

find_file(ORBBEC_SDK_DLL
  NAMES OrbbecSDK.dll
  PATHS
    "${ORBBEC_SDK_ROOT}/bin"
    "${ORBBEC_SDK_ROOT}/lib"
    "${ORBBEC_SDK_ROOT}/lib/x64"
  NO_DEFAULT_PATH
)

find_library(FOXGLOVE_C_LIBRARY
  NAMES foxglove libfoxglove
  PATHS "${FOXGLOVE_SDK_ROOT}/lib"
  NO_DEFAULT_PATH
)

if(NOT FOXGLOVE_C_LIBRARY)
  message(FATAL_ERROR
    "Could not find foxglove C library in ${FOXGLOVE_SDK_ROOT}/lib")
endif()

file(GLOB FOXGLOVE_CPP_SOURCES
  "${FOXGLOVE_SDK_ROOT}/src/*.cpp"
  "${FOXGLOVE_SDK_ROOT}/src/server/*.cpp"
)

if(FOXGLOVE_CPP_SOURCES STREQUAL "")
  message(FATAL_ERROR
    "No Foxglove C++ sources found under ${FOXGLOVE_SDK_ROOT}/src")
endif()

add_executable(orbbec_foxglove_bridge
  src/main.cpp
  src/orbbec_producer.cpp
  src/foxglove_publisher.cpp
  ${FOXGLOVE_CPP_SOURCES}
)

target_include_directories(orbbec_foxglove_bridge PRIVATE
  "${CMAKE_SOURCE_DIR}/include"
  "${ORBBEC_SDK_ROOT}/include"
  "${FOXGLOVE_SDK_ROOT}/include"
)

target_link_libraries(orbbec_foxglove_bridge PRIVATE
  ${ORBBEC_SDK_LIBRARY}
  ${FOXGLOVE_C_LIBRARY}
  ${OpenCV_LIBS}
  Threads::Threads
)

if(WIN32)
  target_compile_definitions(orbbec_foxglove_bridge PRIVATE
    _CRT_SECURE_NO_WARNINGS
    NOMINMAX
  )
  target_compile_options(orbbec_foxglove_bridge PRIVATE
    /EHsc
    /utf-8
  )
  target_link_libraries(orbbec_foxglove_bridge PRIVATE
    ws2_32
    crypt32
    ntdll
    userenv
    secur32
    bcrypt
    iphlpapi
  )

  if(ORBBEC_SDK_DLL)
    add_custom_command(TARGET orbbec_foxglove_bridge POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ORBBEC_SDK_DLL}"
        "$<TARGET_FILE_DIR:orbbec_foxglove_bridge>/OrbbecSDK.dll"
    )
  else()
    message(WARNING "Could not find OrbbecSDK.dll under ${ORBBEC_SDK_ROOT}; runtime may fail.")
  endif()

  if(EXISTS "${ORBBEC_SDK_ROOT}/bin/OrbbecSDKConfig.xml")
    add_custom_command(TARGET orbbec_foxglove_bridge POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ORBBEC_SDK_ROOT}/bin/OrbbecSDKConfig.xml"
        "$<TARGET_FILE_DIR:orbbec_foxglove_bridge>/OrbbecSDKConfig.xml"
    )
  endif()

  if(EXISTS "${ORBBEC_SDK_ROOT}/bin/extensions")
    add_custom_command(TARGET orbbec_foxglove_bridge POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ORBBEC_SDK_ROOT}/bin/extensions"
        "$<TARGET_FILE_DIR:orbbec_foxglove_bridge>/extensions"
    )
  endif()

endif()
