cmake_minimum_required(VERSION 3.20)
project(orbbec_foxglove_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ORBBEC_BUILD_FOXGLOVE_SINK "Build Foxglove publisher library target" ON)
option(ORBBEC_BUILD_BRIDGE_APP "Build bridge executable app target" ON)
option(ORBBEC_BUILD_PRODUCER_APP "Build producer-only executable app target (no Foxglove dependency)" OFF)

if(ORBBEC_BUILD_BRIDGE_APP AND NOT ORBBEC_BUILD_FOXGLOVE_SINK)
  message(FATAL_ERROR "ORBBEC_BUILD_BRIDGE_APP requires ORBBEC_BUILD_FOXGLOVE_SINK=ON")
endif()

set(ORBBEC_SDK_ROOT "" CACHE PATH "Path to Orbbec SDK root (must contain include/libobsensor and lib)")
set(FOXGLOVE_SDK_ROOT "" CACHE PATH "Path to Foxglove C++ SDK root (must contain include, src, lib)")

if(NOT EXISTS "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
  message(FATAL_ERROR
    "ORBBEC_SDK_ROOT is invalid. Expected: ${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
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

function(orbbec_apply_common_cxx_options target_name)
  if(WIN32)
    target_compile_definitions(${target_name} PRIVATE
      _CRT_SECURE_NO_WARNINGS
      NOMINMAX
    )
    target_compile_options(${target_name} PRIVATE
      /EHsc
      /utf-8
    )
  endif()
endfunction()

function(orbbec_add_windows_runtime_assets target_name)
  if(NOT WIN32)
    return()
  endif()

  if(ORBBEC_SDK_DLL)
    add_custom_command(TARGET ${target_name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ORBBEC_SDK_DLL}"
        "$<TARGET_FILE_DIR:${target_name}>/OrbbecSDK.dll"
    )
  else()
    message(WARNING "Could not find OrbbecSDK.dll under ${ORBBEC_SDK_ROOT}; runtime may fail.")
  endif()

  if(EXISTS "${ORBBEC_SDK_ROOT}/bin/OrbbecSDKConfig.xml")
    add_custom_command(TARGET ${target_name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ORBBEC_SDK_ROOT}/bin/OrbbecSDKConfig.xml"
        "$<TARGET_FILE_DIR:${target_name}>/OrbbecSDKConfig.xml"
    )
  endif()

  if(EXISTS "${ORBBEC_SDK_ROOT}/bin/extensions")
    add_custom_command(TARGET ${target_name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ORBBEC_SDK_ROOT}/bin/extensions"
        "$<TARGET_FILE_DIR:${target_name}>/extensions"
    )
  endif()
endfunction()

# Reusable core library: producer + shared interfaces.
add_library(orbbec_core STATIC
  src/orbbec_producer.cpp
)
add_library(orbbec::core ALIAS orbbec_core)

target_include_directories(orbbec_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${ORBBEC_SDK_ROOT}/include"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(orbbec_core
  PUBLIC
    ${ORBBEC_SDK_LIBRARY}
    ${OpenCV_LIBS}
    Threads::Threads
)

orbbec_apply_common_cxx_options(orbbec_core)

if(ORBBEC_BUILD_FOXGLOVE_SINK)
  set(FOXGLOVE_CPP_INCLUDE_DIR "")
  set(FOXGLOVE_CPP_SRC_DIR "")
  set(FOXGLOVE_C_INCLUDE_DIR "")
  set(FOXGLOVE_C_LIBRARY_HINTS "")

  if(EXISTS "${FOXGLOVE_SDK_ROOT}/include/foxglove/server.hpp" AND
     EXISTS "${FOXGLOVE_SDK_ROOT}/src")
    # Prebuilt SDK layout (or sdk/cpp/foxglove subdir used as root).
    set(FOXGLOVE_CPP_INCLUDE_DIR "${FOXGLOVE_SDK_ROOT}/include")
    set(FOXGLOVE_CPP_SRC_DIR "${FOXGLOVE_SDK_ROOT}/src")
    if(EXISTS "${FOXGLOVE_SDK_ROOT}/include/foxglove-c/foxglove-c.h")
      set(FOXGLOVE_C_INCLUDE_DIR "${FOXGLOVE_SDK_ROOT}/include")
    endif()
    list(APPEND FOXGLOVE_C_LIBRARY_HINTS
      "${FOXGLOVE_SDK_ROOT}/lib"
      "${FOXGLOVE_SDK_ROOT}/lib64"
      "${FOXGLOVE_SDK_ROOT}/../../c/target/release"
      "${FOXGLOVE_SDK_ROOT}/../../target/release"
      "${FOXGLOVE_SDK_ROOT}/../c/target/release"
      "${FOXGLOVE_SDK_ROOT}/../target/release"
      "${FOXGLOVE_SDK_ROOT}/../../c/target/debug"
      "${FOXGLOVE_SDK_ROOT}/../../target/debug"
      "${FOXGLOVE_SDK_ROOT}/../c/target/debug"
      "${FOXGLOVE_SDK_ROOT}/../target/debug"
    )
  elseif(EXISTS "${FOXGLOVE_SDK_ROOT}/cpp/foxglove/include/foxglove/server.hpp" AND
         EXISTS "${FOXGLOVE_SDK_ROOT}/cpp/foxglove/src")
    # Foxglove SDK monorepo layout.
    set(FOXGLOVE_CPP_INCLUDE_DIR "${FOXGLOVE_SDK_ROOT}/cpp/foxglove/include")
    set(FOXGLOVE_CPP_SRC_DIR "${FOXGLOVE_SDK_ROOT}/cpp/foxglove/src")
    if(EXISTS "${FOXGLOVE_SDK_ROOT}/c/include/foxglove-c/foxglove-c.h")
      set(FOXGLOVE_C_INCLUDE_DIR "${FOXGLOVE_SDK_ROOT}/c/include")
    endif()
    list(APPEND FOXGLOVE_C_LIBRARY_HINTS
      "${FOXGLOVE_SDK_ROOT}/lib"
      "${FOXGLOVE_SDK_ROOT}/c/target/release"
      "${FOXGLOVE_SDK_ROOT}/target/release"
      "${FOXGLOVE_SDK_ROOT}/c/target/debug"
      "${FOXGLOVE_SDK_ROOT}/target/debug"
    )
  else()
    message(FATAL_ERROR
      "FOXGLOVE_SDK_ROOT is invalid. Expected one of:\n"
      "  - <root>/include/foxglove/server.hpp and <root>/src\n"
      "  - <root>/cpp/foxglove/include/foxglove/server.hpp and <root>/cpp/foxglove/src")
  endif()

  if(FOXGLOVE_C_INCLUDE_DIR STREQUAL "")
    if(EXISTS "${FOXGLOVE_SDK_ROOT}/c/include/foxglove-c/foxglove-c.h")
      set(FOXGLOVE_C_INCLUDE_DIR "${FOXGLOVE_SDK_ROOT}/c/include")
    elseif(EXISTS "${FOXGLOVE_SDK_ROOT}/../c/include/foxglove-c/foxglove-c.h")
      set(FOXGLOVE_C_INCLUDE_DIR "${FOXGLOVE_SDK_ROOT}/../c/include")
    elseif(EXISTS "${FOXGLOVE_SDK_ROOT}/../../c/include/foxglove-c/foxglove-c.h")
      set(FOXGLOVE_C_INCLUDE_DIR "${FOXGLOVE_SDK_ROOT}/../../c/include")
    endif()
  endif()

  if(FOXGLOVE_C_INCLUDE_DIR STREQUAL "")
    message(FATAL_ERROR
      "Could not find foxglove-c headers. Expected foxglove-c/foxglove-c.h near FOXGLOVE_SDK_ROOT=${FOXGLOVE_SDK_ROOT}")
  endif()

  list(REMOVE_DUPLICATES FOXGLOVE_C_LIBRARY_HINTS)

  find_library(FOXGLOVE_C_LIBRARY
    NAMES foxglove libfoxglove
    PATHS ${FOXGLOVE_C_LIBRARY_HINTS}
    NO_DEFAULT_PATH
  )

  if(NOT FOXGLOVE_C_LIBRARY)
    string(JOIN ", " FOXGLOVE_C_LIBRARY_HINTS_STRING ${FOXGLOVE_C_LIBRARY_HINTS})
    message(FATAL_ERROR
      "Could not find foxglove C library (libfoxglove.*). Searched: ${FOXGLOVE_C_LIBRARY_HINTS_STRING}")
  endif()

  file(GLOB FOXGLOVE_CPP_SOURCES
    "${FOXGLOVE_CPP_SRC_DIR}/*.cpp"
    "${FOXGLOVE_CPP_SRC_DIR}/server/*.cpp"
  )

  if(FOXGLOVE_CPP_SOURCES STREQUAL "")
    message(FATAL_ERROR
      "No Foxglove C++ sources found under ${FOXGLOVE_CPP_SRC_DIR}")
  endif()

  # Local static build of Foxglove C++ layer to keep sink reusable by link target.
  add_library(orbbec_foxglove_cpp STATIC
    ${FOXGLOVE_CPP_SOURCES}
  )
  add_library(orbbec::foxglove_cpp ALIAS orbbec_foxglove_cpp)
  target_include_directories(orbbec_foxglove_cpp
    PUBLIC
      "${FOXGLOVE_CPP_INCLUDE_DIR}"
      "${FOXGLOVE_C_INCLUDE_DIR}"
  )
  orbbec_apply_common_cxx_options(orbbec_foxglove_cpp)

  add_library(orbbec_foxglove_sink STATIC
    src/foxglove_publisher.cpp
  )
  add_library(orbbec::foxglove_sink ALIAS orbbec_foxglove_sink)

  target_include_directories(orbbec_foxglove_sink
    PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
      "${FOXGLOVE_CPP_INCLUDE_DIR}"
  )

  target_link_libraries(orbbec_foxglove_sink
    PUBLIC
      orbbec::core
      orbbec::foxglove_cpp
      ${FOXGLOVE_C_LIBRARY}
  )

  orbbec_apply_common_cxx_options(orbbec_foxglove_sink)

  if(WIN32)
    target_link_libraries(orbbec_foxglove_sink
      PUBLIC
        ws2_32
        crypt32
        ntdll
        userenv
        secur32
        bcrypt
        iphlpapi
    )
  endif()
endif()

if(ORBBEC_BUILD_BRIDGE_APP)
  add_executable(orbbec_foxglove_bridge
    src/main.cpp
  )

  target_include_directories(orbbec_foxglove_bridge PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(orbbec_foxglove_bridge PRIVATE
    orbbec::core
    orbbec::foxglove_sink
  )

  orbbec_apply_common_cxx_options(orbbec_foxglove_bridge)
  orbbec_add_windows_runtime_assets(orbbec_foxglove_bridge)
endif()

if(ORBBEC_BUILD_PRODUCER_APP)
  add_executable(orbbec_camera_producer
    src/producer_main.cpp
  )

  target_include_directories(orbbec_camera_producer PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(orbbec_camera_producer PRIVATE
    orbbec::core
  )

  orbbec_apply_common_cxx_options(orbbec_camera_producer)
  orbbec_add_windows_runtime_assets(orbbec_camera_producer)
endif()
