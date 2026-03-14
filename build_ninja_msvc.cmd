@echo off
setlocal
cd /d "%~dp0"

if not defined VSCMD_VER (
  call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64
  if errorlevel 1 exit /b 1
)

set "CMAKE_EXE=C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA_EXE=C:/Program Files (x86)/Microsoft Visual Studio/2019/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja/ninja.exe"
set "ORBBEC_SDK_ROOT=C:/Program Files/OrbbecSDK 2.7.6"
set "FOXGLOVE_SDK_ROOT=C:/Users/USER/Documents/amr_ws/foxglove-sdk"
set "VCPKG_TOOLCHAIN=C:/Users/USER/Documents/amr_ws/vcpkg/scripts/buildsystems/vcpkg.cmake"

if exist "C:\Users\USER\Documents\amr_ws\vcpkg\scripts\buildsystems\vcpkg.cmake" (
  "%CMAKE_EXE%" ^
    -S . ^
    -B build-ninja-msvc ^
    -G Ninja ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
    -DORBBEC_SDK_ROOT="%ORBBEC_SDK_ROOT%" ^
    -DFOXGLOVE_SDK_ROOT="%FOXGLOVE_SDK_ROOT%" ^
    -DCMAKE_TOOLCHAIN_FILE="%VCPKG_TOOLCHAIN%" ^
    -DVCPKG_TARGET_TRIPLET=x64-windows
) else (
  "%CMAKE_EXE%" ^
    -S . ^
    -B build-ninja-msvc ^
    -G Ninja ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
    -DORBBEC_SDK_ROOT="%ORBBEC_SDK_ROOT%" ^
    -DFOXGLOVE_SDK_ROOT="%FOXGLOVE_SDK_ROOT%"
)
if errorlevel 1 exit /b 1

"%CMAKE_EXE%" --build build-ninja-msvc
exit /b %errorlevel%
