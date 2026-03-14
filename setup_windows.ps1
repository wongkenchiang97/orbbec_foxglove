param(
  [Parameter(Mandatory = $true)]
  [string]$OrbbecSdkRoot,

  [Parameter(Mandatory = $true)]
  [string]$FoxgloveSdkRoot,

  [string]$VcpkgToolchain = "C:/vcpkg/scripts/buildsystems/vcpkg.cmake",
  [string]$BuildType = "Release",
  [string]$ConfigPath = "",
  [switch]$BuildOnly
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $ScriptDir "build"

if ([string]::IsNullOrWhiteSpace($ConfigPath)) {
  $ConfigPath = Join-Path $ScriptDir "config/camera_config.ini"
}

Write-Host "Configuring project..."
cmake -S $ScriptDir -B $BuildDir `
  -DCMAKE_TOOLCHAIN_FILE=$VcpkgToolchain `
  -DORBBEC_SDK_ROOT=$OrbbecSdkRoot `
  -DFOXGLOVE_SDK_ROOT=$FoxgloveSdkRoot

Write-Host "Building project..."
cmake --build $BuildDir --config $BuildType

if ($BuildOnly) {
  Write-Host "Build completed (BuildOnly=true)."
  exit 0
}

$ExePath = Join-Path $BuildDir "$BuildType/orbbec_foxglove_bridge.exe"
if (!(Test-Path $ExePath)) {
  $ExePath = Join-Path $BuildDir "orbbec_foxglove_bridge.exe"
}

if (!(Test-Path $ExePath)) {
  throw "Executable not found after build: $ExePath"
}

if (!(Test-Path $ConfigPath)) {
  throw "Config file not found: $ConfigPath"
}

Write-Host "Starting bridge..."
Write-Host "Using config: $ConfigPath"
& $ExePath --config $ConfigPath
