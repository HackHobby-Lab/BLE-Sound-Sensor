# Build + flash helper for the ICS43434 mic test on nRF52833 DK.
# Usage:
#   .\build.ps1            # build only
#   .\build.ps1 flash      # build then flash
#   .\build.ps1 clean      # wipe build dir

param([string]$action = "build")

$ErrorActionPreference = "Stop"

$Toolchain = "C:\ncs\toolchains\fd21892d0f"
$NcsRoot   = "C:\ncs\v3.2.4"
$AppDir    = $PSScriptRoot
$BuildDir  = Join-Path $AppDir "build"
$Board     = "nrf52833dk/nrf52833"

# Activate the NCS toolchain environment for this process.
$env:PATH = "$Toolchain\opt\bin\Scripts;$Toolchain\opt\bin;$Toolchain\nrfutil\bin;$Toolchain\bin;$Toolchain\mingw64\bin;$Toolchain\opt\zephyr-sdk\arm-zephyr-eabi\bin;$env:PATH"
$env:ZEPHYR_BASE             = "$NcsRoot\zephyr"
$env:ZEPHYR_TOOLCHAIN_VARIANT = "zephyr"
$env:ZEPHYR_SDK_INSTALL_DIR  = "$Toolchain\opt\zephyr-sdk"
$env:PYTHONPATH              = "$Toolchain\opt\bin;$Toolchain\opt\bin\Lib;$Toolchain\opt\bin\Lib\site-packages"

switch ($action) {
    "clean" {
        if (Test-Path $BuildDir) { Remove-Item -Recurse -Force $BuildDir }
        Write-Host "Cleaned $BuildDir"
        return
    }
    "build" {
        west build -b $Board $AppDir --build-dir $BuildDir
    }
    "flash" {
        west build -b $Board $AppDir --build-dir $BuildDir
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        west flash --build-dir $BuildDir
    }
    default {
        Write-Host "Unknown action: $action (use build | flash | clean)"
        exit 1
    }
}
