[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug",
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$projectRoot = $PSScriptRoot

foreach ($command in @("cmake", "ninja", "arm-none-eabi-gcc")) {
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) {
        throw "$command was not found. Install it and add it to PATH."
    }
}

Push-Location $projectRoot
try {
    cmake --preset $Config
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed." }

    if ($Clean) {
        cmake --build --preset $Config --target clean
        if ($LASTEXITCODE -ne 0) { throw "Clean failed." }
    }

    cmake --build --preset $Config
    if ($LASTEXITCODE -ne 0) { throw "Build failed." }

    $elf = Join-Path $projectRoot "build/$Config/Vehicle_Target.elf"
    if (-not (Test-Path -LiteralPath $elf)) {
        throw "Build finished, but firmware was not found: $elf"
    }

    Write-Host "Build finished: $elf" -ForegroundColor Green
}
finally {
    Pop-Location
}
