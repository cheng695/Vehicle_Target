[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug",
    [string]$Device = "STM32F427II",
    [ValidateSet("SWD", "JTAG")]
    [string]$Interface = "SWD",
    [ValidateRange(100, 50000)]
    [int]$Speed = 4000
)

$ErrorActionPreference = "Stop"
$projectRoot = $PSScriptRoot
$firmware = Join-Path $projectRoot "build/$Config/Vehicle_Target.elf"

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "Firmware not found: $firmware. Run .\build.ps1 -Config $Config first."
}

$jlink = Get-Command "JLink.exe" -ErrorAction SilentlyContinue
if ($jlink) {
    $jlinkExe = $jlink.Source
}
else {
    $jlinkExe = Get-ChildItem "C:\Program Files\SEGGER", "C:\Program Files (x86)\SEGGER" `
        -Filter "JLink.exe" -File -Recurse -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}

if (-not $jlinkExe) {
    throw "JLink.exe was not found. Install SEGGER J-Link Software or add it to PATH."
}

$commandFile = Join-Path ([System.IO.Path]::GetTempPath()) ("vehicle-target-{0}.jlink" -f [guid]::NewGuid())
$commands = @(
    "device $Device"
    "if $Interface"
    "speed $Speed"
    "connect"
    "r"
    "h"
    "loadfile $($firmware.Replace('\', '/'))"
    "r"
    "g"
    "exit"
)

try {
    Set-Content -LiteralPath $commandFile -Value $commands -Encoding ASCII
    & $jlinkExe -NoGui 1 -ExitOnError 1 -CommandFile $commandFile
    if ($LASTEXITCODE -ne 0) { throw "J-Link download failed with exit code $LASTEXITCODE." }
    Write-Host "Download finished: $firmware" -ForegroundColor Green
}
finally {
    Remove-Item -LiteralPath $commandFile -Force -ErrorAction SilentlyContinue
}
