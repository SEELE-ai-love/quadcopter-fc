param(
    [ValidateSet("Debug", "Release")][string]$Configuration = "Debug",
    [switch]$ConfigureOnly
)
$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$bundleRoot = $env:CUBE_BUNDLE_PATH
if (-not $bundleRoot) { $bundleRoot = [Environment]::GetEnvironmentVariable("CUBE_BUNDLE_PATH", "User") }
function Find-BundleTool([string]$Package, [string]$Executable) {
    if ($bundleRoot) {
        $packageRoot = Join-Path $bundleRoot $Package
        if (Test-Path -LiteralPath $packageRoot) {
            $versions = Get-ChildItem -LiteralPath $packageRoot -Directory | Sort-Object { [version](($_.Name -split '\+')[0]) } -Descending
            foreach ($version in $versions) {
                $candidate = Join-Path $version.FullName "bin\$Executable"
                if (Test-Path -LiteralPath $candidate) { return $candidate }
            }
        }
    }
    $command = Get-Command $Executable -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    throw "Cannot find $Executable. Set CUBE_BUNDLE_PATH or add its bin directory to PATH."
}
$cmakeExe = Find-BundleTool "cmake" "cmake.exe"
$ninjaExe = Find-BundleTool "ninja" "ninja.exe"
if ($env:ARM_GCC_ROOT) { $gccExe = Join-Path $env:ARM_GCC_ROOT "bin\arm-none-eabi-gcc.exe" }
else { $gccExe = Find-BundleTool "gnu-tools-for-stm32" "arm-none-eabi-gcc.exe" }
if (-not (Test-Path -LiteralPath $gccExe)) { throw "Compiler not found: $gccExe" }
$gccRoot = Split-Path -Parent (Split-Path -Parent $gccExe)
Push-Location -LiteralPath $projectRoot
try {
    & $cmakeExe --preset $Configuration "-DARM_GCC_ROOT=$gccRoot" "-DCMAKE_MAKE_PROGRAM=$ninjaExe"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    if (-not $ConfigureOnly) {
        & $cmakeExe --build --preset $Configuration --parallel 4
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
}
finally { Pop-Location }
