#
# SAINT.OS Build Script for Windows (PowerShell)
#

param(
    [switch]$Clean,
    [switch]$Release,
    [switch]$Debug,
    [switch]$Test,
    [int]$Jobs = 0,
    [switch]$InstallDeps,
    [switch]$Dev,
    [switch]$Help
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$PackageDir = Split-Path -Parent $ScriptDir
$WorkspaceDir = Split-Path -Parent $PackageDir

function Write-Info {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor Green
}

function Write-Warn {
    param([string]$Message)
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Write-Error {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

function Find-ROS2Setup {
    $ros2Paths = @(
        "C:\opt\ros\jazzy\setup.bat",
        "C:\opt\ros\iron\setup.bat",
        "C:\opt\ros\humble\setup.bat",
        "C:\dev\ros2_humble\setup.bat",
        "C:\dev\ros2\setup.bat"
    )

    foreach ($path in $ros2Paths) {
        if (Test-Path $path) {
            return $path
        }
    }

    # Check ROS_DISTRO environment variable
    $rosDistro = $env:ROS_DISTRO
    if ($rosDistro) {
        $path = "C:\opt\ros\$rosDistro\setup.bat"
        if (Test-Path $path) {
            return $path
        }
    }

    return $null
}

function Test-Dependencies {
    Write-Info "Checking dependencies..."

    # Check Python
    try {
        $pythonVersion = python --version 2>&1
        Write-Info "  Python: $pythonVersion"
    } catch {
        Write-Error "Python not found"
        exit 1
    }

    # Check ROS2
    $ros2Setup = Find-ROS2Setup
    if (-not $ros2Setup) {
        Write-Error "ROS2 installation not found"
        Write-Error "Please install ROS2 Humble or later"
        exit 1
    }
    Write-Info "  ROS2 setup: $ros2Setup"

    # Source ROS2 and check colcon
    $colconPath = where.exe colcon 2>$null
    if (-not $colconPath) {
        Write-Error "colcon not found"
        exit 1
    }
    Write-Info "  colcon: found"

    return $ros2Setup
}

function Invoke-Clean {
    Write-Info "Cleaning build artifacts..."
    Push-Location $WorkspaceDir

    $dirsToRemove = @("build", "install", "log")
    foreach ($dir in $dirsToRemove) {
        if (Test-Path $dir) {
            Write-Info "  Removing $dir"
            Remove-Item -Recurse -Force $dir
        }
    }

    Pop-Location
    Write-Info "Clean complete"
}

function Invoke-Build {
    param(
        [string]$ROS2Setup,
        [string]$BuildType = "Release",
        [int]$ParallelJobs = 0
    )

    Write-Info "Building SAINT.OS ($BuildType)..."
    Push-Location $WorkspaceDir

    $colconArgs = @(
        "build",
        "--symlink-install",
        "--cmake-args", "-DCMAKE_BUILD_TYPE=$BuildType"
    )

    if ($ParallelJobs -gt 0) {
        $colconArgs += @("--parallel-workers", $ParallelJobs.ToString())
    }

    # Create a batch file to source ROS2 and run colcon
    $batchContent = @"
@echo off
call "$ROS2Setup"
colcon $($colconArgs -join ' ')
"@
    $batchFile = Join-Path $env:TEMP "saint_build.bat"
    Set-Content -Path $batchFile -Value $batchContent

    & cmd /c $batchFile
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Build failed"
        Pop-Location
        exit 1
    }

    Pop-Location
    Write-Info "Build complete"
}

function Invoke-Tests {
    param([string]$ROS2Setup)

    Write-Info "Running tests..."
    Push-Location $WorkspaceDir

    $batchContent = @"
@echo off
call "$ROS2Setup"
colcon test --pytest-args -v
colcon test-result --verbose
"@
    $batchFile = Join-Path $env:TEMP "saint_test.bat"
    Set-Content -Path $batchFile -Value $batchContent

    & cmd /c $batchFile

    Pop-Location
    Write-Info "Tests complete"
}

function Install-PythonDeps {
    param([switch]$DevMode)

    Write-Info "Installing Python dependencies..."
    Push-Location $PackageDir

    if ($DevMode) {
        pip install -e ".[dev]"
    } else {
        pip install -e .
    }

    Pop-Location
    Write-Info "Python dependencies installed"
}

function Show-Usage {
    Write-Host @"
SAINT.OS Build Script for Windows

Usage: .\build.ps1 [OPTIONS]

Options:
  -Clean         Clean build artifacts before building
  -Release       Build in Release mode (default)
  -Debug         Build in Debug mode
  -Test          Run tests after building
  -Jobs N        Number of parallel jobs
  -InstallDeps   Install Python dependencies
  -Dev           Install development dependencies
  -Help          Show this help message
"@
}

# Main
function Main {
    if ($Help) {
        Show-Usage
        exit 0
    }

    Write-Host "========================================"
    Write-Host "SAINT.OS Build Script"
    Write-Host "========================================"
    Write-Host "Platform: Windows"
    Write-Host "Workspace: $WorkspaceDir"
    Write-Host ""

    # Check dependencies
    $ros2Setup = Test-Dependencies
    Write-Host ""

    # Install Python deps if requested
    if ($InstallDeps) {
        Install-PythonDeps -DevMode:$Dev
        Write-Host ""
    }

    # Clean if requested
    if ($Clean) {
        Invoke-Clean
        Write-Host ""
    }

    # Determine build type
    $buildType = if ($Debug) { "Debug" } else { "Release" }

    # Build
    Invoke-Build -ROS2Setup $ros2Setup -BuildType $buildType -ParallelJobs $Jobs
    Write-Host ""

    # Test if requested
    if ($Test) {
        Invoke-Tests -ROS2Setup $ros2Setup
        Write-Host ""
    }

    Write-Host "========================================"
    Write-Info "Build successful!"
    Write-Host ""
    Write-Host "To use the built package, source the setup script:"
    Write-Host "  call $WorkspaceDir\install\setup.bat"
    Write-Host ""
}

Main
