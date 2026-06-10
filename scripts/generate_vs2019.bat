@echo off
REM ===========================================================================
REM Generate a Visual Studio 2019 solution from CMake.
REM
REM   generate_vs2019.bat            -> tests-only solution (no CUDA needed)
REM   generate_vs2019.bat pipeline   -> full CUDA/TensorRT/DXGI pipeline
REM
REM For the pipeline build, set these first:
REM   set TENSORRT_ROOT=C:\TensorRT-10.x
REM   set EIGEN3_INCLUDE_DIR=C:\libs\eigen-3.4.0   (or install via vcpkg)
REM and make sure the CUDA Toolkit VS2019 integration is installed.
REM ===========================================================================
setlocal
cd /d "%~dp0\.."

if /I "%1"=="pipeline" (
    echo [generate] Full pipeline solution -> build\vs2019-pipeline\rtpercept.sln
    cmake --preset vs2019-pipeline
) else (
    echo [generate] Tests-only solution -> build\vs2019\rtpercept.sln
    cmake --preset vs2019
)

if errorlevel 1 (
    echo.
    echo CMake configuration failed. See the message above.
    exit /b 1
)
echo.
echo Done. Open the generated .sln in Visual Studio 2019, or build from the CLI:
echo   cmake --build build\vs2019 --config Release
endlocal
