:: Reset errorlevel status so we are not inheriting this state from the calling process:
:: @call :CLEAN_EXIT
@echo off

if /I not "%~1"=="__inner__" (
  start "PhysX project generation" cmd /k ""%~f0" __inner__ %*"
  exit /b 0
)

shift /1
setlocal enabledelayedexpansion

set "PHYSX_ROOT_DIR=%~dp0"

:: Convert backslashes to forward slashes
set "PHYSX_ROOT_DIR=%PHYSX_ROOT_DIR:\=/%"

set "PHYSX_PRESET=%1"
if "%PHYSX_PRESET%"=="" (
  set "PHYSX_PRESET=vc18win64"
)

set "PACKMAN_CMD=%PHYSX_ROOT_DIR%buildtools\packman\packman"

call "%PACKMAN_CMD%" init
if errorlevel 1 (
  set "FINAL_ERROR=!ERRORLEVEL!"
  echo.
  echo ============================================================
  echo Packman init failed with error code !FINAL_ERROR!.
  echo ============================================================
  goto FINAL_PAUSE
)

echo Running packman in preparation for cmake ...

set "PACKMAN_PLATFORM=%PHYSX_PRESET%"
set "PACKMAN_PLATFORM=%PACKMAN_PLATFORM:.user=%"

call "%PACKMAN_CMD%" pull "%PHYSX_ROOT_DIR%dependencies.xml" --platform %PACKMAN_PLATFORM%
if errorlevel 1 (
  set "FINAL_ERROR=!ERRORLEVEL!"
  echo.
  echo ============================================================
  echo Packman pull failed with error code !FINAL_ERROR!.
  echo ============================================================
  goto FINAL_PAUSE
)

if not defined PM_freeglut_PATH (
  if exist "D:/packman-repo/chk/freeglut-windows/3.4_1.1" (
    set "PM_freeglut_PATH=D:/packman-repo/chk/freeglut-windows/3.4_1.1"
  )
)

echo Using freeglut:
echo %PM_freeglut_PATH%

set "VSWHERE_EXE="
if defined PM_vswhere_PATH (
  if exist "%PM_vswhere_PATH%\VsWhere.exe" (
    set "VSWHERE_EXE=%PM_vswhere_PATH%\VsWhere.exe"
  )
)

if not defined VSWHERE_EXE (
  if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" (
    set "VSWHERE_EXE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
  )
)

if not defined VSWHERE_EXE (
  set "FINAL_ERROR=1"
  echo.
  echo ============================================================
  echo vswhere.exe was not found.
  echo ============================================================
  goto FINAL_PAUSE
)

for /f "usebackq delims=" %%i in (`"%VSWHERE_EXE%" -products * -latest -property installationPath`) do (
  set "VS_INSTALL_DIR=%%i"
)

if not defined VS_INSTALL_DIR (
  set "FINAL_ERROR=1"
  echo.
  echo ============================================================
  echo No Visual Studio installation was found.
  echo ============================================================
  goto FINAL_PAUSE
)

echo Using Visual Studio instance:
echo   %VS_INSTALL_DIR%

set "CMAKE_GENERATOR=Visual Studio 18 2026"
set "CMAKE_GENERATOR_PLATFORM=x64"
set "CMAKE_GENERATOR_INSTANCE=%VS_INSTALL_DIR%"

set "VCTOOLS_FILE=%VS_INSTALL_DIR%\VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt"

if exist "%VCTOOLS_FILE%" (
  set /p VCTOOLS_VERSION=<"%VCTOOLS_FILE%"
  set "VS180CLPATH=%VS_INSTALL_DIR%\VC\Tools\MSVC\!VCTOOLS_VERSION!\bin\HostX64\x64\cl.exe"
)

if defined VS180CLPATH (
  echo Using compiler:
  echo   %VS180CLPATH%
)

set "PATH=C:\Program Files\CMake\bin;%PATH%"

echo.
cmake --version
echo.

echo Running generate script:
echo   "%PM_PYTHON%" "%PHYSX_ROOT_DIR%buildtools/cmake_generate_projects.py" %PHYSX_PRESET%
echo.

call "%PM_PYTHON%" "%PHYSX_ROOT_DIR%buildtools/cmake_generate_projects.py" %PHYSX_PRESET%

set "GENERATE_ERROR=%ERRORLEVEL%"

echo.
echo ============================================================
echo PhysX project generation returned error code %GENERATE_ERROR%.
echo ============================================================
echo.

if not exist "%PHYSX_ROOT_DIR%compiler/%PHYSX_PRESET%/PhysXSDK.slnx" (
  set "FINAL_ERROR=1"
  echo ============================================================
  echo PhysXSDK.sln was not generated.
  echo Expected:
  echo   %PHYSX_ROOT_DIR%compiler/%PHYSX_PRESET%/PhysXSDK.sln
  echo.
  echo CMake may have failed even if cmake_generate_projects.py returned 0.
  echo ============================================================
  goto FINAL_PAUSE
)

if not "%GENERATE_ERROR%"=="0" (
  set "FINAL_ERROR=%GENERATE_ERROR%"
  goto FINAL_PAUSE
)

set "FINAL_ERROR=0"

echo ============================================================
echo PhysX project generation completed successfully.
echo ============================================================

:FINAL_PAUSE
echo.
echo Final error code: %FINAL_ERROR%
echo.
pause
exit /b %FINAL_ERROR%
