:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off
setlocal
set "PHYSX_ROOT_DIR=%~dp0"
set "PHYSX_ROOT_DIR=%PHYSX_ROOT_DIR:\=/%"
set "PHYSX_PRESET=%~1"
if not defined PHYSX_PRESET set "PHYSX_PRESET=vc18win64"
set "PACKMAN_CMD=%PHYSX_ROOT_DIR%buildtools\packman\packman"
call "%PACKMAN_CMD%" init
if errorlevel 1 exit /b %errorlevel%
call "%PACKMAN_CMD%" pull "%PHYSX_ROOT_DIR%dependencies.xml" --platform %PHYSX_PRESET:.user=%
if errorlevel 1 exit /b %errorlevel%

:: Respect explicit CUDA configuration, otherwise select the validated toolkit.
if not defined CUDA_PATH if defined CUDA_PATH_V13_3 set "CUDA_PATH=%CUDA_PATH_V13_3%"
if not defined CUDA_PATH if exist "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v13.3\bin\nvcc.exe" set "CUDA_PATH=%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v13.3"
if not defined PM_CUDA_PATH if defined CUDA_PATH set "PM_CUDA_PATH=%CUDA_PATH%"

:: Packman now declares vc18 dependencies; this also supports an existing cache.
if not defined PM_freeglut_PATH if exist "D:\packman-repo\chk\freeglut-windows\3.4_1.1" set "PM_freeglut_PATH=D:/packman-repo/chk/freeglut-windows/3.4_1.1"
if not defined PM_cmake_PATH if exist "%ProgramFiles%\CMake\bin\cmake.exe" set "PATH=%ProgramFiles%\CMake\bin;%PATH%"

:: CMake selects the Visual Studio instance for the generator in the preset.
call "%PM_PYTHON%" "%PHYSX_ROOT_DIR%buildtools/cmake_generate_projects.py" %PHYSX_PRESET%
exit /b %errorlevel%
