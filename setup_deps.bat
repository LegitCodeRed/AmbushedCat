@echo off
REM Setup script for UsbSync dependencies (Windows)

echo Initializing Link submodules (ASIO standalone)...
cd dep\link
git submodule update --init --recursive
cd ..\..

echo.
echo Dependencies initialized successfully!
echo.
echo You can now build with:
echo   mingw32-make clean
echo   mingw32-make -j4
echo.
echo Or if using MSYS2 bash:
echo   make clean
echo   make -j4
