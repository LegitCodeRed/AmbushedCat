#!/bin/bash
# Setup script for UsbSync dependencies

echo "Initializing Link submodules (ASIO standalone)..."
cd dep/link
git submodule update --init --recursive
cd ../..

echo ""
echo "Dependencies initialized successfully!"
echo ""
echo "You can now build with:"
echo "  make clean"
echo "  make -j4"
echo ""
echo "Or on Windows with MSYS2:"
echo "  mingw32-make clean"
echo "  mingw32-make -j4"
