#! /bin/sh
cmake -DCMAKE_BUILD_TYPE=Debug -S . -B ./debug
cd debug
make -j VERBOSE=1
cd ..
echo "Debug files generated under debug folder"