#! /bin/sh
cmake -S . -B ./build
cd ./build
make -j VERBOSE=1
cd ..
echo "Build files generated under build folder"