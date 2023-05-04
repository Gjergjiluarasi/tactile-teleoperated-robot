#! /bin/sh
cmake -S . -B ./build
cd ./build
make
cd ..
echo "Build files generated under build folder"