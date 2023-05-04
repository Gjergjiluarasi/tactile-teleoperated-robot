#! /bin/sh
cmake -DCMAKE_BUILD_TYPE=Release -S . -B ./release
cd release
make -j VERBOSE=1
cd ..
echo "Release files generated under release folder"