cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
