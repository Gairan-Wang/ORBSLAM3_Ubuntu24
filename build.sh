echo "Building g2o..."
cd /Thirdparty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
cd ../../../

echo "Uncompressing vocabulary..."
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..


echo "Building ORB_SLAM3 ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
