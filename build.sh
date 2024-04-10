echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus
echo "Configuring and building Thirdparty/Sophus ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../line_descriptor
echo "Configuring and building Thirdparty/Line Descriptor ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../ELSED
echo "Configuring and building Thirdparty/ELSED ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

buildArpackLib="OFF"
if [ "$buildArpackLib" = "ON" ]; then
 echo "Configuring and building Thirdparty/Arpackpp ..."
 cd ../../arpackpp
 rm -rf external
 ./install-openblas.sh
 ./install-arpack-ng.sh
 ./install-superlu.sh
fi

cd ../../../
echo "Uncompress vocabulary ..."
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building UL-SLAM ..."

mkdir build
cd build
cmake .. -DUSE_LINE_PLUKER=true -DCMAKE_BUILD_TYPE=Debug
sudo make -j 20
