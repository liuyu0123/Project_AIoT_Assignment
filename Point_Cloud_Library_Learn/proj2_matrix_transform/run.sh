mkdir build
cd build
cmake ..
make
cp ../../utils/cube.ply .
./matrix_transform cube.ply