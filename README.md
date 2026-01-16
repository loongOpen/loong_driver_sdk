# loong_driver_sdk
cd build
cmake ..
make
make install

cd build
cmake -DAARCH64=ON ..
make
make install

cd build
cmake -DRISCV64=ON ..
make
make install

cd build
cmake -DAARCH64=ON -DNIIC=ON ..
make
make install