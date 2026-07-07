# loong_driver_sdk
cd build
cmake ..
make install
cd ..

cd build
cmake -DRISCV64=ON ..
make install
cd ..

cd build
cmake -DAARCH64=ON ..
make install
cd ..

cd build
cmake -DAARCH64=ON -DNIIC=ON ..
make install
cd ..

cd build
cmake -DAARCH64=ON -DENPHT=ON ..
make install
cd ..