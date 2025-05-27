# openloong_driver_sdk
cd build
cmake -DARM64=OFF ..
make
make install
cmake -DARM64=ON ..
make
make install