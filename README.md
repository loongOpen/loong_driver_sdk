# openloong_driver_sdk
make x86_64
make aarch64

cd build
cmake -DARM64=OFF ..
make
cmake -DARM64=ON ..
make