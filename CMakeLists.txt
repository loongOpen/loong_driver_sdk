cmake_minimum_required(VERSION 3.10)
project(driver_test VERSION 3.9.0 LANGUAGES CXX)
option(ARM64 "aarch64" OFF)
if(ARM64)
    set(march aarch64)
    set(arch a64)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../../third_party/lib_lin_a64")
else(ARM64)
    set(march x86_64)
    set(arch x64)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../../third_party/lib_lin_x64")
endif(ARM64)
set(CMAKE_CXX_COMPILER ${march}-linux-gnu-g++)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_INSTALL_PREFIX ..)
configure_file(version.h.in version.h)
link_directories(
    ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/lib;
    ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/lib;
    ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/lib
)
add_library(loong_driver_sdk_${arch} SHARED common.cpp config_xml.cpp rs232.cpp rs485.cpp can.cpp ecat.cpp loong_driver_sdk.cpp)
set_target_properties(loong_driver_sdk_${arch} PROPERTIES NO_SONAME ON)
target_include_directories(loong_driver_sdk_${arch} PUBLIC
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/include
    ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/include
    ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/include
)
target_link_libraries(loong_driver_sdk_${arch} PUBLIC ethercat modbus tinyxml2 pthread)
add_executable(driver_test_${arch} driver_test.cpp)
target_link_libraries(driver_test_${arch} PUBLIC loong_driver_sdk_${arch})
install(TARGETS loong_driver_sdk_${arch} driver_test_${arch}
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
)