cmake_minimum_required(VERSION 3.10)
project(driver_test VERSION 3.5.0 LANGUAGES CXX)
option(ARM64 "aarch64" OFF)
if(ARM64)
    set(march aarch64)
else(ARM64)
    set(march x86_64)
endif(ARM64)
set(CMAKE_CXX_COMPILER ${march}-linux-gnu-g++)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
configure_file(version.h.in version.h)
add_library(ethercat STATIC IMPORTED)
set_target_properties(ethercat PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/ethercat/lib/libethercat_${march}.a)
add_library(modbus STATIC IMPORTED)
set_target_properties(modbus PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/modbus/lib/libmodbus_${march}.a)
add_library(tinyxml2 STATIC IMPORTED)
set_target_properties(tinyxml2 PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/tinyxml2/lib/libtinyxml2_${march}.a)
add_library(loong_driver_sdk_${march} SHARED common.cpp config_xml.cpp ecat.cpp serial.cpp loong_driver_sdk.cpp rs485.cpp)
set_target_properties(loong_driver_sdk_${march} PROPERTIES NO_SONAME ON)
target_include_directories(loong_driver_sdk_${march} PUBLIC
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/ethercat/include
    ${PROJECT_SOURCE_DIR}/modbus/include
    ${PROJECT_SOURCE_DIR}/tinyxml2/include
)
target_link_libraries(loong_driver_sdk_${march} PUBLIC ethercat modbus tinyxml2 pthread)
add_executable(driver_test_${march} driver_test.cpp)
target_link_libraries(driver_test_${march} PUBLIC loong_driver_sdk_${march})