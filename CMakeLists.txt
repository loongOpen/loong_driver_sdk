cmake_minimum_required(VERSION 3.10)
option(AARCH64 "aarch64" OFF)
option(RISCV64 "riscv64" OFF)
option(NIIC "necro" OFF)
option(ENPHT "enpht" OFF)
option(IPROUTE2 "iproute2" OFF)
set(march x86_64)
set(arch x64)
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../../third_party/lib_lin_x64")
if(AARCH64)
    set(march aarch64)
    set(arch a64)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../../third_party/lib_lin_a64")
endif(AARCH64)
if(RISCV64)
    set(march riscv64)
    set(arch r64)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../../third_party/lib_lin_r64")
endif(RISCV64)
set(CMAKE_C_COMPILER ${march}-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${march}-linux-gnu-g++)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
project(driver_test VERSION 4.3.5 LANGUAGES C CXX)
set(CMAKE_INSTALL_PREFIX ..)
configure_file(version.h.in version.h)
if(NIIC)
    add_definitions(-DNIIC)
    set(Necro_DIR ${PROJECT_SOURCE_DIR}/../loong_third_party/necro/lib/cmake/necro)
    find_package(Necro REQUIRED)
endif(NIIC)
if(IPROUTE2)
    add_definitions(-DIPROUTE2)
endif(IPROUTE2)
if(ENPHT)
    add_definitions(-DENPHT)
    add_library(hbcanhal_${march} SHARED hobot_can_hal_enpht.c)
else(ENPHT)
    add_library(hbcanhal_${march} SHARED hobot_can_hal_null.c)
endif(ENPHT)
set_target_properties(hbcanhal_${march} PROPERTIES NO_SONAME ON)
target_include_directories(hbcanhal_${march} PRIVATE
    ${PROJECT_SOURCE_DIR}/../loong_third_party/enpht/include
)
target_link_directories(hbcanhal_${march} PRIVATE
    ${PROJECT_SOURCE_DIR}
)
if(ENPHT)
    target_link_libraries(hbcanhal_${march} PRIVATE enpht-can)
endif(ENPHT)
if(NIIC)
    add_library(loong_driver_sdk_${arch} SHARED common.cpp config_xml.cpp rs232.cpp rs485.cpp canbase.cpp can.cpp canemu.cpp ecat.cxx loong_driver_sdk.cpp)
else(NIIC)
    add_library(loong_driver_sdk_${arch} SHARED common.cpp config_xml.cpp rs232.cpp rs485.cpp canbase.cpp can.cpp canemu.cpp ecat.cpp loong_driver_sdk.cpp)
endif(NIIC)
set_target_properties(loong_driver_sdk_${arch} PROPERTIES NO_SONAME ON)
target_include_directories(loong_driver_sdk_${arch} PRIVATE
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat-cpp/include
    ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/include
    ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/include
    ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/include
    ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/include
)
target_link_directories(loong_driver_sdk_${arch} PRIVATE
    ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat-cpp/lib
    ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/lib
    ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/lib
    ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/lib
    ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/lib
)
if(NIIC)
    target_link_libraries(loong_driver_sdk_${arch} PRIVATE Necro::qiuniu ethercat-cpp modbus_${march} netlink_${march} util_${march} tinyxml2_${march} pthread hbcanhal_${march})
else(NIIC)
    target_link_libraries(loong_driver_sdk_${arch} PRIVATE ethercat_${march} modbus_${march} netlink_${march} util_${march} tinyxml2_${march} pthread hbcanhal_${march})
endif(NIIC)
add_executable(driver_test_${march} driver_test.cpp)
target_link_libraries(driver_test_${march} PRIVATE loong_driver_sdk_${arch})
install(TARGETS loong_driver_sdk_${arch} hbcanhal_${march} driver_test_${march}
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
)