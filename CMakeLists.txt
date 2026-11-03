cmake_minimum_required(VERSION 3.10)
option(AARCH64 "aarch64" OFF)
option(RISCV64 "riscv64" OFF)
option(LOONGARCH64 "loongarch64" OFF)
option(XENOMAI "xenomai" OFF)
option(NIIC "necro" OFF)
option(ENPHT "enpht" OFF)
option(GD32 "gd32" OFF)
option(IPROUTE2 "iproute2" ON)
option(TEST "test" ON)
if(AARCH64)
    set(march aarch64)
    set(arch a64)
elseif(RISCV64)
    set(march riscv64)
    set(arch r64)
elseif(LOONGARCH64)
    set(march loongarch64)
    set(arch l64)
else(AARCH64)
    set(march x86_64)
    set(arch x64)
endif(AARCH64)
if(XENOMAI)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../../third_party/lib_lin_${arch}:/usr/xenomai/lib")
else(XENOMAI)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../../third_party/lib_lin_${arch}")
endif(XENOMAI)
set(CMAKE_C_COMPILER ${march}-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${march}-linux-gnu-g++)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(driver_test VERSION 4.4.7 LANGUAGES C CXX)
set(CMAKE_INSTALL_PREFIX ..)
configure_file(version.h.in version.h)
if(NIIC)
    add_definitions(-DNIIC)
    set(Necro_DIR ${PROJECT_SOURCE_DIR}/../loong_third_party/necro/lib/cmake/necro)
    find_package(Necro REQUIRED)
    add_library(necro SHARED IMPORTED)
    set_target_properties(necro PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../loong_third_party/necro/ld-linux-aarch64.so.1)
    add_library(necroc SHARED IMPORTED)
    set_target_properties(necroc PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../loong_third_party/necro/libc.so.6)
    add_library(necrocxx SHARED IMPORTED)
    set_target_properties(necrocxx PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../loong_third_party/necro/libstdc++.so.6)
elseif(XENOMAI)
    add_definitions(-DXENOMAI)
    add_library(bootstrap STATIC IMPORTED)
    set_target_properties(bootstrap PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../loong_third_party/xenomai/lib/${march}-linux-gnu/xenomai/bootstrap.o)
    add_library(bootstrap_pic STATIC IMPORTED)
    set_target_properties(bootstrap_pic PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../loong_third_party/xenomai/lib/${march}-linux-gnu/xenomai/bootstrap-pic.o)
endif(NIIC)
if(ENPHT)
    add_definitions(-DENPHT)
    add_library(hbcanhal_${march} SHARED hobot_can_hal_enpht.c)
elseif(GD32)
    add_definitions(-DGD32)
    add_library(hbcanhal_${march} SHARED hobot_can_hal_gd32.c)
else(ENPHT)
    add_library(hbcanhal_${march} SHARED hobot_can_hal_null.c)
endif(ENPHT)
set_target_properties(hbcanhal_${march} PROPERTIES NO_SONAME ON)
if(ENPHT)
    target_include_directories(hbcanhal_${march} PRIVATE ${PROJECT_SOURCE_DIR}/../loong_third_party/enpht/include)
    target_link_directories(hbcanhal_${march} PRIVATE ${PROJECT_SOURCE_DIR})
    target_link_libraries(hbcanhal_${march} PRIVATE enpht-can)
endif(ENPHT)
if(IPROUTE2)
    add_definitions(-DIPROUTE2)
endif(IPROUTE2)
if(NIIC)
    add_library(loong_driver_sdk_${arch} SHARED common.cpp config_xml.cpp rs232.cpp rs485.cpp canbase.cpp can.cpp canemu.cpp ecat.cxx loong_driver_sdk.cpp)
else(NIIC)
    add_library(loong_driver_sdk_${arch} SHARED common.cpp config_xml.cpp rs232.cpp rs485.cpp canbase.cpp can.cpp canemu.cpp ecat.cpp loong_driver_sdk.cpp)
endif(NIIC)
set_target_properties(loong_driver_sdk_${arch} PROPERTIES NO_SONAME ON)
if(NIIC)
    target_include_directories(loong_driver_sdk_${arch} PRIVATE
        ${PROJECT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat-cpp/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/eigen/include
    )
    target_link_directories(loong_driver_sdk_${arch} PRIVATE
        ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat-cpp/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/lib
    )
    target_link_libraries(loong_driver_sdk_${arch} PRIVATE tinyxml2_${march} netlink_${march} util_${march} modbus_${march} ethercat-cpp Necro::qiuniu pthread hbcanhal_${march})
elseif(XENOMAI)
    target_include_directories(loong_driver_sdk_${arch} PRIVATE
        ${PROJECT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}/../loong_third_party/xenomai/include/cobalt
        ${PROJECT_SOURCE_DIR}/../loong_third_party/xenomai/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/eigen/include
    )
    target_link_directories(loong_driver_sdk_${arch} PRIVATE
        ${PROJECT_SOURCE_DIR}/../loong_third_party/xenomai/lib/${march}-linux-gnu
        ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/lib
    )
    target_link_libraries(loong_driver_sdk_${arch} PRIVATE tinyxml2_${march} netlink_${march} util_${march} modbus_${march} ethercat_${march} bootstrap_pic alchemy pthread hbcanhal_${march})
else(NIIC)
    target_include_directories(loong_driver_sdk_${arch} PRIVATE
        ${PROJECT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/include
        ${PROJECT_SOURCE_DIR}/../loong_third_party/eigen/include
    )
    target_link_directories(loong_driver_sdk_${arch} PRIVATE
        ${PROJECT_SOURCE_DIR}/../loong_third_party/ethercat/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/modbus/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/netlink/lib
        ${PROJECT_SOURCE_DIR}/../loong_third_party/tinyxml2/lib
    )
    target_link_libraries(loong_driver_sdk_${arch} PRIVATE tinyxml2_${march} netlink_${march} util_${march} modbus_${march} ethercat_${march} pthread hbcanhal_${march})
endif(NIIC)
if(TEST)
    add_executable(driver_test_${march} driver_test.cpp)
    if(LOONGARCH64)
        target_include_directories(driver_test_${march} PRIVATE ${PROJECT_SOURCE_DIR}/../loong_third_party/ncurses/include/loongarch64-linux-gnu)
    else(LOONGARCH64)
        target_include_directories(driver_test_${march} PRIVATE ${PROJECT_SOURCE_DIR}/../loong_third_party/ncurses/include)
    endif(LOONGARCH64)
    if(XENOMAI)
        target_link_directories(driver_test_${march} PRIVATE
            ${PROJECT_SOURCE_DIR}/../loong_third_party/xenomai/lib/${march}-linux-gnu
            ${PROJECT_SOURCE_DIR}/../loong_third_party/ncurses/lib/${march}-linux-gnu
        )
    else(XENOMAI)
        target_link_directories(driver_test_${march} PRIVATE ${PROJECT_SOURCE_DIR}/../loong_third_party/ncurses/lib/${march}-linux-gnu)
    endif(XENOMAI)
    if(NIIC)
        target_link_libraries(driver_test_${march} PRIVATE loong_driver_sdk_${arch} ncurses tinfo necrocxx necroc necro)
    elseif(XENOMAI)
        target_link_libraries(driver_test_${march} PRIVATE loong_driver_sdk_${arch} ncurses tinfo bootstrap cobalt modechk)
        target_link_options(driver_test_${march} PRIVATE -Wl,--wrap=main,--dynamic-list=${PROJECT_SOURCE_DIR}/../loong_third_party/xenomai/lib/${march}-linux-gnu/dynlist.ld)
    else(NIIC)
        target_link_libraries(driver_test_${march} PRIVATE loong_driver_sdk_${arch} ncurses tinfo)
    endif(NIIC)
    install(TARGETS loong_driver_sdk_${arch} hbcanhal_${march} driver_test_${march}
        RUNTIME DESTINATION .
        LIBRARY DESTINATION .
    )
else(TEST)
    install(TARGETS loong_driver_sdk_${march} hbcanhal_${march}
        RUNTIME DESTINATION .
        LIBRARY DESTINATION .
    )
endif(TEST)