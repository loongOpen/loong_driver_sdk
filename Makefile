AR_X86_64 = x86_64-linux-gnu-ar
CXX_X86_64 = x86_64-linux-gnu-g++
AR_AARCH64 = aarch64-linux-gnu-ar
CXX_AARCH64 = aarch64-linux-gnu-g++

all:
	@echo "make x86_64 or aarch64"

x86_64: exe_x86_64 dll_x86_64 lib_x86_64

exe_x86_64: driver_test_x86_64.o dll_x86_64
	$(CXX_X86_64) -o driver_test_x86_64 driver_test_x86_64.o -L. -lloong_driver_sdk_x86_64 -lpthread

dll_x86_64: loong_driver_sdk_x86_64.o ecat_x86_64.o rs485_x86_64.o common_x86_64.o serial_x86_64.o config_xml_x86_64.o
	$(CXX_X86_64) -shared -o libloong_driver_sdk_x86_64.so loong_driver_sdk_x86_64.o ecat_x86_64.o rs485_x86_64.o common_x86_64.o serial_x86_64.o config_xml_x86_64.o ethercat/lib/libethercat_x86_64.a modbus/lib/libmodbus_x86_64.a tinyxml2/lib/libtinyxml2_x86_64.a

lib_x86_64: loong_driver_sdk_x86_64.o ecat_x86_64.o rs485_x86_64.o common_x86_64.o serial_x86_64.o config_xml_x86_64.o
	$(AR_X86_64) rcs libloong_driver_sdk_x86_64.a loong_driver_sdk_x86_64.o ecat_x86_64.o rs485_x86_64.o common_x86_64.o serial_x86_64.o config_xml_x86_64.o

config_xml_x86_64.o: config_xml.cpp config_xml.h
	$(CXX_X86_64) -fPIC -o config_xml_x86_64.o -Itinyxml2/include -c config_xml.cpp

serial_x86_64.o: serial.cpp serial.h loong_driver_sdk.h
	$(CXX_X86_64) -fPIC -o serial_x86_64.o -Iethercat/include -c serial.cpp

common_x86_64.o: common.cpp common.h config_xml.h
	$(CXX_X86_64) -fPIC -o common_x86_64.o -Iethercat/include -Itinyxml2/include -c common.cpp

rs485_x86_64.o: rs485.cpp rs485.h
	$(CXX_X86_64) -fPIC -o rs485_x86_64.o -Iethercat/include -Imodbus/include -c rs485.cpp

ecat_x86_64.o: ecat.cpp ecat.h common.h ptr_que.h config_xml.h
	$(CXX_X86_64) -fPIC -o ecat_x86_64.o -Iethercat/include -Itinyxml2/include -c ecat.cpp

loong_driver_sdk_x86_64.o: loong_driver_sdk.cpp ecat.h rs485.h common.h ptr_que.h serial.h config_xml.h loong_driver_sdk.h version.h
	$(CXX_X86_64) -fPIC -o loong_driver_sdk_x86_64.o -Iethercat/include -Imodbus/include -Itinyxml2/include -c loong_driver_sdk.cpp

driver_test_x86_64.o: driver_test.cpp loong_driver_sdk.h
	$(CXX_X86_64) -o driver_test_x86_64.o -c driver_test.cpp

aarch64: exe_aarch64 dll_aarch64 lib_aarch64

exe_aarch64: driver_test_aarch64.o dll_aarch64
	$(CXX_AARCH64) -o driver_test_aarch64 driver_test_aarch64.o -L. -lloong_driver_sdk_aarch64 -lpthread

dll_aarch64: loong_driver_sdk_aarch64.o ecat_aarch64.o rs485_aarch64.o common_aarch64.o serial_aarch64.o config_xml_aarch64.o
	$(CXX_AARCH64) -shared -o libloong_driver_sdk_aarch64.so loong_driver_sdk_aarch64.o ecat_aarch64.o rs485_aarch64.o common_aarch64.o serial_aarch64.o config_xml_aarch64.o ethercat/lib/libethercat_aarch64.a modbus/lib/libmodbus_aarch64.a tinyxml2/lib/libtinyxml2_aarch64.a

lib_aarch64: loong_driver_sdk_aarch64.o ecat_aarch64.o rs485_aarch64.o common_aarch64.o serial_aarch64.o config_xml_aarch64.o
	$(AR_AARCH64) rcs libloong_driver_sdk_aarch64.a loong_driver_sdk_aarch64.o ecat_aarch64.o rs485_aarch64.o common_aarch64.o serial_aarch64.o config_xml_aarch64.o

config_xml_aarch64.o: config_xml.cpp config_xml.h
	$(CXX_AARCH64) -fPIC -o config_xml_aarch64.o -Itinyxml2/include -c config_xml.cpp

serial_aarch64.o: serial.cpp serial.h loong_driver_sdk.h
	$(CXX_AARCH64) -fPIC -o serial_aarch64.o -Iethercat/include -c serial.cpp

common_aarch64.o: common.cpp common.h config_xml.h
	$(CXX_AARCH64) -fPIC -o common_aarch64.o -Iethercat/include -Itinyxml2/include -c common.cpp

rs485_aarch64.o: rs485.cpp rs485.h
	$(CXX_AARCH64) -fPIC -o rs485_aarch64.o -Iethercat/include -Imodbus/include -c rs485.cpp

ecat_aarch64.o: ecat.cpp ecat.h common.h ptr_que.h config_xml.h
	$(CXX_AARCH64) -fPIC -o ecat_aarch64.o -Iethercat/include -Itinyxml2/include -c ecat.cpp

loong_driver_sdk_aarch64.o: loong_driver_sdk.cpp ecat.h rs485.h common.h ptr_que.h serial.h config_xml.h loong_driver_sdk.h version.h
	$(CXX_AARCH64) -fPIC -o loong_driver_sdk_aarch64.o -Iethercat/include -Imodbus/include -Itinyxml2/include -c loong_driver_sdk.cpp

driver_test_aarch64.o: driver_test.cpp loong_driver_sdk.h
	$(CXX_AARCH64) -o driver_test_aarch64.o -c driver_test.cpp

clean:
	rm -rf driver_test_x86_64 driver_test_aarch64 *.o *.so *.a *.bak