/* Copyright 2025 人形机器人（上海）有限公司
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Designed and built with love @zhihu by @cjrcl.
 */

#pragma once

#ifndef NIIC
#include <ecrt.h>
#else
#include <ecat/task.hpp>
#endif
#include <string>
#include <atomic>
#include <cmath>
#include <eigen3/Eigen/Geometry>

namespace DriverSDK{
#define NSEC_PER_SEC 1000000000L
#define USEC_PER_SEC 1000000L
#define TIMESPEC2NS(T) (T.tv_sec * NSEC_PER_SEC + T.tv_nsec)
#define TIMEVAL2US(T) (T.tv_sec * USEC_PER_SEC + T.tv_usec)

float const Pi = std::acos(-1);

unsigned int const Table[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

unsigned int crc32(unsigned int crc, unsigned char const* buff, unsigned int const length);
void print(unsigned char const* data, int const length);
unsigned short single2half(float const f);
float half2single(unsigned short const u);
int quadchar2int(unsigned char const* qc);
int quadchar2int_(unsigned char const* qc);
float quadchar2float(unsigned char const* qc);
float quadchar2float_(unsigned char const* qc);
void adjustCPU(int* const cpu, int const processor);
float nullSensor(int const value);
float linkTouch(int const value);
float kunweiTech(int const value);

class SwapNode{
public:
    unsigned char* memPtr;
    SwapNode* previous, * next;
    SwapNode(int const size);
    ~SwapNode();
};

class SwapList{
public:
    std::atomic<SwapNode*> nodePtr;
    SwapList(int const size);
    void advanceNodePtr();
    void copyTo(unsigned char* const domainPtr, int const domainSize);
    void copyFrom(unsigned char const* domainPtr, int const domainSize);
    ~SwapList();
};

struct SDOMsg{
#ifndef NIIC
    ec_sdo_request_t* sdoHandler;
#else
    ecat::sdo_request* sdoHandler;
#endif
    long value;
    int alias;
    short state;                // -2: skipped; -1: error; 0: pending; 1, 2: processing; 3: completed
    unsigned short index;
    unsigned char subindex;
    unsigned char signed_;      // 0: unsigned; 1: signed
    unsigned char bitLength;    // 8, 16 or 32
    unsigned char operation;    // 0: write; 1: read
    int recycled;
};

struct REGMsg{
#ifndef NIIC
    ec_reg_request_t* regHandler;
#else
    int slave;
#endif
    long value;
    int alias;
    int recycled;
};

struct DriverRXData{
    int TargetPosition;
    int TargetVelocity;
    short TargetTorque;         // kd (can)
    unsigned short ControlWord; // kp (can)
    signed char Mode;
    signed char Undefined;      // enabled (can)
    short TorqueOffset;
    int VelocityOffset;
};

struct DriverTXData{
    int ActualPosition;
    int ActualVelocity;
    short ActualTorque;
    unsigned short StatusWord;
    signed char ModeDisplay;    // temperatureRotor (can)
    signed char Undefined;      // temperatureMOS (can)
    unsigned short ErrorCode;
};

class DriverParameters{
public:
    float minP, maxP, minV, maxV, minKp, maxKp, minKd, maxKd, minT, maxT, maxC, gearRatio, tConstant, pUnit, targetVUnit, actualVUnit, vOffsetUnit, targetCUnit, actualCUnit, cOffsetUnit, transType;
    DriverParameters();
    void load(std::string const& type);
    ~DriverParameters();
};

class MotorParameters{
public:
    float polarity, countBias, encoderResolution, gearRatioTor, gearRatioPosVel, ratedCurrent, torqueConstant, ratedTorque, maximumTorque, minimumPosition, maximumPosition;
    SDOMsg sdoTemplate, temperatureSDO, clearErrorSDO;
    MotorParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler);
    void load(ecat::sdo_request* const sdoHandler);
#endif
    ~MotorParameters();
};

struct IMURXData{
    int ControlCode;
};

struct IMUTXData{
    float rpy[3];
    float gyr[3];
    float acc[3];
    float q  [4];
    bool quaternion;
};

class IMUParameters{
public:
    float x, y, z;
    Eigen::Quaternionf* q;
    bool transform;
    IMUParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler);
    void load(ecat::sdo_request* const sdoHandler);
#endif
    ~IMUParameters();
};

struct DigitRXData{
    unsigned short TargetPosition;
};

struct DigitTXData{
    unsigned short ActualPosition;
};

struct HandRXData{
    unsigned char stop;
    char Undefined;
    unsigned short TargetSpeedThumb;
    unsigned short TargetSpeedThumbBend;
    unsigned short TargetSpeedForefinger;
    unsigned short TargetSpeedMiddle;
    unsigned short TargetSpeedRing;
    unsigned short TargetSpeedLittle;
    unsigned short TargetAngleThumb;
    unsigned short TargetAngleThumbBend;
    unsigned short TargetAngleForefinger;
    unsigned short TargetAngleMiddle;
    unsigned short TargetAngleRing;
    unsigned short TargetAngleLittle;
    unsigned short CurrentLimitThumb;
    unsigned short CurrentLimitThumbBend;
    unsigned short CurrentLimitForefinger;
    unsigned short CurrentLimitMiddle;
    unsigned short CurrentLimitRing;
    unsigned short CurrentLimitLittle;
};

struct HandTXData{
    unsigned short TouchSensorThumb[4];
    unsigned short TouchSensorForefinger[4];
    unsigned short TouchSensorMiddle[4];
    unsigned short TouchSensorRing[4];
    unsigned short TouchSensorLittle[4];
    unsigned short ActualAngleThumb;
    unsigned short ActualAngleThumbBend;
    unsigned short ActualAngleForefinger;
    unsigned short ActualAngleMiddle;
    unsigned short ActualAngleRing;
    unsigned short ActualAngleLittle;
    unsigned short ActualCurrentThumb;
    unsigned short ActualCurrentThumbBend;
    unsigned short ActualCurrentForefinger;
    unsigned short ActualCurrentMiddle;
    unsigned short ActualCurrentRing;
    unsigned short ActualCurrentLittle;
};

struct ConverterChannel{
    unsigned short Index;
    unsigned short ID;
    unsigned short Length;
    unsigned char Data[64];
};

struct ConverterRXData{
    ConverterChannel channels[8];
};

struct ConverterTXData{
    ConverterChannel channels[8];
};

class EffectorParameters{
public:
    EffectorParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler);
    void load(ecat::sdo_request* const sdoHandler);
#endif
    ~EffectorParameters();
};

struct SensorRXData{
    int ControlCode;
    float x;
    float y;
    float z;
    float a;
    float b;
    float c;
    float d;
};

struct SensorTXData{
    int Fx;
    int Fy;
    int Fz;
    int Mx;
    int My;
    int Mz;
    unsigned int StatusCode;
    unsigned int SampleCounter;
    int Temper;
};

class SensorParameters{
public:
    SensorParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler);
    void load(ecat::sdo_request* const sdoHandler);
#endif
    ~SensorParameters();
};

struct __attribute__((__packed__)) TransferrerSlot{
    unsigned int ID;
    unsigned char RTR;  // rtr or canfd << 1 | eff
    unsigned char DLC;
    unsigned char Byte[8];
};

struct __attribute__((__packed__)) TransferrerRXData_{
    unsigned char Count;
    unsigned char IDE;  // eff
    TransferrerSlot slots[6];
};

struct __attribute__((__packed__)) TransferrerTXData_{
    unsigned char Count;
    unsigned char IDE;
    TransferrerSlot slots[6];
};

struct __attribute__((__packed__)) TransferrerRXData{
    TransferrerSlot slots[64];
};

struct __attribute__((__packed__)) TransferrerTXData{
    TransferrerSlot slots[64];
};

class TransferrerParameters{
public:
    int dof, canfd;
    TransferrerParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler);
    void load(ecat::sdo_request* const sdoHandler);
#endif
    ~TransferrerParameters();
};

template<typename Data>
class DataWrapper
{
public:
    Data* data;
    int offset;
    SwapList* swap;
    DataWrapper(){
        data = new Data();
        memset(data, 0, sizeof(Data));
        offset = -1;
        swap = nullptr;
    }
    void init(int const offset){
        this->offset = offset;
    }
    void config(SwapList* const swap){
        this->swap = swap;
    }
    Data* operator->(){
        if(swap != nullptr){
            return (Data*)(swap->nodePtr.load()->memPtr + offset);
        }
        return data;
    }
    Data* previous(){
        if(swap != nullptr){
            return (Data*)(swap->nodePtr.load()->previous->memPtr + offset);
        }
        return data;
    }
    Data* current(){
        if(swap != nullptr){
            return (Data*)(swap->nodePtr.load()->memPtr + offset);
        }
        return data;
    }
    Data* next(){
        if(swap != nullptr){
            return (Data*)(swap->nodePtr.load()->next->memPtr + offset);
        }
        return data;
    }
    ~DataWrapper(){
        delete data;
    }
};

template<typename RXData, typename TXData, typename Parameters>
class WrapperPair{
public:
    int busCode, order, domain, slave, alias, enabled;
    std::string bus, type;
    DataWrapper<RXData> rx;
    DataWrapper<TXData> tx;
#ifndef NIIC
    ec_sdo_request_t* sdoHandler;
    ec_reg_request_t* regHandler;
#else
    ecat::sdo_request* sdoHandler;
#endif
    Parameters parameters;
    WrapperPair(){
        busCode = -1;   // 0: ECAT; 1: CAN; 2: CANopen; 3: CANEmu; 4: RS-485; 5: RS-232
        order = -1;
        domain = -1;
        slave = -1;
        alias = 0;
        enabled = 0;
        bus = "";
        type = "";
        sdoHandler = nullptr;
#ifndef NIIC
        regHandler = nullptr;
#endif
    }
#ifndef NIIC
    int init(std::string const& bus, int const busCode, int const order, int const domain, int const slave, int const alias, std::string const& type, int const rxOffset, int const txOffset, ec_sdo_request_t* const sdoHandler, ec_reg_request_t* const regHandler){
#else
    int init(std::string const& bus, int const busCode, int const order, int const domain, int const slave, int const alias, std::string const& type, int const rxOffset, int const txOffset){
#endif
        if(this->order != -1){
            printf("trying to re-init %s slave %d:%d with alias %d\n", bus.c_str(), order, slave, alias);
            return -1;
        }
        this->busCode = busCode;
        this->order = order;
        this->domain = domain;
        this->slave = slave;
        this->alias = alias;
        this->bus = bus;
        this->type = type;
        rx.init(rxOffset);
        tx.init(txOffset);
#ifndef NIIC
        this->sdoHandler = sdoHandler;
        this->regHandler = regHandler;
#endif
        return 0;
    }
    int config(std::string const& bus, int const order, int const domain, SwapList* const rxSwap, SwapList* const txSwap){
        if(this->order == -1){
            return 2;
        }
        if(this->bus != bus || this->order != order || this->domain != domain){
            return 1;
        }
        rx.config(rxSwap);
        tx.config(txSwap);
        if(parameters.load(bus, alias, type, sdoHandler) != 0){
            printf("loading parameters failed for %s slave %d:%d with alias %d\n", bus.c_str(), order, slave, alias);
            return -1;
        }
        return 0;
    }
#ifdef NIIC
    int config(std::string const& bus, int const order, int const domain, ecat::sdo_request* const sdoHandler){
        if(this->order == -1){
            return 2;
        }
        if(this->bus != bus || this->order != order || this->domain != domain){
            return 1;
        }
        parameters.load(sdoHandler);
        return 0;
    }
#endif
    ~WrapperPair(){
    }
};
}