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

namespace DriverSDK{
#define NSEC_PER_SEC 1000000000L
#define USEC_PER_SEC 1000000L
#define TIMESPEC2NS(T) (T.tv_sec * NSEC_PER_SEC + T.tv_nsec)
#define TIMEVAL2US(T) (T.tv_sec * USEC_PER_SEC + T.tv_usec)

float const Pi = std::acos(-1);

void print(unsigned char const* data, int const length);
unsigned short single2half(float const f);
float half2single(unsigned short const u);
int quadchar2int(unsigned char const* qc);
int quadchar2int_(unsigned char const* qc);
float quadchar2float(unsigned char const* qc);
float quadchar2float_(unsigned char const* qc);
void adjustCPU(int* const cpu, int const processor);

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
    short state;                // -1: error; 0: pending; 1, 2: processing; 3: completed
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
    char Mode;
    signed char Undefined;      // enabled (can)
    short TorqueOffset;
    int VelocityOffset;
};

struct DriverTXData{
    int ActualPosition;
    int ActualVelocity;
    short ActualTorque;
    unsigned short StatusWord;
    char ModeDisplay;
    signed char Undefined;      // temp (can)
    unsigned short ErrorCode;
};

class DriverParameters{
public:
    float minP, maxP, minV, maxV, minKp, maxKp, minKd, maxKd, minT, maxT, pUnit, targetVUnit, actualVUnit, vOffsetUnit, targetCUnit, actualCUnit, cOffsetUnit, tConstant;
    DriverParameters();
    int load(std::string const& type);
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
#endif
    ~MotorParameters();
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
#endif
    ~SensorParameters();
};

struct __attribute__((__packed__)) TransferrerChannel{
    unsigned int ID;
    unsigned char RTR;
    unsigned char DLC;
    unsigned char Byte[8];
};

struct __attribute__((__packed__)) TransferrerRXData{
    unsigned char Count;
    unsigned char IDE;
    TransferrerChannel channels[6];
};

struct __attribute__((__packed__)) TransferrerTXData{
    unsigned char Count;
    unsigned char IDE;
    TransferrerChannel channels[6];
};

class TransferrerParameters{
public:
    TransferrerParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler);
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
        busCode = -1;   // 0: ECAT; 1: CAN; 2: CANopen; 3: CANEmu; 4: RS-485
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
        parameters.sdoTemplate   .sdoHandler = sdoHandler;
        parameters.temperatureSDO.sdoHandler = sdoHandler;
        parameters.clearErrorSDO .sdoHandler = sdoHandler;
        return 0;
    }
#endif
    ~WrapperPair(){
    }
};
}