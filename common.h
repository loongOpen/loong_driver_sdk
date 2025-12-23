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
#define TIMESPEC2NS(T) (T.tv_sec * NSEC_PER_SEC + T.tv_nsec)

float const Pi = std::acos(-1);

unsigned short single2half(float f);
float half2single(unsigned short u);
void adjustCPU(int* cpu, int processor);

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
    void copyTo(unsigned char* domainPtr, int const domainSize);
    void copyFrom(unsigned char const* domainPtr, int const domainSize);
    ~SwapList();
};

struct SDOMsg{
#ifndef NIIC
    ec_sdo_request_t* sdoHandler;
#else
    int slave;
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

struct DriverRxData{
    int TargetPosition;
    int TargetVelocity;
    short TargetTorque;         // kd (can)
    unsigned short ControlWord; // kp (can)
    char Mode;
    signed char Undefined;      // enabled (can)
    short TorqueOffset;
    int VelocityOffset;
};

struct DriverTxData{
    int ActualPosition;
    int ActualVelocity;
    short ActualTorque;
    unsigned short StatusWord;
    char ModeDisplay;
    signed char Undefined;      // temp (can)
    unsigned short ErrorCode;
};

class MotorParameters{
public:
    float polarity, countBias, encoderResolution, gearRatioTor, gearRatioPosVel, ratedCurrent, torqueConstant, ratedTorque, maximumTorque, minimumPosition, maximumPosition;
    SDOMsg sdoTemplate, temperatureSDO, clearErrorSDO;
    MotorParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, int const slave);
#endif
    ~MotorParameters();
};

struct HandRxData{
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

struct HandTxData{
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

struct DigitRxData{
    unsigned short TargetPosition;
};

struct DigitTxData{
    unsigned short ActualPosition;
};

struct ConverterDatum{
    unsigned short Index;
    unsigned short ID;
    unsigned short Length;
    unsigned char Data[64];
};

struct ConverterRxData{
    ConverterDatum channels[8];
};

struct ConverterTxData{
    ConverterDatum channels[8];
};

class EffectorParameters{
public:
    EffectorParameters();
#ifndef NIIC
    int load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler);
#else
    int load(std::string const& bus, int const alias, std::string const& type, int const slave);
#endif
    ~EffectorParameters();
};

struct SensorRxData{
    int ControlCode;
    float x;
    float y;
    float z;
    float a;
    float b;
    float c;
    float d;
};

struct SensorTxData{
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
    int load(std::string const& bus, int const alias, std::string const& type, int const slave);
#endif
    ~SensorParameters();
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
    Data* previous(){
        if(swap != nullptr){
            return (Data*)(swap->nodePtr.load()->previous->memPtr + offset);
        }
        return data;
    }
    Data* operator->(){
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

template<typename RxData, typename TxData, typename Parameters>
class WrapperPair{
public:
    int busCode, order, domain, slave, alias, enabled;
    std::string bus, type;
    DataWrapper<RxData> rx;
    DataWrapper<TxData> tx;
#ifndef NIIC
    ec_sdo_request_t* sdoHandler;
    ec_reg_request_t* regHandler;
#endif
    Parameters parameters;
    WrapperPair(){
        busCode = -1;
        order = -1;
        domain = -1;
        slave = -1;
        alias = 0;
        enabled = 0;
        bus = "";
        type = "";
#ifndef NIIC
        sdoHandler = nullptr;
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
#ifndef NIIC
        if(parameters.load(bus, alias, type, sdoHandler) < 0){
#else
        if(parameters.load(bus, alias, type, slave) < 0){
#endif
            printf("loading parameters failed for %s slave %d:%d with alias %d\n", bus.c_str(), order, slave, alias);
            return -1;
        }
        return 0;
    }
    ~WrapperPair(){
    }
};
}