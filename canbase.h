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

#include "common.h"
#include <mutex>
#include <limits>

namespace DriverSDK{
class CAN;
class CANEmu;

using canRXFunction = int (*)(int const, int const, int* const, unsigned char* const);
using canTXFunction = void (*)(int const, int const, unsigned char* const, int const, CAN* const);
using canEmuRXFunction = int (*)(int const, int const, int* const, unsigned char* const);
using canEmuTXFunction = void (*)(int const, int const, unsigned char* const, int const, CANEmu* const);

extern int dofAll, dofLeftEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;

unsigned char const EncosEnable   [3] = {0x71, 0x03, 0xe8};
unsigned char const EncosDisable  [3] = {0x6d, 0x00, 0x00};
unsigned char const EncosDamp     [3] = {0x69, 0x00, 0x00};
unsigned char const DamiaoEnable  [8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
unsigned char const DamiaoDisable [8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
unsigned char const DamiaoClrErr  [8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
unsigned char const RealManIAP    [3] = {0x02, 0x49, 0x00};
unsigned char const RealManEnable [3] = {0x02, 0x0a, 0x01};
unsigned char const RealManDisable[3] = {0x02, 0x0a, 0x00};
unsigned char const RealManClrErr [3] = {0x02, 0x0f, 0x01};
unsigned char const AgibotData    [8] = {0x00, 0x00, 0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x00};

unsigned short float2para(float const f, float const min, float const max, int const bit);
float para2float(unsigned short const us, float const min, float const max, int const bit);

int nullRX(int const order, int const alias, int* const slaveID, unsigned char* const data);

void nullCANTX(int const order, int const masterID, unsigned char* const data, int const length, CAN* const can);
void nullCANEmuTX(int const order, int const masterID, unsigned char* const data, int const length, CANEmu* const canemu);

template<typename T>
int encosRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 2:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0007:
            memcpy(data, EncosDamp, 3);
            return 3;
            break;
        case 0x0001:
            break;
        }
        break;
    case 1:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0007:
            break;
        case 0x0001:
            memcpy(data, EncosEnable, 3);
            T::alias2status[alias] = 0x0007;
            return 3;
            break;
        }
        break;
    case 0:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0007:
            memcpy(data, EncosDisable, 3);
            T::alias2status[alias] = 0x0001;
            return 3;
            break;
        case 0x0001:
            break;
        }
        break;
    }
    DriverParameters const* parameters = T::alias2parameters[alias];
    unsigned short p, v, t, kp, kd;
    if((T::alias2status[alias] & 0x0f7f) != 0x0007){
         p = float2para(0.0, parameters->minP,  parameters->maxP,  16);
         v = float2para(0.0, parameters->minV,  parameters->maxV,  12);
        kp = float2para(0.0, parameters->minKp, parameters->maxKp, 12);
        kd = float2para(0.0, parameters->minKd, parameters->maxKd,  9);
         t = float2para(0.0, parameters->minT,  parameters->maxT,  12);
    }else{
         p = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetPosition, parameters->minP,  parameters->maxP,  16);
         v = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetVelocity, parameters->minV,  parameters->maxV,  12);
        kp = float2para(half2single(drivers[alias - 1].rx.previous()->ControlWord),   parameters->minKp, parameters->maxKp, 12);
        kd = float2para(half2single(drivers[alias - 1].rx.previous()->TargetTorque),  parameters->minKd, parameters->maxKd,  9);
         t = float2para(half2single(drivers[alias - 1].rx.previous()->TorqueOffset),  parameters->minT,  parameters->maxT,  12);
    }
    *(data + 0) = kp >> 7;
    *(data + 1) = kp << 1 & 0x00ff | kd >> 8;
    *(data + 2) = kd & 0x00ff;
    *(data + 3) =  p >> 8;
    *(data + 4) =  p & 0x00ff;
    *(data + 5) =  v >> 4;
    *(data + 6) =  v << 4 & 0x00ff |  t >> 8;
    *(data + 7) =  t & 0x00ff;
    return 8;
}

template<typename T>
void encosTX(int const order, int const masterID, unsigned char* const data, int const length, T* const can){
    if(length != 8){
        return;
    }
    unsigned char err = data[0] & 0x1f;
    data[0] = data[2];
    unsigned short p = *(unsigned short*)(data + 0);
    data[2] = data[4];
    unsigned short v = *(unsigned short*)(data + 2);
    v >>= 4;
    data[3] = data[5];
    data[4] = data[4] & 0x0f;
    unsigned short t = *(unsigned short*)(data + 3);
    int const slaveID = T::orderMasterID2slaveID[order][masterID], alias = T::orderSlaveID2alias[order][slaveID];
    DriverParameters const* parameters = T::alias2parameters[alias];
    signed char temperature = (data[6] - 50) / 2;
    bool error = err > 0 && (err != 1 && err != 2 && err != 4 || err == 1 && temperature > 99);
    if(temperature > 80){
        T::alias2status[alias] |= 0x0080;
    }
    *(float*)&drivers[alias - 1].tx.next()->ActualPosition =             para2float(p, parameters->minP, parameters->maxP, 16);
    *(float*)&drivers[alias - 1].tx.next()->ActualVelocity =             para2float(v, parameters->minV, parameters->maxV, 12);
              drivers[alias - 1].tx.next()->ActualTorque   = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
              drivers[alias - 1].tx.next()->Undefined      = temperature;
              drivers[alias - 1].tx.next()->StatusWord     = error ? 0x0008 : T::alias2status[alias];
              drivers[alias - 1].tx.next()->ErrorCode      = error ? err : 0x0000;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    long current = TIMEVAL2US(tv);
    static long previous[8] = {current, current, current, current, current, current, current, current};
    can->mask |= 1 << slaveID;
    if(can->mask == can->MASK){
        if((T::alias2status[alias] & 0x0f7f) == 0x0000){
            int i = 0;
            while(i < 32){
                int a = T::orderSlaveID2alias[order][i];
                if(a > 0 && a <= dofAll){
                    T::alias2status[a] = 0x0001;
                }
                i++;
            }
        }
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        previous[order] = current;
    }else if(current - previous[order] > 20 * can->period / 1000){
        unsigned int x = can->mask ^ can->MASK;
        int i = 0;
        while(i < 32){
            if((x & 1 << i) > 0){
                int a = T::orderSlaveID2alias[order][i];
                T::alias2status[a] |= 0x4000;
                drivers[a - 1].tx.next()->StatusWord = T::alias2status[a];
            }
            i++;
        }
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        previous[order] = current;
    }
}

template<typename T>
int damiaoRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 1:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0007:
            break;
        case 0x0001:
            memcpy(data, DamiaoEnable, 8);
            T::alias2status[alias] = 0x0007;
            return 8;
            break;
        }
        break;
    case 0:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0007:
            memcpy(data, DamiaoDisable, 8);
            T::alias2status[alias] = 0x0001;
            return 8;
            break;
        case 0x0001:
            break;
        }
        break;
    case -1:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0007:
            break;
        case 0x0001:
            memcpy(data, DamiaoClrErr, 8);
            return 8;
            break;
        }
        break;
    }
    DriverParameters const* parameters = T::alias2parameters[alias];
    unsigned short  p = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetPosition, parameters->minP,  parameters->maxP,  16);
    unsigned short  v = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetVelocity, parameters->minV,  parameters->maxV,  12);
    unsigned short kp = float2para(half2single(drivers[alias - 1].rx.previous()->ControlWord),   parameters->minKp, parameters->maxKp, 12);
    unsigned short kd = float2para(half2single(drivers[alias - 1].rx.previous()->TargetTorque),  parameters->minKd, parameters->maxKd, 12);
    unsigned short  t = float2para(half2single(drivers[alias - 1].rx.previous()->TorqueOffset),  parameters->minT,  parameters->maxT,  12);
    *(data + 0) =  p >> 8;
    *(data + 1) =  p & 0x00ff;
    *(data + 2) =  v >> 4;
    *(data + 3) =  v << 4 & 0x00ff | kp >> 8;
    *(data + 4) = kp & 0x00ff;
    *(data + 5) = kd >> 4;
    *(data + 6) = kd << 4 & 0x00ff |  t >> 8;
    *(data + 7) =  t & 0x00ff;
    return 8;
}

template<typename T>
void damiaoTX(int const order, int const masterID, unsigned char* const data, int const length, T* const can){
    if(length != 8){
        return;
    }
    unsigned char err = data[0] >> 4;
    // int const slaveID = data[0] & 0x0f;
    data[0] = data[2];
    unsigned short p = *(unsigned short*)(data + 0);
    data[2] = data[4];
    unsigned short v = *(unsigned short*)(data + 2);
    v >>= 4;
    data[3] = data[5];
    data[4] = data[4] & 0x0f;
    unsigned short t = *(unsigned short*)(data + 3);
    int const slaveID = T::orderMasterID2slaveID[order][masterID], alias = T::orderSlaveID2alias[order][slaveID];
    DriverParameters const* parameters = T::alias2parameters[alias];
    signed char temperature = data[7];
    bool error = err > 1;
    if(temperature > 80){
        T::alias2status[alias] |= 0x0080;
    }
    *(float*)&drivers[alias - 1].tx.next()->ActualPosition =             para2float(p, parameters->minP, parameters->maxP, 16);
    *(float*)&drivers[alias - 1].tx.next()->ActualVelocity =             para2float(v, parameters->minV, parameters->maxV, 12);
              drivers[alias - 1].tx.next()->ActualTorque   = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
              drivers[alias - 1].tx.next()->Undefined      = temperature;
              drivers[alias - 1].tx.next()->StatusWord     = error ? 0x0008 : T::alias2status[alias];
              drivers[alias - 1].tx.next()->ErrorCode      = error ? err : 0x0000;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    long current = TIMEVAL2US(tv);
    static long previous[8] = {current, current, current, current, current, current, current, current};
    can->mask |= 1 << slaveID;
    if(can->mask == can->MASK){
        if((T::alias2status[alias] & 0x0f7f) == 0x0000){
            int i = 0;
            while(i < 32){
                int a = T::orderSlaveID2alias[order][i];
                if(a > 0 && a <= dofAll){
                    T::alias2status[a] = 0x0001;
                }
                i++;
            }
        }
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        previous[order] = current;
    }else if(current - previous[order] > 20 * can->period / 1000){
        unsigned int x = can->mask ^ can->MASK;
        int i = 0;
        while(i < 32){
            if((x & 1 << i) > 0){
                int a = T::orderSlaveID2alias[order][i];
                T::alias2status[a] |= 0x4000;
                drivers[a - 1].tx.next()->StatusWord = T::alias2status[a];
            }
            i++;
        }
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        previous[order] = current;
    }
}

template<typename T>
int realManRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 1:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0250:
            T::alias2status[alias] = 0x0007;
        case 0x0007:
            break;
        case 0x0001:
            memcpy(data, RealManEnable, 3);
            return 3;
            break;
        case 0x0050:
            *slaveID += 0x0600;
            return 0;
            break;
        case 0x0000:
            memcpy(data, RealManIAP, 3);
            return 3;
            break;
        }
        break;
    case 0:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0250:
        case 0x0007:
            memcpy(data, RealManDisable, 3);
            return 3;
            break;
        case 0x0001:
            break;
        case 0x0050:
            *slaveID += 0x0600;
            return 0;
            break;
        case 0x0000:
            memcpy(data, RealManIAP, 3);
            return 3;
            break;
        }
        break;
    case -1:
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0250:
            memcpy(data, RealManDisable, 3);
            return 3;
            break;
        case 0x0007:
            break;
        case 0x0001:
            memcpy(data, RealManClrErr, 3);
            return 3;
            break;
        case 0x0050:
            *slaveID += 0x0600;
            return 0;
            break;
        case 0x0000:
            memcpy(data, RealManIAP, 3);
            return 3;
            break;
        }
        break;
    }
    unsigned char* const data_ = data + 64;
    int length = std::numeric_limits<int>::min(), index = (*slaveID - 1) * 8;
    static bool enabled[8] = {false, false, false, false, false, false, false, false};
    if(*slaveID == 1){
        enabled[order] = true;
    }
    if((T::alias2status[alias] & 0x0f7f) != 0x0007){
        enabled[order] = false;
    }
    if(*slaveID == 1){
        memset(data_, 0x00, 56);
        memset(data_ + 56, 0xff, 7);
        data_[63] = 0xaf;
    }else if(*slaveID == 7){
        if(enabled[order]){
            *slaveID = 0x02f;
            length = -64;
        }else{
            *slaveID = 0x07f;
            memset(data_, 0x00, 23);
            data_[23] = 0x0f;
            return -24;
        }
    }
    DriverParameters const* parameters = T::alias2parameters[alias];
    *(  int*)(data_ + index + 0) =   *(float*)&drivers[alias - 1].rx.previous()->TargetPosition * 180.0 / Pi / parameters->pUnit;
    *(short*)(data_ + index + 4) =   *(float*)&drivers[alias - 1].rx.previous()->VelocityOffset * 30.0 / Pi / parameters->vOffsetUnit;
    *(short*)(data_ + index + 6) = half2single(drivers[alias - 1].rx.previous()->TorqueOffset) / parameters->tConstant / parameters->cOffsetUnit;
    return length;
}

template<typename T>
void realManTX(int const order, int const masterID, unsigned char* const data, int const length, T* const can){
    int const slaveID = T::orderMasterID2slaveID[order][masterID], alias = T::orderSlaveID2alias[order][slaveID];
    DriverParameters const* parameters = T::alias2parameters[alias];
    if(masterID > 0x700){
        if(length != 16){
            return;
        }
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0050:
            T::alias2status[alias] = 0x0250;
            signed char temperature = *(short*)(data + 4) * 0.1;
            unsigned short err = *(unsigned short*)(data + 0);
            if(temperature > 80){
                T::alias2status[alias] |= 0x0080;
            }
            *(float*)&drivers[alias - 1].tx->ActualPosition = *(int*)(data + 8) * parameters->pUnit * Pi / 180.0;
            *(float*)&drivers[alias - 1].tx->ActualVelocity = 0.0;
                      drivers[alias - 1].tx->ActualTorque   = single2half(*(int*)(data + 12) * parameters->actualCUnit * parameters->tConstant);
                      drivers[alias - 1].tx->Undefined      = temperature;
                      drivers[alias - 1].tx->StatusWord     = err > 0 ? 0x0008 : T::alias2status[alias];
                      drivers[alias - 1].tx->ErrorCode      = err;
            break;
        }
        return;
    }else if(masterID > 0x100 && masterID != 0x5fe){
        if(length != 3){
            return;
        }
        switch(T::alias2status[alias] & 0x0f7f){
        case 0x0250:
        case 0x0007:
            if(data[1] == 0x0a){
                T::alias2status[alias] = 0x0001;
            }
            break;
        case 0x0001:
            if(data[1] == 0x0a){
                T::alias2status[alias] = 0x0007;
            }
            break;
        case 0x0000:
            if(data[1] == 0x49 && data[2] == 0x01){
                T::alias2status[alias] = 0x0050;
            }
            break;
        }
        return;
    }else if(masterID > 0x081){
        if(length != 24){
            return;
        }
    }
    signed char temperature = *(short*)(data + 16) * 0.1;
    unsigned short err = *(unsigned short*)(data + 12);
    if(temperature > 80){
        T::alias2status[alias] |= 0x0080;
    }
    *(float*)&drivers[alias - 1].tx.next()->ActualPosition = *(int*)(data + 8) * parameters->pUnit * Pi / 180.0;
    *(float*)&drivers[alias - 1].tx.next()->ActualVelocity = *(int*)(data + 4) * parameters->actualVUnit * Pi / 30.0;
              drivers[alias - 1].tx.next()->ActualTorque   = single2half(*(int*)(data + 0) * parameters->actualCUnit * parameters->tConstant);
              drivers[alias - 1].tx.next()->Undefined      = temperature;
              drivers[alias - 1].tx.next()->StatusWord     = err > 0 ? 0x0008 : T::alias2status[alias];
              drivers[alias - 1].tx.next()->ErrorCode      = err;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    long current = TIMEVAL2US(tv);
    static long previous[8] = {current, current, current, current, current, current, current, current};
    can->mask |= 1 << slaveID;
    if(can->mask == can->MASK){
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        previous[order] = current;
    }else if(current - previous[order] > 20 * can->period / 1000){
        unsigned int x = can->mask ^ can->MASK;
        int i = 0;
        while(i < 32){
            if((x & 1 << i) > 0){
                int a = T::orderSlaveID2alias[order][i];
                T::alias2status[a] |= 0x4000;
                drivers[a - 1].tx.next()->StatusWord = T::alias2status[a];
            }
            i++;
        }
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        previous[order] = current;
    }
}

template<typename T>
int agibotRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    memcpy(data, AgibotData, 8);
    data[1] = (90 - digits[i].rx.previous()->TargetPosition) * 255 / 90;
    return 8;
}

template<typename T>
void agibotTX(int const order, int const masterID, unsigned char* const data, int const length, T* const can){
    if(length != 8){
        return;
    }
    int const slaveID = T::orderMasterID2slaveID[order][masterID], alias = T::orderSlaveID2alias[order][slaveID];
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    digits[i].tx.next()->ActualPosition = (255 - data[2]) * 90 / 255;
    can->mask_ |= 1 << slaveID;
    if(can->mask_ == can->MASK_){
        can->txSwap_->advanceNodePtr();
        can->mask_ = 0;
    }
}

template<typename T>
int linkerBotRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    data[0] = 0x01;
    int j = 0;
    while(j < 6){
        data[j + 1] = (90 - digits[i + j].rx.previous()->TargetPosition) * 255 / 90;
        j++;
    }
    return 7;
}

template<typename T>
void linkerBotTX(int const order, int const masterID, unsigned char* const data, int const length, T* const can){
    if(length != 7){
        return;
    }
    int const slaveID = T::orderMasterID2slaveID[order][masterID], alias = T::orderSlaveID2alias[order][slaveID];
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    int j = 0;
    while(j < 6){
        digits[i + j].tx.next()->ActualPosition = (255 - data[j + 1]) * 90 / 255;
        j++;
    }
    can->mask_ |= 1 << slaveID;
    if(can->mask_ == can->MASK_){
        can->txSwap_->advanceNodePtr();
        can->mask_ = 0;
    }
}

class CANBase{
public:
    int order, canhal, autoRestart, baudrate, canfd, dbaudrate, division, sock, slaveCount;
    char* device;
    static long period;
    static int CANHAL;
    static std::mutex resourceMutex;
    CANBase(int const order, char const* device);
    int ifaceIsUp();
    int ifaceUp();
    int ifaceUp_();
    int ifaceDown();
    int open(int const masterID);
    int send(int const slaveID, unsigned char const* data, int const length);
    int recv(unsigned char* const data, int const length, int* const masterID);
    int sendfd(int const slaveID, unsigned char const* data, int const length);
    int recvfd(unsigned char* const data, int const length, int* const masterID);
    ~CANBase();
};

class CAN : public CANBase{
public:
    std::map<int, std::string> alias2type;
    std::map<int, std::vector<int>> alias2masterIDs;
    std::map<int, int> alias2slaveID;
    SwapList* rxSwap, * txSwap, * rxSwap_, * txSwap_;
    static pthread_t rxPth, txPth, txPth_;
    static int rxCPU, txCPU, txCPU_;
    static std::map<std::string, DriverParameters*> type2parameters;
    static unsigned short* alias2status;
    static DriverParameters** alias2parameters;
    static int orderSlaveID2alias[8][256];
    static int orderMasterID2slaveID[8][2048];
    static canRXFunction rxFuncs[8][256];
    static canTXFunction txFuncs[8][2048];
    unsigned int MASK, mask, MASK_, mask_;
    unsigned char rollingCounter;
    CAN(int const order, char const* device);
    int config();
    static void cleanup(void* arg);
    static void* rx(void* arg);
    static void* tx(void* arg);
    static void cleanup_(void* arg);
    static void* tx__(void* arg);
    static void* tx_(void* arg);
    static int run(std::vector<CAN>& cans);
    ~CAN();
};

class CANEmu : public CANBase{
public:
    std::map<int, std::string> alias2type;
    std::map<int, std::vector<int>> alias2masterIDs;
    std::map<int, int> alias2slaveID;
    SwapList* rxSwap, * txSwap, * rxSwap_, * txSwap_;
    static std::map<std::string, DriverParameters*> type2parameters;
    static unsigned short* alias2status;
    static DriverParameters** alias2parameters;
    static int orderSlaveID2alias[8][256];
    static int orderMasterID2slaveID[8][2048];
    static canEmuRXFunction rxFuncs[8][256];
    static canEmuTXFunction txFuncs[8][2048];
    static int alias2channel[256];
    unsigned int MASK, mask, MASK_, mask_;
    CANEmu(int const order);
    int config();
    static int run(std::vector<CANEmu>& canemus);
    ~CANEmu();
};
}