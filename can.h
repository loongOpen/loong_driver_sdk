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

namespace DriverSDK{
class CAN;
using canRXFunction = int (*)(int const, int const, int* const, unsigned char* const);
using canTXFunction = void (*)(int const, int const, unsigned char* const, int const, CAN* const);

class DriverParameters{
public:
    float minP, maxP, minV, maxV, minKp, maxKp, minKd, maxKd, minT, maxT, pUnit, targetVUnit, actualVUnit, vOffsetUnit, targetCUnit, actualCUnit, cOffsetUnit, tConstant;
    DriverParameters();
    int load(std::string const& type);
    ~DriverParameters();
};

class CANBase{
public:
    int order, canhal, baudrate, canfd, dbaudrate, division, sock, slaveCount;
    char* device;
    static long period;
    static int CANHAL;
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
    SwapList* rxSwap, * txSwap;
    static pthread_t rxPth, txPth, txPth_;
    static int rxCPU, txCPU, txCPU_;
    static std::map<std::string, DriverParameters*> type2parameters;
    static unsigned short* alias2status;
    static DriverParameters** alias2parameters;
    static int orderSlaveID2alias[8][16];
    static int orderMasterID2slaveID[8][2048];
    static canRXFunction rxFuncs[8][16];
    static canTXFunction txFuncs[8][2048];
    unsigned short MASK, mask;
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
}