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
#ifdef XENOMAI
#include <alchemy/task.h>
#include <alchemy/pipe.h>
#endif

namespace DriverSDK{
using sensorFunction = float (*)(int const);

float nullSensor(int const value);
float linkTouch(int const value);
float kunweiTech(int const value);

class ECAT{
public:
    bool dc, refSlave, sdoRequestable, regRequestable;
    int order, fd, localEffectorAlias, localSensorAlias, localTransferrerCount, * domainSizes, cpu;
    static int effectorAlias, sensorAlias;
    std::atomic<int> stalled;
    std::map<int, std::string> alias2type;
    std::vector<std::tuple<std::vector<int>, int, std::string>> aliases2domain2type;
    long period;
    std::map<int, int> alias2slave, alias2domain;
    std::vector<int> domainDivisions;
    std::vector<bool> domainWatchdogs;
    std::vector<std::vector<int>> domain2transferrers;
    ec_domain_t** domains;
    unsigned char** domainPtrs;
    SwapList** rxPDOSwaps, ** txPDOSwaps;
    PtrQue<SDOMsg, 256> sdoRequestQueue, regRequestQueue;
    PtrQue<SDOMsg, 1024> sdoResponseQueue, regResponseQueue;
    ec_master_t* master;
#ifdef XENOMAI
    std::atomic<bool> running;
    RT_TASK task;
    int terminal;
    RT_PIPE pipe;
#else
    pthread_t pth;
#endif
    ECAT(int const order);
    int init();
    int readAlias(unsigned short const slave, std::string const& category, unsigned short const index, unsigned char const subindex, unsigned char const bitLength);
    int requestState(unsigned short const slave, char const* stateString);
    int check();
    int config();
#ifdef XENOMAI
    static void rxtx(void* arg);
#else
    static void* rxtx(void* arg);
#endif
    int run();
    void clean();
    ~ECAT();
    ECAT(ECAT const&&);
};
}