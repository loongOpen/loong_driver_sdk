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

#include "ptr_que.h"
#include "common.h"

namespace DriverSDK{
class ECAT{
public:
    bool dc, sdoRequestable, regRequestable;
    int order, effectorAlias, sensorAlias, * domainSizes, cpu, domainCount, * workingCounters, previousIncompleteness, tryCount;
    std::map<int, std::string> alias2type;
    std::string eni;
    long period;
    std::map<int, int> alias2slave, alias2domain;
    std::vector<int> domainDivisions;
    SwapList** rxPDOSwaps, ** txPDOSwaps;
    SDOMsg* sdoMsg;
    REGMsg* regMsg;
    PtrQue<SDOMsg> sdoRequestQueue, sdoResponseQueue, regRequestQueue, regResponseQueue;
    ecat::task* task;
    ecat::master* master;
    ecat::PdoRegInfo** targetPosition, ** targetVelocity, ** targetTorque, ** controlWord, ** mode, ** torqueOffset, ** velocityOffset, ** actualPosition, ** actualVelocity, ** actualTorque, ** statusWord, ** modeDisplay, ** errorCode;
    ecat::domain_state* domainStates;
    ecat::wc_state_type* wcStates;
    ECAT(int const order);
    int init();
    int readAlias(unsigned short const slave, std::string const& category, unsigned short const index, unsigned char const subindex, unsigned char const bitLength);
    int requestState(unsigned short const slave, char const* stateString);
    int check();
    int config();
    int run();
    void clean();
    ~ECAT();
};
}