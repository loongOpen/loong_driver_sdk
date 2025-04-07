/* Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司
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
    bool dc, sdoRequestable;
    int order, fd;
    std::map<int, std::string> alias2type;
    long period;
    ec_master_t* master;
    std::map<int, int> alias2slave;
    ec_domain_t* domain;
    unsigned char* domainPtr;
    int domainSize;
    SwapList* rxPDOSwap, * txPDOSwap;
    PtrQue<SDOMsg> sdoRequestQueue, sdoResponseQueue;
    pthread_t pth;
    ECAT(int const order);
    int init();
    int readAlias(unsigned short const slave, std::string const& type, unsigned short const index, unsigned char const subindex, unsigned char const bitLength);
    int requestState(unsigned short const slave, char const* stateString);
    int check();
    int config();
    static void* rxtx(void* arg);
    int run();
    void clean();
    ~ECAT();
};
}